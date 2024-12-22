import serial
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict
import logging
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import math
from collections import deque

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("RD03Protocol")

@dataclass
class RadarTarget:
    """Represents a single radar target's data"""
    x_coord: float      # mm, positive or negative
    y_coord: float      # mm, positive or negative
    speed: float        # cm/s, positive or negative
    distance: float     # mm, pixel distance value

class RD03Protocol:
    # Previous class constants and state definitions remain the same...
    HEADER = bytes([0xAA, 0xFF, 0x03, 0x00])
    FOOTER = bytes([0x55, 0xCC])
    TARGET_DATA_SIZE = 8
    MAX_TARGETS = 3
    
    WAITING_HEADER = 0
    READING_DATA = 1
    WAITING_FOOTER = 2
    
    # Number of positions to keep in trace history
    TRACE_LENGTH = 20

    def __init__(self, port: str, baudrate: int = 256000, enable_plot: bool = False):
        """Initialize the RD03D Protocol handler with serial port settings"""
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        self._state = self.WAITING_HEADER
        self._buffer = bytearray()
        self._header_idx = 0
        self._footer_idx = 0
        self.enable_plot = enable_plot
        self.scatter = None
        self.speed_quiver = None
        self.traces = {}  # Dictionary to store trace data for each target
        
        if self.enable_plot:
            self._setup_plot()

    def _setup_plot(self):
        """Initialize the polar plot"""
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        
        # Set plot limits for upper half only
        self.ax.set_thetamin(0)  # Start at -90 degrees
        self.ax.set_thetamax(180)   # End at 90 degrees
        
        self.ax.set_rmax(5000)
        self.ax.set_rticks([1000, 2000, 3000, 4000, 5000])
        self.ax.set_rlabel_position(-45)
        self.ax.grid(True)
        
        # Initialize empty scatter plot and traces
        self.scatter = self.ax.scatter([], [], c=[], cmap='viridis', s=100)
        self.speed_quiver = self.ax.quiver([], [], [], [], color='red', scale=50)
        self.trace_lines = {}  # Store trace line objects
        
        # Add title and labels
        self.ax.set_title("Radar Target Tracking\nRange rings every 1m")
        plt.show()

    def _update_traces(self, target_id: int, r: float, theta: float):
        """Update trace history for a target"""
        if target_id not in self.traces:
            self.traces[target_id] = {
                'r': deque(maxlen=self.TRACE_LENGTH),
                'theta': deque(maxlen=self.TRACE_LENGTH)
            }
            
        self.traces[target_id]['r'].append(r)
        self.traces[target_id]['theta'].append(theta)

    def _update_plot(self, targets: List[RadarTarget]):
        """Update the polar plot with new target data"""
        if not self.enable_plot:
            return

        # Convert Cartesian coordinates to polar
        r_values = []
        theta_values = []
        speeds = []
        u_vectors = []
        v_vectors = []
        
        # Clear old trace lines
        for line in self.trace_lines.values():
            line.remove() if line in self.ax.lines else None
        self.trace_lines.clear()

        for i, target in enumerate(targets):
            # Calculate r and theta from x, y coordinates
            r = math.sqrt(target.x_coord**2 + target.y_coord**2)
            theta = math.atan2(target.y_coord, target.x_coord)
            
            # Update trace history
            self._update_traces(i, r, theta)
            
            # Draw trace line
            if i in self.traces and len(self.traces[i]['r']) > 1:
                trace_line = self.ax.plot(
                    list(self.traces[i]['theta']),
                    list(self.traces[i]['r']),
                    'g-', alpha=0.5, linewidth=1
                )[0]
                self.trace_lines[i] = trace_line
            
            r_values.append(r)
            theta_values.append(theta)
            speeds.append(abs(target.speed))
            
            # Calculate speed vector components
            speed_scale = 10
            u = (target.speed * math.cos(theta)) / speed_scale
            v = (target.speed * math.sin(theta)) / speed_scale
            u_vectors.append(u)
            v_vectors.append(v)

        try:
            # Update scatter plot data
            if r_values:
                self.scatter.set_offsets(np.c_[theta_values, r_values])
                self.scatter.set_array(np.array(speeds))
                
                # Update quiver
                self.speed_quiver.remove()
                self.speed_quiver = self.ax.quiver(theta_values, r_values, 
                                                 u_vectors, v_vectors,
                                                 color='red', scale=50)
            else:
                # If no targets, set empty data
                self.scatter.set_offsets(np.c_[[], []])
                self.scatter.set_array(np.array([]))
                self.speed_quiver.remove()
                self.speed_quiver = self.ax.quiver([], [], [], [], 
                                                 color='red', scale=50)

            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            logger.error(f"Error updating plot: {e}")

    def _decode_coordinate(self, value: int) -> float:
        """Decode a coordinate value according to the protocol specification"""
        # Check if highest bit is set (positive/negative indicator)
        is_negative = not bool(value & 0x8000)
        # Get absolute value (15 bits)
        abs_value = value & 0x7FFF
        return -abs_value if is_negative else abs_value

    def _decode_speed(self, value: int) -> float:
        """Decode a speed value according to the protocol specification"""
        # Check if highest bit is set (positive/negative indicator)
        is_negative = not bool(value & 0x8000)
        # Get absolute value (15 bits)
        abs_value = value & 0x7FFF
        return -abs_value if is_negative else abs_value

    def _parse_target_data(self, data: bytes) -> Optional[RadarTarget]:
        """Parse 8 bytes of target data into a RadarTarget object"""
        if all(b == 0 for b in data):  # Check if target data is all zeros
            return None
            
        # Extract values (little endian)
        x_raw = int.from_bytes(data[0:2], byteorder='little')
        y_raw = int.from_bytes(data[2:4], byteorder='little')
        speed_raw = int.from_bytes(data[4:6], byteorder='little')
        distance = int.from_bytes(data[6:8], byteorder='little')

        return RadarTarget(
            x_coord=self._decode_coordinate(x_raw),
            y_coord=self._decode_coordinate(y_raw),
            speed=self._decode_speed(speed_raw),
            distance=float(distance)  # distance is unsigned
        )

    def read_frame(self) -> List[RadarTarget]:
        """Read and parse a complete data frame from the radar"""
        frame_data = bytearray()
        header_found = False
        
        while True:
            if self._serial.in_waiting:
                byte = ord(self._serial.read())
                
                if not header_found:
                    frame_data.append(byte)
                    # Check for header sequence
                    if len(frame_data) >= 4:
                        if (frame_data[-4:] == bytes([0xAA, 0xFF, 0x03, 0x00])):
                            header_found = True
                            frame_data = frame_data[-4:]  # Keep only the header
                elif header_found:
                    frame_data.append(byte)
                    
                    # Check if we have a complete frame
                    if len(frame_data) >= (4 + 24 + 2):  # Header + 3*8 bytes data + Footer
                        if frame_data[-2:] == bytes([0x55, 0xCC]):
                            # Valid frame received, parse targets
                            targets = []
                            data_start = 4  # After header
                            
                            for i in range(3):  # 3 possible targets
                                target_data = frame_data[data_start + i*8:data_start + (i+1)*8]
                                target = self._parse_target_data(target_data)
                                if target is not None:
                                    targets.append(target)
                            
                            if self.enable_plot:
                                self._update_plot(targets)
                                
                            return targets
                            
                        else:
                            # Invalid frame, start over
                            frame_data = bytearray()
                            header_found = False

    def close(self):
        """Close the serial port"""
        if self.enable_plot:
            plt.close(self.fig)
        self._serial.close()

# Example usage
if __name__ == "__main__":
    # Initialize protocol handler (adjust port name as needed)
    protocol = RD03Protocol("/dev/ttyUSB0", enable_plot=True)  # Enable plotting

    try:
        print("Reading radar data...")
        while True:
            targets = protocol.read_frame()
            if targets:
                print("\nDetected Targets:")
                for i, target in enumerate(targets, 1):
                    print(f"Target {i}:")
                    print(f"  Position: ({target.x_coord}mm, {target.y_coord}mm)")
                    print(f"  Speed: {target.speed}cm/s")
                    print(f"  Distance: {target.distance}mm")

    except KeyboardInterrupt:
        print("\nClosing serial port...")
        protocol.close()