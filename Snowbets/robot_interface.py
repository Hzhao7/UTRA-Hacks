"""
Robot Interface - Serial communication with Arduino robot
Reads sensor data and broadcasts to betting platform
"""

import serial
import serial.tools.list_ports
import threading
import time
import re
from dataclasses import dataclass, asdict
from typing import Optional, Callable, List


@dataclass
class RobotData:
    """Current robot sensor readings"""
    timestamp: float
    
    # Color sensor
    red: int = 0
    green: int = 0
    blue: int = 0
    detected_color: str = "unknown"
    
    # PID data
    error: float = 0.0
    derivative: float = 0.0
    pd_output: float = 0.0
    
    # Motor speeds
    left_speed: int = 0
    right_speed: int = 0
    
    # Ultrasonic
    distance: float = 0.0
    
    # Status
    is_connected: bool = False
    last_update: float = 0.0
    
    def to_dict(self):
        return asdict(self)


class RobotInterface:
    """
    Interface to communicate with Arduino robot over serial
    Parses sensor data and provides it to betting engine
    """
    
    # Color detection thresholds (from simple_line_follow.cpp)
    LINE_RED_MIN, LINE_RED_MAX = 23, 37
    LINE_GREEN_MIN, LINE_GREEN_MAX = 95, 115
    LINE_BLUE_MIN, LINE_BLUE_MAX = 75, 100
    
    WHITE_RED, WHITE_GREEN, WHITE_BLUE = 17, 18, 16
    
    def __init__(self, port: str = None, baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.running = False
        self.read_thread: Optional[threading.Thread] = None
        
        self.data = RobotData(timestamp=time.time())
        self.data_callbacks: List[Callable[[RobotData], None]] = []
        
        # Data history for analysis
        self.history: List[RobotData] = []
        self.max_history = 100
        
        # Simulation state for realistic continuous data
        self._sim_position = 0.0  # Position along virtual track (0-100)
        self._sim_drift = 0.0     # How far off the line (-1 to 1)
        self._sim_speed_target = 120
    
    @staticmethod
    def list_ports() -> List[str]:
        """List available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]
    
    def connect(self, port: str = None) -> bool:
        """Connect to robot serial port"""
        if port:
            self.port = port
        
        if not self.port:
            # Try to auto-detect
            ports = self.list_ports()
            if ports:
                self.port = ports[0]
            else:
                return False
        
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            self.data.is_connected = True
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from robot"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2)
        if self.serial:
            self.serial.close()
        self.data.is_connected = False
    
    def start_reading(self):
        """Start background thread to read serial data"""
        if not self.serial:
            return False
        
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        return True
    
    def _read_loop(self):
        """Background loop to read and parse serial data"""
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        self._parse_line(line)
            except Exception as e:
                print(f"Serial read error: {e}")
            time.sleep(0.01)
    
    def _parse_line(self, line: str):
        """Parse a line of serial output from the robot"""
        now = time.time()
        
        # Parse PD control output format:
        # "G: 102 Err: -95.40 D: 0.00 PD: -333.90 L: 255 R: 0"
        pd_match = re.search(
            r'G:\s*(\d+)\s*Err:\s*([-\d.]+)\s*D:\s*([-\d.]+)\s*PD:\s*([-\d.]+)\s*L:\s*(\d+)\s*R:\s*(\d+)',
            line
        )
        
        if pd_match:
            self.data.green = int(pd_match.group(1))
            self.data.error = float(pd_match.group(2))
            self.data.derivative = float(pd_match.group(3))
            self.data.pd_output = float(pd_match.group(4))
            self.data.left_speed = int(pd_match.group(5))
            self.data.right_speed = int(pd_match.group(6))
            self.data.timestamp = now
            self.data.last_update = now
        
        # Parse RGB calibration output format:
        # "R: 24  G: 102  B: 80"
        rgb_match = re.search(r'R:\s*(\d+)\s*G:\s*(\d+)\s*B:\s*(\d+)', line)
        if rgb_match:
            self.data.red = int(rgb_match.group(1))
            self.data.green = int(rgb_match.group(2))
            self.data.blue = int(rgb_match.group(3))
            self.data.timestamp = now
            self.data.last_update = now
        
        # Determine detected color
        self.data.detected_color = self._classify_color(
            self.data.red, self.data.green, self.data.blue
        )
        
        # Store in history
        self.history.append(RobotData(**asdict(self.data)))
        if len(self.history) > self.max_history:
            self.history.pop(0)
        
        # Notify callbacks
        for callback in self.data_callbacks:
            try:
                callback(self.data)
            except Exception as e:
                print(f"Callback error: {e}")
    
    def _classify_color(self, r: int, g: int, b: int) -> str:
        """Classify the detected color based on RGB values"""
        # Check for white (low values)
        if r <= 20 and g <= 25 and b <= 20:
            return "white"
        
        # Check for red line (normal red)
        if (23 <= r <= 26 and 95 <= g <= 110 and 75 <= b <= 85):
            return "red"
        
        # Check for dark red
        if (33 <= r <= 37 and 107 <= g <= 115 and 91 <= b <= 100):
            return "dark_red"
        
        # Extended red range (combined)
        if (self.LINE_RED_MIN <= r <= self.LINE_RED_MAX and
            self.LINE_GREEN_MIN <= g <= self.LINE_GREEN_MAX and
            self.LINE_BLUE_MIN <= b <= self.LINE_BLUE_MAX):
            return "red"
        
        return "unknown"
    
    def add_callback(self, callback: Callable[[RobotData], None]):
        """Add a callback for data updates"""
        self.data_callbacks.append(callback)
    
    def remove_callback(self, callback: Callable[[RobotData], None]):
        """Remove a callback"""
        if callback in self.data_callbacks:
            self.data_callbacks.remove(callback)
    
    def get_current_data(self) -> RobotData:
        """Get current robot data"""
        return self.data
    
    def get_average_error(self, seconds: float = 5.0) -> float:
        """Get average PID error over last N seconds"""
        cutoff = time.time() - seconds
        recent = [d.error for d in self.history if d.timestamp > cutoff]
        return sum(abs(e) for e in recent) / len(recent) if recent else 0.0
    
    def get_speed_category(self) -> str:
        """Get current speed category"""
        avg_speed = (self.data.left_speed + self.data.right_speed) / 2
        if avg_speed == 0:
            return "stopped"
        elif avg_speed <= 80:
            return "slow"
        elif avg_speed <= 150:
            return "medium"
        else:
            return "fast"
    
    def is_on_line(self) -> bool:
        """Check if robot is currently on the line"""
        return self.data.detected_color in ["red", "dark_red"]
    
    def simulate_data(self):
        """Generate realistic simulated data that changes smoothly over time"""
        import random
        import math
        
        # Simulate robot moving along a track with curves
        # Position advances, drift oscillates to simulate line following
        self._sim_position = (self._sim_position + 0.5) % 100
        
        # Create realistic drift pattern - sometimes on line, sometimes correcting
        # Add some noise and occasional larger deviations
        drift_change = random.gauss(0, 0.08)
        
        # Simulate track curves at certain positions
        if 20 < self._sim_position < 30 or 60 < self._sim_position < 75:
            # Sharp turn section - more drift
            drift_change += random.choice([-0.15, 0.15])
        
        self._sim_drift = max(-1, min(1, self._sim_drift + drift_change))
        
        # PD control tries to correct drift
        correction = -self._sim_drift * 0.3
        self._sim_drift += correction
        
        # Determine if on line based on drift magnitude
        on_line = abs(self._sim_drift) < 0.6
        
        if on_line:
            # On line - interpolate between red and dark red based on position
            if random.random() < 0.7:
                # Normal red
                self.data.red = 24 + random.randint(-2, 2)
                self.data.green = 102 + random.randint(-5, 5)
                self.data.blue = 80 + random.randint(-4, 4)
                self.data.detected_color = "red"
            else:
                # Dark red section
                self.data.red = 35 + random.randint(-2, 2)
                self.data.green = 111 + random.randint(-4, 4)
                self.data.blue = 95 + random.randint(-4, 4)
                self.data.detected_color = "dark_red"
        else:
            # Off line - white surface
            self.data.red = 17 + random.randint(-2, 2)
            self.data.green = 18 + random.randint(-2, 2)
            self.data.blue = 16 + random.randint(-2, 2)
            self.data.detected_color = "white"
        
        # Calculate realistic PID values based on drift
        self.data.error = self._sim_drift * 80 + random.gauss(0, 5)
        
        # Derivative based on change (smoothed)
        if self.history:
            prev_error = self.history[-1].error
            self.data.derivative = (self.data.error - prev_error) * 0.8 + random.gauss(0, 2)
        else:
            self.data.derivative = random.gauss(0, 3)
        
        # PD output
        self.data.pd_output = (3.5 * self.data.error) + (2.0 * self.data.derivative)
        
        # Motor speeds based on PD output
        base = self._sim_speed_target
        self.data.left_speed = max(0, min(255, int(base - self.data.pd_output * 0.5)))
        self.data.right_speed = max(0, min(255, int(base + self.data.pd_output * 0.5)))
        
        # Ultrasonic - mostly clear with occasional obstacles
        if random.random() < 0.05:
            self.data.distance = random.uniform(5, 15)  # Obstacle detected
        else:
            self.data.distance = random.uniform(25, 45)  # Clear
        
        self.data.timestamp = time.time()
        self.data.last_update = time.time()
        self.data.is_connected = True
        
        # Store in history
        self.history.append(RobotData(**asdict(self.data)))
        if len(self.history) > self.max_history:
            self.history.pop(0)
        
        # Notify callbacks
        for callback in self.data_callbacks:
            try:
                callback(self.data)
            except Exception as e:
                print(f"Callback error: {e}")


# Global robot interface instance
robot = RobotInterface()
