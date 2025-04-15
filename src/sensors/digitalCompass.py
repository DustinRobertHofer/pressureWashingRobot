import math
import time
import smbus
from config.robotConfig import SENSOR_PINS

class PiCompass:
    """Digital compass implementation for Raspberry Pi using HMC5883L."""
    
    def __init__(self):
        """Initialize digital compass with I2C interface."""
        try:
            self.i2c_bus = smbus.SMBus(1)  # Use I2C bus 1 on Raspberry Pi
            self.compass_addr = SENSOR_PINS['compass']['address']
            
            # Initialize the compass
            self._setup_compass()
            
            # Calibration values
            self.x_offset = 0
            self.y_offset = 0
            self.x_scale = 1.0
            self.y_scale = 1.0
            
            # Initial heading
            self.heading = 0.0
            self.last_reading_time = 0
            self.max_reading_age = 0.1  # Max age of reading in seconds
            
            self.compass_available = True
            time.sleep(0.1)  # Allow compass to stabilize
            print("Digital compass initialized successfully")
            
        except Exception as e:
            print(f"Failed to initialize digital compass: {e}")
            self.compass_available = False
    
    def _setup_compass(self):
        """Setup and initialize the HMC5883L digital compass."""
        # HMC5883L Register addresses
        CONFIG_A = 0x00
        CONFIG_B = 0x01
        MODE = 0x02
        
        # Configuration values
        # CONFIG_A: 8 samples averaged, 15Hz data output rate, normal measurement
        self.i2c_bus.write_byte_data(self.compass_addr, CONFIG_A, 0x70)
        
        # CONFIG_B: Gain=1.3, +/- 1.3Ga range
        self.i2c_bus.write_byte_data(self.compass_addr, CONFIG_B, 0x20)
        
        # MODE: Continuous measurement mode
        self.i2c_bus.write_byte_data(self.compass_addr, MODE, 0x00)
        
        # Wait for first measurement
        time.sleep(0.1)
    
    def _read_raw_data(self):
        """Read raw magnetometer data from compass."""
        try:
            # HMC5883L data registers
            DATA_X_MSB = 0x03
            
            # Read 6 bytes (X, Z, Y axes - note the unconventional order)
            data = self.i2c_bus.read_i2c_block_data(self.compass_addr, DATA_X_MSB, 6)
            
            # Convert the data to 16-bit signed values
            x = self._twos_complement(data[0] << 8 | data[1], 16)
            z = self._twos_complement(data[2] << 8 | data[3], 16)
            y = self._twos_complement(data[4] << 8 | data[5], 16)
            
            return x, y, z
            
        except Exception as e:
            print(f"Error reading compass data: {e}")
            return None, None, None
    
    def _twos_complement(self, val, bits):
        """Convert an unsigned integer in 2's complement form to a signed integer."""
        if (val & (1 << (bits - 1))) != 0:
            val = val - (1 << bits)
        return val
    
    def get_bearing(self):
        """Get current heading in radians."""
        if not self.compass_available:
            return 0.0  # Return default if compass not available
            
        current_time = time.time()
        
        # Check if we need a new reading
        if current_time - self.last_reading_time > self.max_reading_age:
            self._update_heading()
            self.last_reading_time = current_time
            
        return self.heading
        
    def _update_heading(self):
        """Update the current heading from compass data."""
        x, y, z = self._read_raw_data()
        
        if x is None or y is None:
            return
            
        # Apply calibration
        x_calibrated = (x - self.x_offset) * self.x_scale
        y_calibrated = (y - self.y_offset) * self.y_scale
        
        # Calculate heading - arctangent of y/x
        # Note: atan2 returns angle in range (-pi, pi)
        heading = math.atan2(y_calibrated, x_calibrated)
        
        # Convert to 0-2π range
        if heading < 0:
            heading += 2 * math.pi
            
        self.heading = heading
    
    def calibrate(self):
        """Calibrate the compass by collecting min/max values."""
        if not self.compass_available:
            print("Compass not available for calibration")
            return
            
        print("Calibrating digital compass...")
        print("Please rotate the robot in a full circle slowly (15 seconds)...")
        
        # Initialize min/max values
        min_x = min_y = 32767
        max_x = max_y = -32768
        
        start_time = time.time()
        while time.time() - start_time < 15:
            x, y, z = self._read_raw_data()
            
            if x is not None and y is not None:
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                min_y = min(min_y, y)
                max_y = max(max_y, y)
                
            time.sleep(0.1)
        
        # Calculate offsets (center of ellipse)
        self.x_offset = (min_x + max_x) / 2
        self.y_offset = (min_y + max_y) / 2
        
        # Calculate scaling factors to transform ellipse to circle
        x_range = max_x - min_x
        y_range = max_y - min_y
        
        # Avoid division by zero
        if x_range == 0:
            x_range = 1
        if y_range == 0:
            y_range = 1
            
        # Calculate average range
        avg_range = (x_range + y_range) / 2
            
        # Scale factors
        self.x_scale = avg_range / x_range
        self.y_scale = avg_range / y_range
        
        print("Compass calibration complete.")
        print(f"Offsets: X={self.x_offset}, Y={self.y_offset}")
        print(f"Scaling: X={self.x_scale}, Y={self.y_scale}")
    
    def cleanup(self):
        """Clean up resources."""
        # Nothing specific to clean up for I2C devices
        pass 