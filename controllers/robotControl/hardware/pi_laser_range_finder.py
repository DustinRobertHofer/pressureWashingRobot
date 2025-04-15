import time
import serial
import smbus
from hardware.pi_config import SENSOR_PINS

class PiLaserRangeFinder:
    """Laser range finder implementation for Raspberry Pi."""
    
    def __init__(self):
        """Initialize laser range finder with serial or I2C interface."""
        self.config = SENSOR_PINS['laser_range']
        self.interface_type = self._detect_interface_type()
        
        # Initialize the appropriate interface
        if self.interface_type == 'serial':
            self._init_serial()
        elif self.interface_type == 'i2c':
            self._init_i2c()
        else:
            print("No supported interface detected for laser range finder")
            self.interface_available = False
            return
            
        # Configuration parameters
        self.last_reading = None
        self.last_reading_time = 0
        self.max_reading_age = 0.2  # Maximum age of reading in seconds
        
        print(f"Laser range finder initialized using {self.interface_type} interface")
        self.interface_available = True
    
    def _detect_interface_type(self):
        """Detect which interface to use for the laser range finder."""
        # If UART port is specified, use serial
        if self.config['uart_port']:
            return 'serial'
        
        # If I2C address is specified and not 0, use I2C
        if self.config['i2c_address'] and self.config['i2c_address'] != 0:
            return 'i2c'
        
        # If GPIO pins are specified for software serial, use serial
        if self.config['rx_pin'] and self.config['tx_pin']:
            return 'serial'
            
        # Default to I2C as it's more common for laser range finders
        return 'i2c'
    
    def _init_serial(self):
        """Initialize serial communication for the laser range finder."""
        try:
            # Use hardware UART if available, otherwise use specified pins (would require additional software serial implementation)
            port = self.config['uart_port']
            
            # Common baud rates for laser range finders: 9600, 115200
            self.serial = serial.Serial(port, 115200, timeout=0.5)
            time.sleep(0.1)  # Allow serial port to initialize
            
        except Exception as e:
            print(f"Error initializing laser range finder serial interface: {e}")
            self.interface_available = False
    
    def _init_i2c(self):
        """Initialize I2C communication for the laser range finder."""
        try:
            self.i2c_bus = smbus.SMBus(1)  # Use I2C bus 1 on Raspberry Pi
            self.i2c_addr = self.config['i2c_address']
            
            # Set up the laser range finder (specific to the device type)
            # These settings are for a TFmini/VL53L0X type sensor
            # For different sensors, would need to adjust initialization sequence
            
            # Example: Write to configuration register (device-specific)
            # self.i2c_bus.write_byte_data(self.i2c_addr, 0x00, 0x01)  # Start continuous ranging
            
            time.sleep(0.1)  # Allow sensor to initialize
            
        except Exception as e:
            print(f"Error initializing laser range finder I2C interface: {e}")
            self.interface_available = False
    
    def get_distance(self):
        """Get distance reading in meters."""
        if not hasattr(self, 'interface_available') or not self.interface_available:
            return None
            
        current_time = time.time()
        
        # Check if we need a new reading
        if (self.last_reading is None or 
            current_time - self.last_reading_time > self.max_reading_age):
            self.last_reading = self._read_distance()
            self.last_reading_time = current_time
            
        return self.last_reading
    
    def _read_distance(self):
        """Read distance from laser range finder."""
        if self.interface_type == 'serial':
            return self._read_distance_serial()
        elif self.interface_type == 'i2c':
            return self._read_distance_i2c()
        return None
    
    def _read_distance_serial(self):
        """Read distance using serial interface."""
        try:
            # Clear any stale data
            if self.serial.in_waiting > 0:
                self.serial.reset_input_buffer()
            
            # Send trigger command if needed (device specific)
            # For TFmini: no command needed for continuous mode
            # For some devices: self.serial.write(b'\x55')  # Example trigger command
            
            # Read data - format depends on the specific laser range finder
            # For TFmini: 9-byte packet
            # For other devices: might be different format
            
            # Generic approach - read a chunk of data
            data = self.serial.read(9)  # Adjust size based on device protocol
            
            if len(data) < 9:
                return None
                
            # Parse data according to device protocol
            # Example for TFmini:
            if data[0] == 0x59 and data[1] == 0x59:
                distance = data[2] + data[3] * 256
                return distance / 100.0  # Convert to meters
                
            return None
            
        except Exception as e:
            print(f"Error reading from laser range finder serial: {e}")
            return None
    
    def _read_distance_i2c(self):
        """Read distance using I2C interface."""
        try:
            # The exact implementation depends on the specific laser range finder model
            # Example for VL53L0X:
            # 1. Write to register to trigger measurement if needed
            # self.i2c_bus.write_byte_data(self.i2c_addr, 0x00, 0x01)
            
            # 2. Wait for data to be ready (device specific)
            # time.sleep(0.05)
            
            # 3. Read distance registers
            # Read 2 bytes from register 0x14-0x15 (VL53L0X range data)
            # data = self.i2c_bus.read_i2c_block_data(self.i2c_addr, 0x14, 2)
            # distance = (data[0] << 8) | data[1]
            
            # Simplified placeholder implementation
            # Replace with actual code for your specific laser range finder
            data_high = self.i2c_bus.read_byte_data(self.i2c_addr, 0x0F)
            data_low = self.i2c_bus.read_byte_data(self.i2c_addr, 0x10)
            
            distance = (data_high << 8) | data_low
            
            # Convert to meters
            return distance / 100.0  # Assuming distance in cm
            
        except Exception as e:
            print(f"Error reading from laser range finder I2C: {e}")
            return None
    
    def cleanup(self):
        """Clean up resources."""
        if hasattr(self, 'serial') and self.serial:
            self.serial.close()
        # No specific cleanup needed for I2C 