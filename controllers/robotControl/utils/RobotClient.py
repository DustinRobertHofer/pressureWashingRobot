import socket
import json
import threading
import time
from utils.RobotInterface import RobotInterface

class RobotClient:
    def __init__(self, host="127.0.0.1", port=5000, max_retries=5, retry_delay=2):
        self.host = host
        self.port = port
        self.socket = None
        self.robot_interface = RobotInterface.get_instance()
        self.boundary_points = None
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self.connected = False
        self.cleaning_started = False  # Flag to track if cleaning should start

    def get_cleaning_status(self):
        """Get the current cleaning status"""
        return self.cleaning_started

    def connect(self):
        """Connect to the server with retry logic"""
        for attempt in range(self.max_retries):
            try:
                self.socket = socket.socket()
                self.socket.connect((self.host, self.port))
                print(f"Connected to server at {self.host}:{self.port}")
                self.connected = True
                return True
            except ConnectionRefusedError:
                print(f"Connection attempt {attempt + 1}/{self.max_retries} failed. Server not available.")
                if attempt < self.max_retries - 1:
                    print(f"Retrying in {self.retry_delay} seconds...")
                    time.sleep(self.retry_delay)
                else:
                    print("Max retries reached. Please ensure the server is running.")
                    return False
            except Exception as e:
                print(f"Unexpected error during connection: {e}")
                return False

    def receive_data(self):
        """Receive and process data from the server"""
        if not self.connected:
            print("Not connected to server. Cannot receive data.")
            return

        try:
            while self.connected:
                data = self.socket.recv(1024)
                if not data:
                    print("Server closed connection")
                    break
                
                try:
                    decoded_data = data.decode('utf-8')
                    print(f"Received raw data: {decoded_data}")
                    
                    # Parse the received data
                    message = json.loads(decoded_data)
                    
                    # Handle different message types
                    if 'type' in message:
                        if message['type'] == 'area_data':
                            # Convert the points to the format expected by robotControl
                            self.boundary_points = []
                            for point in message['points']:
                                # Convert string coordinates to float
                                x = float(point['x'])
                                y = float(point['y'])
                                self.boundary_points.append({'x': x, 'y': y})
                            
                            # Pass the boundary points through the interface
                            self.robot_interface.set_boundary_points(self.boundary_points)
                            print("Boundary points updated through interface")
                            
                        elif message['type'] == 'command' and message['action'] == 'start_cleaning':
                            print("Received start cleaning command")
                            self.cleaning_started = True
                            
                except json.JSONDecodeError:
                    print("Error: Received invalid JSON data")
                    
        except Exception as e:
            print(f"Error in receive_data: {e}")
        finally:
            self.connected = False
            if self.socket:
                self.socket.close()
            print("Connection closed")

    def start(self):
        """Start the client and begin listening for data"""
        if self.connect():
            # Start a thread to receive data
            receive_thread = threading.Thread(target=self.receive_data)
            receive_thread.daemon = True  # Thread will exit when main program exits
            receive_thread.start()
            print("Client started and listening for data...")
        else:
            print("Failed to start client")

if __name__ == "__main__":
    client = RobotClient()
    client.start()
    # Keep the main thread running
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Client shutting down...") 