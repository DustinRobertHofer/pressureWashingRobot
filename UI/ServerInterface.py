import socket
import json

class ServerInterface:
    def __init__(self, host='localhost', port=5000):
        self.host = host
        self.port = port
        self.socket = None

    def connect(self):
        """Connect to the server"""
        try:
            self.socket = socket.socket()
            self.socket.connect((self.host, self.port))
            print("Connected to server")
            return True
        except Exception as e:
            print(f"Failed to connect to server: {e}")
            return False

    def send_area_data(self, area_data):
        """Send area data to the server"""
        if not self.socket:
            print("Not connected to server")
            return False
        
        try:
            # Convert area_data to JSON string
            data = json.dumps(area_data)
            self.socket.sendall(data.encode('utf-8'))
            return True
        except Exception as e:
            print(f"Error sending area data: {e}")
            return False

    def close(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            self.socket = None 