import socket


class wirelessClient:

    global s 
    s = socket.socket()

    def connectToServer(ip, port):
        connectionStatus = 0

        try:
            print(f"Connecting to {ip}:{port}")
            s.connect((ip, port))
        except socket.error as msg:
            print(f"Failed to connect: {msg}")
            connectionStatus = 0
        else:
            print(f"Successfully connected to {ip}:{port}")
            connectionStatus = 1
        
        return connectionStatus

    def sendData(data):
        s.send(data.encode())

    


# hostname = 'DESKTOP-HREQUAA'
# port = 8000

