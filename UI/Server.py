# server.py
import socket
import threading

listenSocket = socket.socket()
Port = 8000
maxConnections = 999
IP = socket.gethostname()

listenSocket.bind(('', Port))

listenSocket.listen(maxConnections)
print("Server Started at " + IP + " on Port " + str(Port))

(clientsocket, address) = listenSocket.accept()
print("New Connection Made " + str(address))

running = True

while running:
    message = clientsocket.recv(1024).decode()
    print(message)


