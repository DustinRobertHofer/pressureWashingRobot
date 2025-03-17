# server.py
import socket
import threading

def handle_client(conn, addr):
    print(f"Connected by {addr}")
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print(f"Received: {data.decode('utf-8')}")
            send_to_client(conn)
            conn.sendall(data)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        conn.close()
        print(f"Connection with {addr} closed")

def send_to_client(conn):
    message = ("Hello Client").encode()
    conn.sendall(message)

def server_program():
    host = socket.gethostname()
    port = 5000

    server_socket = socket.socket()
    server_socket.bind((host, port))
    server_socket.listen(5)
    print("Server started. Waiting for connections...")

    while True:
        conn, address = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(conn, address))
        client_thread.start()

if __name__ == '__main__':
    server_program()