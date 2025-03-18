# server.py
import socket
import threading
import json

# Global list to store all connected clients
clients = []

def broadcast_data(data, exclude_client=None):
    """Broadcast data to all connected clients except the excluded one"""
    for client in clients[:]:  # Create a copy of the list to avoid modification during iteration
        if client != exclude_client:  # Skip the excluded client
            try:
                client.sendall(data.encode('utf-8'))
            except Exception as e:
                print(f"Error sending data to client: {e}")
                clients.remove(client)

def handle_area_data(data):
    """Handle area data messages"""
    try:
        area_data = json.loads(data)
        if area_data['type'] == 'area_data':
            print(f"Received area data:")
            print(f"Shape: {area_data['shape']}")
            print(f"Points: {area_data['points']}")
            return True
    except json.JSONDecodeError:
        print("Error: Received invalid JSON data")
    except KeyError:
        print("Error: Missing required fields in area data")
    return False

def handle_command(data):
    """Handle command messages"""
    try:
        command = json.loads(data)
        if command['type'] == 'command':
            print(f"Received command: {command['action']}")
            return True
    except json.JSONDecodeError:
        print("Error: Received invalid JSON data")
    except KeyError:
        print("Error: Missing required fields in command")
    return False

def handle_client(conn, addr):
    print(f"Connected by {addr}")
    clients.append(conn)  # Add client to the list
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            
            # Try to decode the received data
            try:
                decoded_data = data.decode('utf-8')
                print(f"Received: {decoded_data}")
                
                # Parse the message
                message = json.loads(decoded_data)
                
                # Handle different message types
                if 'type' in message:
                    if message['type'] == 'area_data':
                        print(f"Processing area data:")
                        print(f"Shape: {message['shape']}")
                        print(f"Points: {message['points']}")
                        
                        # Send success response to the original sender
                        response = json.dumps({
                            'status': 'success',
                            'message': 'Area data received successfully'
                        })
                        conn.sendall(response.encode('utf-8'))
                        
                        # Broadcast area data to other clients (not the sender)
                        for client in clients:
                            if client != conn:  # Don't send back to the original sender
                                try:
                                    client.sendall(decoded_data.encode('utf-8'))
                                except Exception as e:
                                    print(f"Error sending data to client: {e}")
                                    if client in clients:
                                        clients.remove(client)
                    elif message['type'] == 'command':
                        print(f"Processing command: {message['action']}")
                        
                        # Send success response to the original sender
                        response = json.dumps({
                            'status': 'success',
                            'message': f"Command {message['action']} received successfully"
                        })
                        conn.sendall(response.encode('utf-8'))
                        
                        # Broadcast command to other clients (not the sender)
                        for client in clients:
                            if client != conn:  # Don't send back to the original sender
                                try:
                                    client.sendall(decoded_data.encode('utf-8'))
                                except Exception as e:
                                    print(f"Error sending command to client: {e}")
                                    if client in clients:
                                        clients.remove(client)
                    else:
                        # Only send error response to the original sender if it's not a recognized message type
                        print(f"Received unknown message type: {message['type']}")
                        response = json.dumps({
                            'status': 'error',
                            'message': f"Unknown message type: {message['type']}"
                        })
                        conn.sendall(response.encode('utf-8'))
                else:
                    print("Message missing type field")
                    response = json.dumps({
                        'status': 'error',
                        'message': 'Message missing type field'
                    })
                    conn.sendall(response.encode('utf-8'))
                    
            except json.JSONDecodeError:
                print("Error: Received invalid JSON data")
                conn.sendall(b"Error: Invalid data format")
            except Exception as e:
                print(f"Error processing message: {e}")
                response = json.dumps({
                    'status': 'error',
                    'message': f'Error processing message: {str(e)}'
                })
                conn.sendall(response.encode('utf-8'))
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if conn in clients:
            clients.remove(conn)  # Remove client from the list
        conn.close()
        print(f"Connection with {addr} closed")

def server_program():
    host = "127.0.0.1"
    port = 5000

    server_socket = socket.socket()
    # Add socket reuse options
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    
    try:
        server_socket.bind((host, port))
        server_socket.listen(5)
        print("Server started. Waiting for connections...")

        while True:
            conn, address = server_socket.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, address))
            client_thread.start()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        server_socket.close()

if __name__ == '__main__':
    server_program()