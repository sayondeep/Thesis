import socket
import struct

# Server address and port
server_address = 'localhost'
server_port = 12345

# Connect to the server
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((server_address, server_port))

while True:
    # Get location details from the user
    x = float(input("Enter X: "))
    y = float(input("Enter Y: "))

    # Send the location details to the server
    #client_socket.sendall(struct.pack('dd', x, y))
    client_socket.send(struct.pack('dd', x, y))

    # Receive the tile info from the server
    tile_info = client_socket.recv(1024).decode()

    # Display the tile info
    print("Tile Info:", tile_info)

    # Check if the user wants to exit
    choice = input("Enter 1 to give location or 0 to exit: ")
    if choice == '0':
        break

# Close the connection
client_socket.close()
