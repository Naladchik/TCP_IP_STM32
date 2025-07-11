import socket
import sys

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('localhost', 10000)
print('starting up on %s port %s' % server_address)
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

while True:
    # Wait for a connection
    print('waiting for a connection')
    connection, client_address = sock.accept()
    print('connection from', client_address)
while True:
    data = connection.recv(16)
    print('received "%s"' % data)
    if data:
        print('sending data back to the client')
        connection.sendall(data.encode('utf-8'))
    else:
        print('no more data from', client_address)
        break
        
connection.close()
