import socket
import sys

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('192.168.100.50', 0))

# Connect the socket to the port where the server is listening
server_address = ('192.168.100.103', 502)
print('connecting to %s port %s' % server_address)
sock.connect(server_address)

# Send data
message = 'Huy'
print('sending "%s"' % message)
sock.sendall(message.encode('utf-8'))

# Look for the response
amount_received = 0
amount_expected = len(message)
    
while amount_received < amount_expected:
    data = sock.recv(16)
    amount_received += len(data)
    print('received "%s"' % data)

print('closing socket')
sock.close()
