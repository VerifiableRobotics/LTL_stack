#!/usr/bin/env python

import socket
import sys
import os

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address given on the command line
if(len(sys.argv) > 1):
    server_name = sys.argv[1]
else:
    server_name = os.popen('ip addr show eth0').read().split("inet ")[1].split("/")[0]
server_address = (server_name, 65432)
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)
sock.listen(1)

hsiip = "127.0.0.1"

while True:
    print >>sys.stderr, 'waiting for a connection'
    connection, client_address = sock.accept()
    try:
        print >>sys.stderr, 'client connected:', client_address
        while True:
            data = connection.recv(50)
            print >>sys.stderr, 'received "%s"' % data
            if data:
                if data.split()[0] == 'hsi':
                    davidip = data.split()[1]            
                    with open("ip.txt", 'a') as out_file:
                        out_file.write(data + "\n")
            if data == 'hsiip':
                send = str("hsiIP from server ") + davidip
                connection.sendall(send)
            elif data:
                connection.sendall(data)
            else:
                break
                
    finally:
        connection.close()
