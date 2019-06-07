#!/usr/bin/env python

import socket
import sys
import os
import logging
import time
import json

class TCP:
	def __init__(self):
		# Create a TCP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		#self._my_ip = os.popen('ip addr show wifi0').read().split("inet ")[1].split("/")[0]
		self._my_ip = '128.84.127.47'

		self.hsiip = '127.0.0.1'
		# Connect the socket to the port on the server given by the caller
		if(len(sys.argv)>1):
			self.server_address = (sys.argv[1], 65432)
		else:
			self.server_address = (self.hsiip, 65432)

	def __del__(self):
		self.sock.close()
	
	def getIP(self):
		# get aero board ip
		#os.system('scp pi@132.236.59.220:/home/pi/MEng/meng-hsi-group/tcp/ip.txt .')
		#file = open("ip.txt", "r")
		#content = file.readlines()[-1]
		#print(content)
		#self.hsiip = content.split()[1]
		self.hsiip = '10.148.8.230'
		self.server_address = (self.hsiip, 65432)
		
	def connect(self):
		#print >>sys.stderr, 'connecting to %s port %s' % self.server_address
		
		try:
			self.sock.connect(self.server_address)
		except:
			logging.error('could not connect socket')
	
	def sendData(self, data):
		try:		    
			self.sock.sendall(data)
		    #print >>sys.stderr, 'sending "%s"' % data

		#    amount_received = 0
		#    amount_expected = len(data)
		#    while amount_received < amount_expected:
		#	recv_data = self.sock.recv(len(data))
		#	amount_received += len(recv_data)
		#	print >>sys.stderr, 'received "%s"' % recv_data
		
		except:
			logging.error('excpetion caught in tcp.sendd_data')
		finally:
			pass

	def recvData(self):
		try:
			# self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
			self.sock.bind((self._my_ip, 65432))
			self.sock.listen(1)

			while True:
				logging.info('waiting for a connection')
				connection, client_address = self.sock.accept()

				while True:
					print('Connected by ', client_address)
					data = connection.recv(50)
					if data:
						print("The data is " + data)
					else:
						break
		except:
			logging.error("The data couldn't received")

	def closeSocket(self):
		self.sock.close()

if __name__ == "__main__":

	FORMAT = '[%(asctime)s] p%(process)s {%(pathname)s:%(lineno)d} %(levelname)s - %(message)s'
	logging.basicConfig(format=FORMAT, level=logging.INFO)

	tcp = TCP()
	tcp.getIP()
	tcp.connect()
	tcp.sendData(json.dumps({"cmd":"SetSpeed"}).encode())
	tcp.closeSocket()
	
	# tcp.recvData()
	# tcp.closeSocket()
	
