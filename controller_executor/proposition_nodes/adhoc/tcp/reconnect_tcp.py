#!/usr/bin/env python

import socket
import sys
import os
import threading
from threading import Timer 

class TCP:
	def __init__(self, data_handler = None):

		self.data_handler = data_handler
		
		#self.hsiip = os.popen('ip addr show wlp1s0').read().split("inet ")[1].split("/")[0]
		self.hsiip = "127.0.0.1"
		self.last_hsiip = self.hsiip

		self.pi_server_address = "132.236.59.220"
		
		# Connect the socket to the port on the server given by the caller
		if(len(sys.argv)>1):
			self.aero_address = (sys.argv[1], 65432)
		else:
			self.aero_address = (self.hsiip, 65432)
		

	def __del__(self):
		for t in threading.enumerate():
			t.cancel()
			
	def getSelfIP(self):
		try:		
			print('trying to update ip address')
			self.hsiip = os.popen('ip addr show wlp1s0').read().split("inet ")[1].split("/")[0]
		except:
			self.last_hsiip = self.hsiip
			print("Please update myself IP address")
			self.hsiip = None

                self.checkIP()

	def checkIP(self):
		
		self.check_ip_timer = Timer(3.0, self.getSelfIP)
		self.check_ip_timer.start()
		for i in threading.enumerate():
			if i.name == "MainThread":
				if not i.is_alive():
					for t in threading.enumerate():
						if t.name is not "MainThread": t.cancel()
                print("==================== check IP =====================")
		return self.sendIPtoPi()

	def setServer(self):
		# Create a TCP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		
		print >>sys.stderr, 'starting up on %s port %s' % self.aero_address
		self.sock.bind(self.aero_address)
		self.sock.listen(1)
	
	def sendIPtoPi(self):
		self.hsiip = os.popen('ip addr show wlp1s0').read().split("inet ")[1].split("/")[0]
		print('last_hsiip: ',self.last_hsiip)
		print('self.hsiip: ',self.hsiip)
		ip_changed = True if self.last_hsiip != self.hsiip else False
		if ip_changed:
			print("ip_changed from {} to {}".format(self.last_hsiip, self.hsiip))
			with open("/home/hsi/ip.txt", 'a') as out_file:
                		out_file.write("hsiip {}".format(self.hsiip) + "\n")
			os.system('scp -rp /home/hsi/ip.txt pi@{}:/home/pi/MEng/meng-hsi-group/tcp'.format(self.pi_server_address))

		self.last_hsiip = self.hsiip

		return ip_changed
		

	def waitData(self):
		while True:
			print >>sys.stderr, 'waiting for a connection'
			connection, client_address = self.sock.accept()
			try:
				print >>sys.stderr, 'client connected:', client_address
				while True:
					data = connection.recv(50)
					if data:
						if self.data_handler == None:
							print >>sys.stderr, 'received "%s"' % data
						else:
							self.data_handler(data)
							#connection.sendall(data)
					else:
						break
			except:
				self.sendIPtoPi()
				self.setServer()
					
			finally:
				connection.close()

if __name__ == "__main__":
	tcp = TCP()
	tcp.setServer()
	tcp.checkIP()
	tcp.waitData()
	
