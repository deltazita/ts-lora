#!/usr/bin/python3

import socket
import sys

pkts = sys.argv[1] # packets to send per node
TCP_IP = '192.168.0.1' # gateway-req IP
TCP_PORT = 8000
BUFFER_SIZE = 512
MESSAGE = "init:"+str(pkts)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
s.send( bytes( MESSAGE.encode('utf-8') ) )
s.close()
