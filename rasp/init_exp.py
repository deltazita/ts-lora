#!/usr/bin/env python

import socket
import sys

pkts = sys.argv[1] # packets to send per node
OTA_IP = "10.91.8.242" # OTA server IP
TCP_IP = '192.168.0.10' # gatewa-req IP
TCP_PORT = 8000
BUFFER_SIZE = 512
MESSAGE = "init:"+OTA_IP+":"+str(pkts)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
s.send(MESSAGE)
s.close()
