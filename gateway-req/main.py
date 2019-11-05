# TS-LoRa (gateway-requests side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3

import socket
import struct
from network import LoRa
from network import WLAN
import binascii
import ubinascii
import pycom
import time
import uos
import crypto
import _thread
import uerrno
from machine import Timer
import sys

# Colors
off = 0x000000
red = 0x7f0000
green = 0x007f00
blue = 0x00007f

pycom.heartbeat(False)
_LORA_PKG_FORMAT = "!BB%ds"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
MY_ID = 0x01
freqs = [869700000, 869850000, 865000000, 865600000, 866200000, 866800000, 867400000, 868000000] # 2x125, 6x500 channels, 1% rdc
index = {}
JoinNonce = {}

wlan = WLAN(mode=WLAN.STA)
if not wlan.isconnected():
    wlan.connect('rasp', auth=(WLAN.WPA2, 'lalalala'), timeout=5000)
    while not wlan.isconnected():
        machine.idle()
print("I got IP"+wlan.ifconfig()[0])

pycom.rgbled(green)
print("Starting with lora...")
lora = LoRa(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)
lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
lora_sock.setblocking(False)

def receive_req():
    global index
    global JoinNonce
    registered = []
    while (True):
        recv_pkg = lora_sock.recv(100)
        if (len(recv_pkg) > 2):
            recv_pkg_len = recv_pkg[1]
            recv_pkg_id = recv_pkg[0]
            if (recv_pkg_len < 100) and (int(recv_pkg_id) <= 35):
                pycom.rgbled(blue)
                dev_id, leng, msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                print('Dev %d requested: %s' % (dev_id, msg))
                msg = str(msg)[2:]
                msg = msg[:-1]
                try: # the packet may arrive corrupted
                    (mac, JoinEUI, DevNonce, req_sf) = msg.split(":")
                    int(mac, 16)
                    int(JoinEUI, 16)
                    DevNonce = int(DevNonce)
                    req_sf = int(req_sf)
                except:
                    print("wrong node packet format!", msg)
                else:
                    exists = 0
                    slot = 0
                    for n in registered:
                        (id, nslot) = n
                        if (int(dev_id) == id):
                            exists = 1
                            slot = nslot
                    if (exists == 0):
                        slot = index[req_sf]
                        index[req_sf] += 1
                        registered.append([int(dev_id), slot])
                    JoinNonce[int(dev_id)] += 1
                    # send/receive data to/from Raspberry Pi
                    wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    wlan_s.connect(('192.168.0.254', 8000))
                    wlan_s.send(str(dev_id)+":"+str(slot)+":"+mac+":"+str(JoinNonce[int(dev_id)])+":"+JoinEUI+":"+str(DevNonce))
                    msg = wlan_s.recv(512)
                    msg = str(msg)[2:]
                    msg = msg[:-1]
                    try:
                        (rdev_id, DevAddr, AppSkey) = msg.split(":")
                    except:
                        print("wrong RPi packet format!", msg)
                        wlan_s.close()
                    else:
                        # send registered node info to the data gateway
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.connect(('192.168.0.'+str(req_sf-5), 8000))
                        try:
                            s.send(str(dev_id)+":"+str(slot)+":"+AppSkey)
                        except OSError as e:
                            pycom.rgbled(red)
                            if e.args[0] != uerrno.EINPROGRESS:
                                raise
                        else:
                            # send DevAddr and JoinNonce to the node
                            msg = str(dev_id)+":"+str(DevAddr)+":"+str(JoinNonce[int(dev_id)])
                            print(msg, str(slot))
                            lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[1], power_mode=LoRa.TX_ONLY, bandwidth=LoRa.BW_125KHZ, sf=12, tx_power=7)
                            pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
                            lora_sock.send(pkg) # I should encrypt that using node's AppKey
                            lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)

# wait for requests
for i in range(7,13):
    index[i] = 0
for i in range(11,36):
    JoinNonce[i] = 0
_thread.start_new_thread(receive_req, ())
print("Ready to accept requests!")
