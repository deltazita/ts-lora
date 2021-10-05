# TS-LoRa (gateway-requests+stats side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3
#
# Tested with Pycom firmware v1.18.2 (legacy)

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

print("MAC address: " + ubinascii.hexlify(machine.unique_id(),':').decode())


# Colors
off = 0x000000
red = 0x7f0000
green = 0x007f00
blue = 0x00007f

pycom.heartbeat(False)
_LORA_PKG_FORMAT = "!BB%ds"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
MY_ID = 0x01
freqs = [868100000, 868300000, 868500000, 867100000, 867300000, 867500000, 867700000, 867900000]
index = {}
JoinNonce = {}
stats = []
exp_is_running = 0
registered = []

wlan = WLAN(mode=WLAN.STA)
if not wlan.isconnected():
    wlan.connect('rasp', auth=(WLAN.WPA2, 'lalalala'), timeout=5000)
    while not wlan.isconnected():
        machine.idle()
print("I got IP "+wlan.ifconfig()[0])

pycom.rgbled(green)
print("Starting with lora...")
lora = LoRa(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)
lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
lora_sock.setblocking(False)

def receive_req():
    global index
    global JoinNonce
    global stats
    global registered
    registered = []
    while (exp_is_running):
        recv_pkg = lora_sock.recv(100)
        if (len(recv_pkg) > 2):
            recv_pkg_len = recv_pkg[1]
            recv_pkg_id = recv_pkg[0]
            if (recv_pkg_len < 100) and (int(recv_pkg_id) <= 35):
                pycom.rgbled(blue)
                dev_id, leng, msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                msg = msg.decode('utf-8')
                count = msg.count(":")
                if (count == 3): # the packet is a join request
                    print("Received a join request from", dev_id)
                    try: # the packet may arrive corrupted
                        (mac, JoinEUI, DevNonce, req_sf) = msg.split(":")
                        int(mac, 16)
                        int(JoinEUI, 16)
                        DevNonce = int(DevNonce)
                        req_sf = int(req_sf)
                    except:
                        print("wrong node packet format!")
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
                        try:
                            # send/receive data to/from Raspberry Pi
                            wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            wlan_s.connect(('192.168.0.254', 8000))
                        except:
                            print("Connection to RPi failed!")
                            wlan_s.close()
                        else:
                            wlan_s.send(str(dev_id)+":"+str(slot)+":"+mac+":"+str(JoinNonce[int(dev_id)])+":"+JoinEUI+":"+str(DevNonce))
                            msg = wlan_s.recv(512)
                            msg = msg.decode('utf-8')
                            try:
                                (rdev_id, DevAddr, AppSkey) = msg.split(":")
                            except:
                                print("wrong RPi packet format!")
                                pycom.rgbled(red)
                                wlan_s.close()
                            else:
                                try:
                                    # send registered node info to the data gateway
                                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                                    s.connect(('192.168.0.'+str(req_sf-5), 8000))
                                    s.send(str(dev_id)+":"+str(slot)+":"+AppSkey)
                                except:
                                    print("Data-gw socket error")
                                    pycom.rgbled(red)
                                    wlan_s.close()
                                    s.close()
                                else:
                                    # send DevAddr and JoinNonce to the node
                                    msg = str(dev_id)+":"+str(DevAddr)+":"+str(JoinNonce[int(dev_id)])
                                    print(msg, str(slot))
                                    lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[1], power_mode=LoRa.TX_ONLY, bandwidth=LoRa.BW_125KHZ, sf=12, tx_power=7)
                                    pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
                                    lora_sock.send(pkg) # I should encrypt that using node's AppKey
                                    lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)
                                    wlan_s.close()
                                    s.close()
                                    pycom.rgbled(green)
                elif (count == 5): # the packet is a statistics packet
                    try:
                        (i, succeeded, retrans, dropped, rx, tx) = msg.split(":")
                        i = int(i)
                        succeeded = int(succeeded)
                        retrans = int(retrans)
                        dropped = int(dropped)
                        rx = float(rx)
                        tx = float(tx)
                    except:
                        print("wrong packet format!")
                    else:
                        if dev_id not in stats:
                            print("Received stats from", dev_id)
                            print('Node %d: %d %d %d %d %f %f' % (int(dev_id), i, succeeded, retrans, dropped, rx, tx))
                            stats.append(dev_id)
    print("Stopped accepting join requests!")

def exp_start():
    global stats
    global exp_is_running
    host = wlan.ifconfig()[0]
    port = 8000
    wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    wlan_s.bind((host, port))
    wlan_s.listen(5)
    while (True):
        conn, addr = wlan_s.accept()
        data = conn.recv(512)
        data = data.decode('utf-8')
        if (len(data) > 10):
            try:
                print(data)
                (init, ip, pkts) = data.split(":")
                if (init == "init"):
                    exp_is_running = 0
                    print("---------------------------------")
                    print("New experiment with", pkts, "packets")
                    pycom.rgbled(blue)
                    msg = ip+":"+pkts
                    time.sleep(1)
                    exp_is_running = 1
                    _thread.start_new_thread(receive_req, ())
                    print("Ready to accept new join requests!")
                    time.sleep(1)
                    lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.TX_ONLY, bandwidth=LoRa.BW_125KHZ, sf=12)
                    pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
                    # for j in range(3): # send it 3 times
                    lora_sock.send(pkg)
                    # time.sleep(1)
                    lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)
                    pycom.rgbled(green)
                    stats = []
            except:
                print("wrong packet format!")

# wait for requests
for i in range(7,13):
    index[i] = 0
for i in range(11,36): # for 25 devices
    JoinNonce[i] = 0

exp_start() # run a wlan server to initiate a new experiment via the network server (RPi)
