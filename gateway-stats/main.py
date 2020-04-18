# TS-LoRa (gateway-stats side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3

import socket
import struct
from network import LoRa
from network import WLAN
import pycom
import _thread
import ubinascii
from machine import SD
import os
import time

# Colors
off = 0x000000
red = 0x7f0000
green = 0x007f00
blue = 0x00007f

MY_ID = 0x00
pycom.heartbeat(False)
_LORA_PKG_FORMAT = "!BB%ds"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
freqs = [868100000, 868300000, 868500000, 867100000, 867300000, 867500000, 867700000, 867900000]

pycom.rgbled(green)
print("Starting with lora...")
lora = LoRa(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[7], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)
lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
lora_sock.setblocking(False)
stats = []
detailed_stats = []
run_stats = 1
sd = SD() # remove those lines if you don't use an SD card
os.mount(sd, '/sd')

def receive_stats():
    global stats
    global detailed_stats
    global sd
    global run_stats
    print("Ready to accept stats!")
    while (run_stats == 1):
        recv_pkg = lora_sock.recv(250)
        if (len(recv_pkg) > 2):
            recv_pkg_len = recv_pkg[1]
            recv_pkg_id = recv_pkg[0]
            if (recv_pkg_len > 10):
                try: # the packet may arrive corrupted
                    dev_id, leng, msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                    # print('Dev %d sent: %s' % (dev_id, msg))
                    msg = str(msg)[2:][:-1]
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
                        print('Node %d: %d %d %d %d %f %f' % (int(dev_id), i, succeeded, retrans, dropped, rx, tx))
                        try:
                            f = open('/sd/results.txt', 'a')
                            f.write('Node %d: %d %d %d %d %f %f\n' % (int(dev_id), i, succeeded, retrans, dropped, rx, tx))
                            f.close()
                        except:
                            print("error with the file!")
                        stats.append(dev_id)
                        detailed_stats.append([int(dev_id), i, succeeded, retrans, dropped, rx, tx])
    print("Stopped accepting stats!")

def exp_start():
    global stats
    global detailed_stats
    global run_stats
    wlan = WLAN(mode=WLAN.STA)
    print(ubinascii.hexlify(wlan.mac(),':').decode())
    if not wlan.isconnected():
        wlan.connect(ssid='rasp', auth=(WLAN.WPA2, 'lalalala'), timeout=5000)
    while not wlan.isconnected():
        machine.idle()
    print("I got IP"+wlan.ifconfig()[0])
    host = wlan.ifconfig()[0]
    port = 8000
    wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("socket created")
    wlan_s.bind((host, port))
    wlan_s.listen(5)
    print("Waiting for initialization...")
    _thread.start_new_thread(receive_stats, ())
    while (True):
        conn, addr = wlan_s.accept()
        print('Got connection from', addr)
        data = conn.recv(512)
        data = str(data)[2:][:-1]
        if (len(data) > 10):
            try:
                (init, ip, pkts) = data.split(":")
                if (init == "init"):
                    pycom.rgbled(red)
                    msg = ip+":"+pkts
                    run_stats = 0
                    time.sleep(1)
                    lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.TX_ONLY, bandwidth=LoRa.BW_125KHZ, sf=12)
                    pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
                    # lora_sock.send(pkg)
                    for j in range(3):
                        lora_sock.send(pkg)
                        time.sleep(1)
                    lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[7], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)
                    pycom.rgbled(green)
                    run_stats = 1
                    stats = []
                    detailed_stats = []
                    _thread.start_new_thread(receive_stats, ())
            except:
                print("wrong packet format!")

exp_start()
