# TS-LoRa (gateway-stats side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3

import socket
import struct
from network import LoRa
import pycom
import _thread

# Colors
off = 0x000000
red = 0x7f0000
green = 0x007f00
blue = 0x00007f

pycom.heartbeat(False)
_LORA_PKG_FORMAT = "!BB%ds"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
freqs = [869700000, 869850000, 865000000, 865600000, 866200000, 866800000, 867400000, 868000000] # 2x125, 6x500 channels, 1% rdc

pycom.rgbled(green)
print("Starting with lora...")
lora = LoRa(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[7], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=7)
lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
lora_sock.setblocking(False)
detailed_stats = []

def receive_stats():
    stats = []
    global detailed_stats
    while (True):
        recv_pkg = lora_sock.recv(100)
        if (len(recv_pkg) > 2):
            recv_pkg_len = recv_pkg[1]
            recv_pkg_id = recv_pkg[0]
            if (recv_pkg_len < 100) and (int(recv_pkg_id) <= 35):
                pycom.rgbled(blue)
                dev_id, leng, msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                # print('Dev %d requested: %s' % (dev_id, msg))
                msg = str(msg)[2:]
                msg = msg[:-1]
                try: # the packet may arrive corrupted
                    (i, succeeded, retrans, dropped) = msg.split(":")
                    i = int(i)
                    succeeded = int(succeeded)
                    retrans = int(retrans)
                    dropped = int(dropped)
                except:
                    print("wrong node packet format!", msg)
                else:
                    if dev_id not in stats:
                        print('Node %d: %d %d %d %d' % (int(dev_id), int(i), int(succeeded), int(retrans), int(dropped)))
                        stats.append(dev_id)
                        detailed_stats.append([int(dev_id), int(i), int(succeeded), int(retrans), int(dropped)])

# wait for requests
_thread.start_new_thread(receive_stats, ())
print("Ready to accept stats!")
