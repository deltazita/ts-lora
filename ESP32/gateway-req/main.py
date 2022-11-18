# TS-LoRa (gateway-requests+stats side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3
#
# Tested with Heltec LoRa v2 433MHz SX1278 and micropython v1.19.1 for ESP32 LILYGO

import socket
import struct
import network
import binascii
import ubinascii
import time
import uos
import _thread
import uerrno
from machine import Timer
import sys
from machine import SoftI2C, Pin, SPI
from lora import LoRa
import ssd1306
from time import sleep

led = Pin(25,Pin.OUT)
rst = Pin(16, Pin.OUT)
rst.value(1)
scl = Pin(15, Pin.OUT, Pin.PULL_UP)
sda = Pin(4, Pin.OUT, Pin.PULL_UP)
i2c = SoftI2C(scl=scl, sda=sda, freq=450000)
oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3c)
oled.poweron()

def oled_lines(line1, line2, line3, line4):
    oled.fill(0)
    oled.text(line1, 0, 0)
    oled.text(line2, 0, 10)
    oled.text(line3, 0, 20)
    oled.text(line4, 0, 30)
    oled.show()

oled_lines("TS-LoRa", "Requests GW", " ", " ")

# SPI pins
SCK  = 5
MOSI = 27
MISO = 19
CS   = 18
RX   = 26

spi = SPI(
    1,
    baudrate=1000000,
    sck=Pin(SCK, Pin.OUT, Pin.PULL_DOWN),
    mosi=Pin(MOSI, Pin.OUT, Pin.PULL_UP),
    miso=Pin(MISO, Pin.IN, Pin.PULL_UP),
)
spi.init()

lora = LoRa( spi, cs=Pin(CS, Pin.OUT), rx=Pin(RX, Pin.IN), )

_LORA_PKG_FORMAT = "!BB%ds"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
MY_ID = 1
# freqs = [868.1, 868.3, 868.5, 867.1, 867.3, 867.5, 867.7, 867.9]
# freqs = [903.9, 904.1, 904.3, 904.5, 904.7, 904.9, 905.1, 905.3]
freqs = [433.175, 433.325, 433.475, 433.625, 433.775, 433.925, 434.075, 434.225] # 433.175 - 434.665 according to heltec
index = {}
JoinNonce = {}
stats = []
exp_is_running = 1
registered = []

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
mac = ubinascii.hexlify(wlan.config('mac')).decode().upper()
mac = ':'.join(mac[i:i+2] for i in range(0,12,2))
print(mac)
oled_lines("TS-LoRa", "Requests GW", mac, " ")
if not wlan.isconnected():
    print('connecting to network...')
    wlan.connect('rasp', 'lalalala')
    while not wlan.isconnected():
        pass
# print('network config:', wlan.ifconfig())
oled_lines("TS-LoRa", "Requests GW", wlan.ifconfig()[0], " ")

lora.set_spreading_factor(12)
lora.set_frequency(freqs[0])

# wait for requests
for i in range(7, 13):
    index[i] = 0
for i in range(11, 36):  # for 25 devices
    JoinNonce[i] = 0x00000001 # 32 bit

def handler(x):
    global exp_is_running
    global index
    global JoinNonce
    global stats
    global registered
    # print(x[2])
    if (len(x) > 2) and (x[2] == 1):
        try:
            (dev_id, leng, ptype, mac, JoinEUI, DevNonce, req_sf) = struct.unpack("BBBQQIB", x)
            print(str(dev_id), str(leng), str(ptype), hex(mac), hex(JoinEUI), str(DevNonce), str(req_sf))
            if (req_sf > 12):
                raise
        except:
            print("could not unpack!")
            exp_is_running = 0
        else:
            if (ptype == 1):  # the packet is a join request
                if (len(hex(mac)+hex(JoinEUI)+str(DevNonce)+str(req_sf)) != leng):
                    print("wrong packet!")
                print("Received a join request from", dev_id)
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
                    wlan_s = socket.socket(
                        socket.AF_INET, socket.SOCK_STREAM)
                    wlan_s.connect(('192.168.0.254', 8000))
                except:
                    print("Connection to RPi failed!")
                    wlan_s.close()
                else:
                    print(str(dev_id)+":"+str(slot)+":"+hex(mac)[2:]+":"+str(JoinNonce[dev_id])+":"+hex(JoinEUI)[2:]+":"+str(DevNonce))
                    wlan_s.send(str(dev_id)+":"+str(slot)+":"+hex(mac)[2:]+":"+
                                str(JoinNonce[dev_id])+":"+hex(JoinEUI)[2:]+":"+str(DevNonce))
                    msg = wlan_s.recv(512)
                    try:
                        (rdev_id, leng, DevAddr, AppSkey) = struct.unpack("BBI%ds" % msg[1], msg)
                        print("Received from RPi:", rdev_id, hex(DevAddr), AppSkey)
                    except:
                        print("wrong RPi packet format!")
                        wlan_s.close()
                    else:
                        try:
                            # send registered node info to the data gateway
                            pkt = struct.pack("BBB%ds" % len(AppSkey), dev_id, len(AppSkey), int(slot), AppSkey)
                            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            s.connect(('192.168.0.'+str(req_sf-5), 8000))
                            s.send(pkt)
                        except:
                            print("Data-gw socket error")
                            wlan_s.close()
                            s.close()
                        else:
                            # send DevAddr and JoinNonce to the node
                            msg = str(dev_id)+hex(DevAddr)[2:]+str(JoinNonce[dev_id])
                            lora.set_frequency(freqs[1])
                            pkg = struct.pack("BBBII", MY_ID, len(msg), dev_id, DevAddr, JoinNonce[dev_id])
                            # I should encrypt that using node's AppKey
                            lora.send(pkg)
                            print("Responded with a join accept!")
                            lora.set_frequency(freqs[0])
                            wlan_s.close()
                            s.close()
    elif (len(x) > 2) and (x[2] == 2):  # the packet is a statistics packet
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
                print('Node %d: %d %d %d %d %f %f' % (
                    int(dev_id), i, succeeded, retrans, dropped, rx, tx))
                stats.append(dev_id)

def receive_req():
    global registered
    registered = []
    while (exp_is_running):
        lora.recv()
        lora.on_recv(handler)
    print("Stopped accepting join requests!")
    led.value(0)


def exp_start():
    global stats
    global exp_is_running
    host = wlan.ifconfig()[0]
    port = 8000
    wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    wlan_s.bind((host, port))
    wlan_s.listen(5)
    print("Ready...")
    led.value(1)
    while (True):
        conn, addr = wlan_s.accept()
        data = conn.recv(512)
        data = data.decode('utf-8')
        if (len(data) > 10):
            try:
                (init, ip, pkts) = data.split(":")
                if (init == "init"):
                    exp_is_running = 0
                    print("---------------------------------")
                    print("New experiment with", pkts, "packets")
                    led.value(1)
                    # msg = ip+":"+pkts
                    msg = pkts
                    time.sleep(1)
                    exp_is_running = 1
                    _thread.start_new_thread(receive_req, ())
                    print("Ready to accept new join requests!")
                    time.sleep(1)
                    pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
                    lora.send(pkg)
                    oled_lines("TS-LoRa", "Requests GW", wlan.ifconfig()[0], "Sent new exp")
                    lora.standby()
                    stats = []
            except:
                print("wrong packet format!")

#exp_start()  # run a wlan server to initiate a new experiment via the network server (RPi)
_thread.start_new_thread(receive_req, ())
