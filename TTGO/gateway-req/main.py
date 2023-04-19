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
import sys
from machine import SoftI2C, Pin, SPI, reset, idle
from lora import LoRa
import ssd1306
from time import sleep

# led = Pin(25,Pin.OUT) # Heltec V2
led = Pin(2,Pin.OUT) # TTGO
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

MY_ID = 0x01 # do not change this
freqs = [868.1, 868.3, 868.5, 867.1, 867.3, 867.5, 867.7, 867.9, 869.525]
# freqs = [433.175, 433.325, 433.475, 433.625, 433.775, 433.925, 434.075, 434.225] # 433.175 - 434.665 according to heltec
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
print(wlan.ifconfig())
oled_lines("TS-LoRa", "Requests GW", wlan.ifconfig()[0], " ")

lora.set_spreading_factor(12)
lora.set_frequency(freqs[-1])
lora.set_preamble_length(10)
lora.set_crc(False)
lora.set_implicit(True)
lora.set_coding_rate(5)
lora.set_payload_length(27) # implicit header is on

# wait for requests
for i in range(7, 13):
    index[i] = 0
for i in range(11, 261): # for 250 devices (1KB - too much memory?)
    JoinNonce[i] = 0x00000001 # 32 bit

def handler(x):
    global lora
    global exp_is_running
    global index
    global JoinNonce
    global stats
    global registered
    print(x, len(x))
    if (len(x) > 20) and (x[1] == 1):
        led.value(1)
        try:
            (dev_id, ptype, mac, JoinEUI, DevNonce, req_sf, rcrc) = struct.unpack("BBQQIBI", x)
            print(dev_id, ptype, hex(mac), hex(JoinEUI), DevNonce, req_sf, rcrc)
        except:
            print("could not unpack!")
            # exp_is_running = 0
            led.value(0)
        else:
            rmsg = b''.join([b'1', mac.to_bytes(8, 'big'), JoinEUI.to_bytes(8, 'big'), DevNonce.to_bytes(4, 'big'), int(req_sf).to_bytes(1, 'big')])
            print(ubinascii.crc32(rmsg), rcrc)
            if (ubinascii.crc32(rmsg) != rcrc):
                print("CRC failed")
            else:
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
                if (dev_id in stats):
    		        stats.remove(dev_id)
                try:
                    # send/receive data to/from Raspberry Pi
                    wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    wlan_s.connect(('192.168.0.254', 8000))
                except:
                    print("Connection to RPi failed!")
                    wlan_s.close()
                    led.value(0)
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
                        led.value(0)
                    else:
                        try:
                            # send registered node info to the data gateway
                            pkt = struct.pack("BBB%ds" % len(AppSkey), dev_id, len(AppSkey), int(slot), AppSkey)
                            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            s.connect(('192.168.0.'+str(req_sf), 8000))
                            s.send(pkt)
                        except:
                            print("Data-gw socket error")
                            wlan_s.close()
                            s.close()
                            led.value(0)
                        else:
                            # send DevAddr and JoinNonce to the node
                            rmsg = b''.join([dev_id.to_bytes(1, 'big'), DevAddr.to_bytes(4, 'big'), JoinNonce[dev_id].to_bytes(4, 'big')])
                            crc = ubinascii.crc32(rmsg)
                            pkg = struct.pack("BBIII", MY_ID, dev_id, DevAddr, JoinNonce[dev_id], crc)
                            # I should encrypt that using node's AppKey
                            lora.send(pkg)
                            lora.standby()
                            print("Responded with a join accept!")
                            lora.set_preamble_length(10)
                            lora.set_crc(False)
                            lora.set_implicit(True)
                            lora.set_coding_rate(5)
                            lora.set_payload_length(27) # implicit header is on
                            wlan_s.close()
                            s.close()
                            led.value(0)
    elif (len(x) > 2) and (x[2] == 2):  # the packet is a statistics packet
        try:
            (dev_id, leng, ptype, i, succeeded, retrans, dropped, rx, tx) = struct.unpack("BBBHHHHff", x)
        except:
            print("wrong stat packet format!")
        else:
            if dev_id not in stats:
                print("Received stats from", dev_id)
                print('Node %d: %d %d %d %d %f %f' % (dev_id, i, succeeded, retrans, dropped, rx, tx))
                stats.append(dev_id)
    lora.recv()

def receive_req():
    global registered
    registered = []
    lora.on_recv(handler)
    lora.recv()
    while (exp_is_running):
        idle()
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
                    pkg = struct.pack('BB%ds' % len(msg), MY_ID, len(msg), msg)
                    lora.send(pkg)
                    oled_lines("TS-LoRa", "Requests GW", wlan.ifconfig()[0], "Sent new exp")
                    lora.standby()
                    stats = []
            except:
                print("wrong packet format!")

#exp_start()  # run a wlan server to initiate a new experiment via the network server (RPi)
# _thread.start_new_thread(receive_req, ())
receive_req()
