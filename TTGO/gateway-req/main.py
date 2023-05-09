# TS-LoRa (gateway-requests+stats side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3
#
# Tested with Heltec LoRa v2 433MHz SX1278 and micropython v1.20.0 for ESP32 LILYGO

import socket
import struct
import network
import binascii
import ubinascii
import time
import _thread
import uerrno
from machine import SoftI2C, Pin, SPI
from lora import LoRa
import ssd1306

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
j_sf = 10
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
print("MAC address:", mac)
oled_lines("TS-LoRa", "Requests GW", mac, " ")
if not wlan.isconnected():
    print('connecting to the network...')
    wlan.connect('rasp', 'lalalala')
    while not wlan.isconnected():
        pass
print(wlan.ifconfig())
oled_lines("TS-LoRa", "Requests GW", wlan.ifconfig()[0], " ")

lora.set_spreading_factor(j_sf)
lora.set_frequency(freqs[-1])
# lora.set_implicit(True)
# lora.set_preamble_length(10)
# lora.set_crc(False)
# lora.set_coding_rate(5)
# lora.set_payload_length(27) # implicit header is on

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
    #print(x, len(x))
    if (len(x) > 20) and (x[1] == 1):
        led.value(1)
        try:
            (dev_id, ptype, mac, JoinEUI, DevNonce, req_sf, rcrc) = struct.unpack("BBQQIBI", x)
            print("Received a join-request from:", dev_id, ptype, hex(mac), hex(JoinEUI), DevNonce, req_sf, rcrc)
        except Exception as e:
            print("could not unpack!", e)
            led.value(0)
        else:
            rmsg = b''.join([b'1', mac.to_bytes(8, 'big'), JoinEUI.to_bytes(8, 'big'), DevNonce.to_bytes(4, 'big'), int(req_sf).to_bytes(1, 'big')])
            if (ubinascii.crc32(rmsg) != rcrc):
                print("CRC failed")
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
                if (dev_id in stats):
    		        stats.remove(dev_id)
                try:
                    # send/receive data to/from Raspberry Pi
                    wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    wlan_s.connect(('192.168.0.254', 8000))
                except Exception as e:
                    print("Connection to RPi failed!", e)
                    wlan_s.close()
                    led.value(0)
                else:
                    rmsg = b''.join([int(dev_id).to_bytes(1, 'big'), slot.to_bytes(1, 'big'), mac.to_bytes(8, 'big'), JoinNonce[dev_id].to_bytes(4, 'big'), JoinEUI.to_bytes(8, 'big'), int(DevNonce).to_bytes(4, 'big')])
                    crc = ubinascii.crc32(rmsg)
                    pkg = struct.pack('BBQIQII', int(dev_id), slot, int(mac), JoinNonce[dev_id], int(JoinEUI), DevNonce, crc)
                    wlan_s.send(pkg)
                    msg = wlan_s.recv(512)
                    try:
                        (rdev_id, leng, DevAddr, AppSkey) = struct.unpack("BBI%ds" % msg[1], msg)
                        print("Received from RPi:", rdev_id, hex(DevAddr), ubinascii.hexlify(AppSkey).decode())
                    except Exception as e:
                        print("wrong RPi packet format!", e)
                        wlan_s.close()
                        led.value(0)
                    else:
                        try:
                            # send registered node info to the data gateway
                            pkt = struct.pack("BBB%ds" % len(AppSkey), dev_id, len(AppSkey), int(slot), AppSkey)
                            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            s.connect(('192.168.0.'+str(req_sf), 8000))
                            s.send(pkt)
                        except Exception as e:
                            print("Data-gw socket error", e)
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
                            print("Responded with a join accept!")
                            wlan_s.close()
                            s.close()
                            led.value(0)
    elif (len(x) > 20) and (x[2] == 2):  # the packet is a statistics packet
        try:
            (dev_id, leng, ptype, i, succeeded, retrans, dropped, rx, tx) = struct.unpack("BBBiiiiff", x) # this must also be 27b long
        except Exception as e:
            print("wrong stat packet format!", e)
        else:
            if dev_id not in stats:
                print("Received stats from", dev_id)
                print('Node %d: %d %d %d %d %f %f' % (dev_id, i, succeeded, retrans, dropped, rx, tx))
                stats.append(dev_id)
    # lora.standby()
    # lora.set_implicit(True)
    # lora.set_preamble_length(10)
    # lora.set_crc(False)
    # lora.set_coding_rate(5)
    # lora.set_payload_length(27)
    lora.on_recv(handler)
    lora.recv()

def receive_req():
    global lora
    global registered
    registered = []
    # lora.set_implicit(True)
    # lora.set_preamble_length(10)
    # lora.set_payload_length(27)
    # lora.set_crc(False)
    # lora.set_coding_rate(5)
    lora.on_recv(handler)
    lora.recv()
    while (exp_is_running):
        time.sleep(0.1)
        pass
    print("Stopped accepting join requests!")
    led.value(0)


def exp_start():
    global stats
    global exp_is_running
    global lora
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
        if (len(data) > 2):
            try:
                (init, pkts) = data.split(":")
                if (init == "init"):
                    exp_is_running = 0
                    print("---------------------------------")
                    print("New experiment with", pkts, "packets")
                    led.value(1)
                    time.sleep(1)
                    exp_is_running = 1
                    lora.set_implicit(False)
                    lora.set_preamble_length(8)
                    # lora.set_crc(True)
                    pkg = struct.pack('Bi', MY_ID, int(pkts))
                    lora.send(pkg)
                    oled_lines("TS-LoRa", "Requests GW", wlan.ifconfig()[0], "Sent new exp")
                    time.sleep(1)
                    _thread.start_new_thread(receive_req, ())
                    print("Ready to accept new join requests!")
                    stats = []
            except Exception as e:
                print("wrong packet format!", e)

# uncomment the following line to initiate new experiments via the network server (RPi's init_exp.py script)
# exp_start()
# OR bypass the init script by uncommenting the following line
receive_req()
