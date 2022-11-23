# TS-LoRa (gateway-data side)
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
import math
from chrono import Chrono
from cryptolib import aes
import gc

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

oled_lines("TS-LoRa", "Data GW SF7", " ", " ")

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
MY_ID = 0x02
# freqs = [868.1, 868.3, 868.5, 867.1, 867.3, 867.5, 867.7, 867.9]
# freqs = [903.9, 904.1, 904.3, 904.5, 904.7, 904.9, 905.1, 905.3]
freqs = [433.175, 433.325, 433.475, 433.625, 433.775, 433.925, 434.075, 434.225] # 433.175 - 434.665 according to heltec
my_sf = int(MY_ID) + 5
(guard, my_bw_index, packet_size) = (15, 0, 16) # the packet size must align with the nodes packet size
index = 0
slot = {}
KEY = {}

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
mac = ubinascii.hexlify(wlan.config('mac')).decode().upper()
mac = ':'.join(mac[i:i+2] for i in range(0,12,2))
print(mac)
oled_lines("TS-LoRa", "Data GW SF7", mac, " ")
if not wlan.isconnected():
    print('connecting to network...')
    wlan.connect('rasp', 'lalalala')
    while not wlan.isconnected():
        pass
# print('network config:', wlan.ifconfig())
oled_lines("TS-LoRa", "Data GW SF7", wlan.ifconfig()[0], " ")

lora.set_spreading_factor(my_sf)
lora.set_frequency(freqs[my_sf-5])

# this is borrowed from LoRaSim (https://www.lancaster.ac.uk/scc/sites/lora/lorasim.html)
def airtime_calc(sf,cr,pl,bw):
    H = 0        # implicit header disabled (H=0) or not (H=1)
    DE = 0       # low data rate optimization enabled (=1) or not (=0)
    Npream = 8
    if bw == 125 and sf in [11, 12]:
        # low data rate optimization mandated for BW125 with SF11 and SF12
        DE = 1
    if sf == 6:
        # can only have implicit header with SF6
        H = 1
    Tsym = (2.0**sf)/bw
    Tpream = (Npream + 4.25)*Tsym
    payloadSymbNB = 8 + max(math.ceil((8.0*pl-4.0*sf+28+16-20*H)/(4.0*(sf-2*DE)))*(cr+4),0)
    Tpayload = payloadSymbNB * Tsym
    return Tpream + Tpayload

def update_index():
    global index
    global slot
    global KEY
    host = '192.168.1.'+str(my_sf)
    port = 8000
    wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("WiFi socket created.")
    wlan_s.bind((host, port))
    wlan_s.listen(150)
    while (True):
        conn, addr = wlan_s.accept()
        print('Got connection from', addr)
        data = conn.recv(256)
        if len(data) > 2:
            (id, leng, nslot, AppSKey) = struct.unpack("BBB%ds" % data[1], data)
            id = int(id)
            nslot = int(nslot)
            if (id not in KEY):
                slot[nslot] = id
                print('slot %d to %d' % (nslot, id))
            else:
                print('node %d rejoined (slot %d)' % (id, nslot))
            KEY[id] = AppSKey
            if (nslot > index):
                index = nslot
    wlan_s.close()

def handler(recv_pkg):
    global packet_size
    global received
    global acks
    print(recv_pkg)
    if (len(recv_pkg) > 2):
        recv_pkg_len = recv_pkg[1]
        recv_pkg_id = recv_pkg[0]
        if (recv_pkg_len == packet_size):
            dev_id, leng, msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
            if (len(msg) == packet_size): # format check
                received += 1
                print("Received", msg, "from:", dev_id)
                msg = aes(KEY[dev_id], 1).encrypt(msg)
                print("Decrypted text:", msg)
                acks.append(str(dev_id))

def receive_data():
    global index
    global guard
    global slot
    global packet_size
    global received
    global acks
    guard = 1000*guard
    (overall_received, overall_sent) = (0, 0)
    airt = int(airtime_calc(my_sf,1,packet_size+2,125)*1000)
    duty_cycle_limit_slots = math.ceil(100*airt/(airt + 2*guard))
    print("duty cycle slots:", duty_cycle_limit_slots)
    print("packet airtime (ms):", airt/1000)
    print("guard time (ms):", guard/1000)
    lora.standby()
    chrono = Chrono()
    chrono.start()
    i = 0x00000001
    while(True):
        print(i, "----------------------------------------------------")
        print("Net size is:", index+1)
        chrono.reset()
        round_start = chrono.read_us()
        received = 0
        acks = []
        if (int(index) > duty_cycle_limit_slots):
            round_length = math.ceil(int(index)*(airt + 2*guard))
        else:
            round_length = math.ceil(duty_cycle_limit_slots*(airt + 2*guard))
        rec_start = chrono.read_us()
        led.value(0)
        while ((chrono.read_us() - round_start) < round_length-(airt + 2*guard)): # kill the last slot
            lora.recv()
            lora.on_recv(handler)
        lora.sleep()
        print(received, "packet(s) received")
        rec_lasted = chrono.read_us()-rec_start
        if (rec_lasted < round_length):
            print("I'll sleep a bit to align with the round length")
            time.sleep_us(int(round_length-rec_lasted))
        print("Receiving lasted (ms):", (chrono.read_us()-rec_start)/1000)
        print("...should last (ms):", round_length/1000)
        proc_t = chrono.read_us()
        ack_msg = ""
        for n in range(int(index)+1):
            if n in slot:
                id = str(slot[n])
                if id in acks:
                    ack_msg = ack_msg+"1"
                else:
                    ack_msg = ack_msg+"0"
        if (ack_msg == ""):
            ack_msg += "0"
        ack_msg = int(ack_msg, 2)
        proc_t = chrono.read_us()-proc_t
        print("proc time (ms):", proc_t/1000)
        sync_start = chrono.read_us()
        time.sleep_us(guard)
        data = str(index+1)+str(int(proc_t/1000))+hex(ack_msg)
        pkg = struct.pack("BBBfI", MY_ID, len(data), index+1, proc_t, ack_msg)
        lora.standby()
        led.value(1)
        lora.send(pkg)
        print("Sent sync: "+hex(ack_msg))
        led.value(0)
        lora.standby()
        time.sleep_ms(100) # node time after sack
        print("sync lasted (ms):", (chrono.read_us()-sync_start)/1000)
        print("round lasted (ms):", (chrono.read_us()-round_start)/1000)
        i += 1

_thread.start_new_thread(update_index, ())
_thread.start_new_thread(receive_data, ())
