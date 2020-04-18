# TS-LoRa (gateway-data side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3
#
# Tested with firmware v1.18.2

import socket
import struct
from network import LoRa
from network import WLAN
import ubinascii
import pycom
import time
import _thread
from machine import Timer
import math

# Colors
off = 0x000000
red = 0x7f0000
green = 0x007f00
blue = 0x00007f
white = 0xFFFAFA
pycom.heartbeat(False)

_LORA_PKG_FORMAT = "!BB%ds"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
MY_ID = 0x02
my_sf = int(MY_ID) + 5
(guard, my_bw_index, sync_rate, packet_size) = (15, 0, 1, 16) # the packet size is without the overhead (2 bytes)
freqs = [868100000, 868300000, 868500000, 867100000, 867300000, 867500000, 867700000, 867900000]
index = 0
slot = {}
KEY = {}
if (my_bw_index == 0):
    my_bw = LoRa.BW_125KHZ
    my_bw_plain = 125
elif (my_bw_index == 1):
    my_bw = LoRa.BW_250KHZ
    my_bw_plain = 250
elif (my_bw_index == 2):
    my_bw = LoRa.BW_500KHZ
    my_bw_plain = 500

wlan = WLAN(mode=WLAN.STA)
print("My MAC address is:", ubinascii.hexlify(wlan.mac(),':').decode())
if not wlan.isconnected():
    wlan.connect('rasp', auth=(WLAN.WPA2, 'lalalala'), timeout=5000)
    while not wlan.isconnected():
        machine.idle()
print("I got IP"+wlan.ifconfig()[0])

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
    host = '192.168.0.'+str(int(MY_ID))
    port = 8000
    wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("socket created")
    wlan_s.bind((host, port))
    wlan_s.listen(5)
    while (True):
        conn, addr = wlan_s.accept()
        print('Got connection from', addr)
        data = conn.recv(512)
        data = str(data)[2:]
        data = data[:-1]
        if (len(data) > 2):
            (id, nslot, AppSKey) = data.split(":")
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

def receive_data():
    global index
    global guard
    global slot
    global packet_size
    lora = LoRa(mode=LoRa.LORA, rx_iq=True, frequency=freqs[my_sf-5], region=LoRa.EU868, power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
    lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    lora_sock.setblocking(False)
    guard = 1000*guard
    (overall_received, overall_sent) = (0, 0)
    airt = int(airtime_calc(my_sf,1,packet_size+2,my_bw_plain)*1000)
    duty_cycle_limit_slots = math.ceil(100*airt/(airt + 2*guard))
    print("duty cycle slots:", duty_cycle_limit_slots)
    print("packet airtime (ms):", airt/1000)
    print("guard time (ms):", guard/1000)
    chrono = Timer.Chrono()
    chrono.start()
    i = 1
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
        lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-5], power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
        rec_start = chrono.read_us()
        pycom.rgbled(green)
        while ((chrono.read_us() - round_start) < round_length-66000): # the following line may take up to 66ms
            recv_pkg = lora_sock.recv(256)
            if (len(recv_pkg) > 2):
                recv_pkg_len = recv_pkg[1]
                recv_pkg_id = recv_pkg[0]
                if (int(recv_pkg_id) <= 35) and (int(recv_pkg_len) == int(packet_size)):
                    dev_id, leng, msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                    if (len(msg) == packet_size): # format check
                        received += 1
                        # print('Received from: %d' % dev_id)
                        # print(lora.stats())
                        acks.append(str(int(dev_id)))
                        pycom.rgbled(off)
        print(received, "packets received")
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
        if (ack_msg != ""):
            ack_msg = str(hex(int(ack_msg, 2)))[2:]
        print("proc time (ms):", (chrono.read_us()-proc_t)/1000)
        proc_t = chrono.read_us()-proc_t
        if (i % sync_rate == 0): # SACK
            sync_start = chrono.read_us()
            pycom.rgbled(white)
            time.sleep_us(int(guard*3/2)) # let's make it long so all the nodes are up
            lora.init(mode=LoRa.LORA, tx_iq=True, frequency=freqs[my_sf-5], region=LoRa.EU868, power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf, tx_power=14)
            data = str(index+1)+":"+str(int(proc_t/1000))+":"+ack_msg
            pkg = struct.pack(_LORA_PKG_FORMAT % len(data), MY_ID, len(data), data)
            pycom.rgbled(red)
            lora_sock.send(pkg)
            print("Sent sync: "+data)
            pycom.rgbled(off)
            time.sleep_ms(13) # node time after sack
            print("sync lasted (ms):", (chrono.read_us()-sync_start)/1000)
        print("round lasted (ms):", (chrono.read_us()-round_start)/1000)
        i += 1

_thread.start_new_thread(update_index, ())
_thread.start_new_thread(receive_data, ())
