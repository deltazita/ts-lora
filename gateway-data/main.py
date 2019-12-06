# TS-LoRa (gateway-data side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3

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
(guard, sync_rate, prc) = (15, 1, 10)
freqs = [869700000, 869850000, 865000000, 865600000, 866200000, 866800000, 867400000, 868000000] # 2x125, 6x500
# airtimes for 100bytes + 8 symbol preamble
airtime = [[0.174336, 0.087168, 0.043584], [0.307712, 0.153856, 0.076928], [0.553984, 0.276992, 0.138496], [1.026048, 0.513024, 0.256512], [2.215936, 0.944128, 0.472064], [3.940352, 1.724416, 0.862208]]
index = 0
slot = {}
KEY = {}
my_bw_index = 0
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
if not wlan.isconnected():
    wlan.connect('rasp', auth=(WLAN.WPA2, 'lalalala'), timeout=5000)
    while not wlan.isconnected():
        machine.idle()
print("I got IP"+wlan.ifconfig()[0])

# print(ubinascii.hexlify(wlan.mac(),':').decode())

def airtime_calc(sf,cr,pl,bw):
    H = 0        # implicit header disabled (H=0) or not (H=1)
    DE = 0       # low data rate optimization enabled (=1) or not (=0)
    Npream = 8   # number of preamble symbol (12.25  from Utz paper)
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
    global prc
    lora = LoRa(mode=LoRa.LORA, rx_iq=True, frequency=freqs[my_sf-5], region=LoRa.EU868, power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
    lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    lora_sock.setblocking(False)
    guard = 1000*guard
    (overall_received, overall_sent) = (0, 0)
    airt = math.ceil((airtime[my_sf-7][my_bw_index])*1e6)
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
        pycom.rgbled(off)
        received = 0
        acks = []
        if (int(index) > duty_cycle_limit_slots):
            round_length = math.ceil(int(index)*(airt + 2*guard))
        else:
            round_length = math.ceil(duty_cycle_limit_slots*(airt + 2*guard))
        # round_length += 1000 # add 10ms to balance with the nodes SACK processing time
        # lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-5], power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
        # print("started receiving at (ms):", chrono.read_ms())
        while ((chrono.read_us() - round_start) < round_length):
            recv_pkg = lora_sock.recv(8192)
            if (len(recv_pkg) > 2):
                recv_pkg_len = recv_pkg[1]
                recv_pkg_id = recv_pkg[0]
                if (int(recv_pkg_id) <= 35) and (int(recv_pkg_len) == 98):
                    dev_id, leng, msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                    # print('Device: %d - Pkg:  %s' % (dev_id, msg))
                    if (len(msg) == 98): # format check
                        received += 1
                        print('Received from: %d' % dev_id)
                        acks.append(str(int(dev_id)))
        print(received, "packets received")
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
        print("proc time (ms):", chrono.read_us()-proc_t)
        if (i % sync_rate == 0): # SACK
            sync_start = chrono.read_us()
            pycom.rgbled(white)
            time.sleep_us(int(guard*3/2)) # let's make it long so all the nodes are up
            lora.init(mode=LoRa.LORA, tx_iq=True, frequency=freqs[my_sf-5], region=LoRa.EU868, power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf, tx_power=14)
            data = str(index+1)+":"+str(int(guard/1000))+":"+ack_msg
            pkg = struct.pack(_LORA_PKG_FORMAT % len(data), MY_ID, len(data), data)
            pycom.rgbled(red)
            lora_sock.send(pkg)
            print("Sent sync: "+data)
            lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-5], power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
            time.sleep_ms(math.ceil(airtime_calc(my_sf,1,len(data),my_bw_plain))) # wait until the nodes get the packet
            print("sync lasted (ms):", (chrono.read_us()-sync_start)/1000)
        print("round lasted (ms):", (chrono.read_us()-round_start)/1000)
        print("should last (ms):", round_length/1000)
        i += 1

_thread.start_new_thread(update_index, ())
_thread.start_new_thread(receive_data, ())
