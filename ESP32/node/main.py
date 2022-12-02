# TS-LoRa (node side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3
#
# Tested with Heltec LoRa v2 433MHz SX1278 and LILYGO TTGO 868/915MHz SX1276
# micropython v1.19.1 for ESP32 LILYGO

import os
import sys
import time
import struct
from lora import LoRa
import machine
import ubinascii
import math
import uhashlib
import _thread
# from OTA import WiFiOTA
from chrono import Chrono
from machine import SoftI2C, Pin, SPI
import ssd1306
import network
import random
from cryptolib import aes
import gc

### FUNCTIONS ###

# def OTA_update(_ip):
#     ota = WiFiOTA("OTA_SSID", "", _ip, 8000)
#     print("Performing OTA")
#     pycom.rgbled(0xDB7093)
#     try:
#         ota.connect()
#         ota.update()
#     except:
#         print("Cannot connect to server!")
#         pass
#     pycom.rgbled(0x000000)

# ids are used for the experiments convenience but are not required for the protocol operation
def get_id():
    global MY_ID
    global my_mac
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    my_mac = ubinascii.hexlify(wlan.config('mac')).decode()
    wlan.active(False)
    print(my_mac)
    f = open("ids","r")
    for line in f:
        fields = line.split(" ")
        if (fields[0] == my_mac):
            MY_ID = int(fields[1])
    print("WIFI MAC:", my_mac, "-> My id:", MY_ID)

def random_sleep(max_sleep):
    t = random.getrandbits(32)
    time.sleep(1+t%max_sleep) # wake-up at a random time

def zfill(s, width):
    return '{:0>{w}}'.format(s, w=width)

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

def req_handler(recv_pkg):
    global DevAddr
    global JoinNonce
    global oled_list
    # print(recv_pkg)
    if (len(recv_pkg) > 2):
        recv_pkg_len = recv_pkg[1]
        recv_pkg_id = recv_pkg[0]
        if (int(recv_pkg_id) == 1):
            # TO DO: check if the packet arrived correctly
            (id, leng, dev_id, DevAddr, JoinNonce) = struct.unpack("BBBII", recv_pkg)
            print('Received response from', id, dev_id, hex(DevAddr), JoinNonce)
            if (int(dev_id) == int(MY_ID)):
                oled_list.append("Rcved join accept")
                oled_lines()
    lora.recv()

def join_request():
    global lora
    global DevEUI
    global active_tx
    global active_rx
    global chrono
    global my_slot
    global AppKey
    global DevNonce
    global AppSKey
    global oled_list
    lora.set_spreading_factor(12)
    lora.set_frequency(freqs[0])
    rmsg = hex(DevEUI)+hex(JoinEUI)+str(DevNonce)+str(my_sf)
    print(MY_ID, len(rmsg), "1", hex(DevEUI), hex(JoinEUI), DevNonce, my_sf)
    pkg = struct.pack("BBBQQIB", MY_ID, len(rmsg), 0x01, DevEUI, JoinEUI, DevNonce, my_sf)
    while (True):
        lora.standby()
        led.value(1)
        start = chrono.read_ms()
        lora.send(pkg)
        lora.sleep()
        led.value(0)
        active_tx += chrono.read_ms()-start
        print("Join request sent!")
        oled_list.append("Join req. sent")
        oled_lines()
        time.sleep_ms(100)
        lora.set_frequency(freqs[1])
        start = chrono.read_ms()
        lora.on_recv(req_handler)
        lora.recv()
        while(chrono.read_ms() - start < 5000):
            if(DevAddr != ""):
                break
        lora.sleep()
        active_rx += chrono.read_ms()-start
        if (str(DevAddr) == ""):
            print("No answer received!")
            random_sleep(5)
            DevNonce += 0x00000001
        else:
            break

    # AppSKey generation
    text = struct.pack("BIiI", 0x02, JoinNonce, 0xFFFFFF, DevNonce)
    while (len(text) % 16 != 0):
        text = "".join([text,"0"])
    cipher_en = aes(AppKey, 1)
    AppSKey = cipher_en.encrypt(text)
    print("Length of the text and AppSKey:", len(text), len(AppSKey))
    # slot generation
    text = "".join([hex(DevAddr)[2:], hex(DevEUI)[2:]])
    thash = uhashlib.sha256()
    thash.update(text)
    thash = int(ubinascii.hexlify(thash.digest()), 16)
    my_slot = thash % S
    print("Slot =", my_slot, "DevAddr =", hex(DevAddr))
    oled_list.append("Slot="+str(my_slot))
    oled_lines()
    print("joining the network lasted (ms):", chrono.read_ms()-join_start)
    sync()

def sync_handler(recv_pkg):
    global lora
    global index
    global active_tx
    global active_rx
    global chrono
    global proc_gw
    global sack_rcv
    global sack_bytes
    # print(recv_pkg)
    if (len(recv_pkg) > 2):
        ack_len = recv_pkg[1]
        recv_pkg_id = recv_pkg[0]
        if (int(recv_pkg_id) == (my_sf-5)):
            sack_rcv = chrono.read_us()
            (id, leng, index, proc_gw, acks) = struct.unpack("BBBB%ds" % ack_len, recv_pkg)
            sack_bytes = 4 + ack_len
            print("ACK!")
            oled_list.append("Synchronized!")
            oled_lines()

def sync():
    global lora
    global index
    global active_tx
    global active_rx
    global chrono
    global sack_rcv
    global oled_list
    lora.set_spreading_factor(my_sf)
    lora.set_frequency(freqs[my_sf-5])
    sync_start = chrono.read_us()
    sack_rcv = 0
    index = 0
    oled_list.append("Waiting for SACK")
    oled_lines()
    print("Waiting for SACK...")
    lora.on_recv(sync_handler)
    lora.recv()
    while (index == 0):
        machine.idle()
        if (index > 0):
            active_rx += (chrono.read_us() - sync_start)
    print("sync slot lasted (ms):", (chrono.read_us()-sync_start)/1000)
    print("active time during join-sync rx/tx (ms):", active_rx/1000, active_tx/1000)

def generate_msg():
    global msg
    msg = random.getrandbits(32) # just a random 4-byte int
    msg = hex(msg)[2:]
    while (len(msg) < packet_size):
        msg = msg + msg
    while (len(msg) > packet_size): # just correct the size
        msg = msg[:-1]

def sack_handler(recv_pkg):
    global index
    global chrono
    global proc_gw
    global sack_rcv
    global clock_correct
    global sync_start
    global fpas
    global acks
    global oled_list
    global corrections
    # print(recv_pkg[0], recv_pkg[2])
    if (len(recv_pkg) > 2):
        ack_len = recv_pkg[1]
        recv_pkg_id = recv_pkg[0]
        if (int(recv_pkg_id) == (my_sf-5)):
            sack_rcv = chrono.read_us()
            rssi = lora.get_rssi()
            print("SACK received!", rssi)
            oled_list.append("SACK rcved ("+str(rssi)+")")
            oled_lines()
            wt = sack_rcv-sync_start-airtime_calc(my_sf,1,sack_bytes,my_bw_plain)*1000
            print("Waiting time before receiving SACK (ms):", wt/1000)
            if (wt != guard) and (fpas == 0): # first packet after sync may delay a bit
                clock_correct = wt - guard # leave some confidence gap
                if (abs(clock_correct) > 150000): # something is wrong if this happens
                    clock_correct = 0
                corrections.append(clock_correct)
                if (len(corrections) > 10):
                    corrections.pop(0)
                clock_correct = sum(corrections) / len(corrections)
            (id, leng, index, proc_gw, acks) = struct.unpack("BBBB%ds" % ack_len, recv_pkg)
            acks = acks.decode("utf-8")

def start_transmissions():
    global lora
    global index
    global active_tx
    global active_rx
    global chrono
    global my_slot
    global proc_gw
    global AppSKey
    global succeeded
    global retrans
    global dropped
    global join_start
    global sack_rcv
    global msg
    global sync_start
    global fpas
    global acks
    global clock_correct
    global oled_list
    global corrections
    airt = int(airtime_calc(my_sf,1,packet_size+2,my_bw_plain)*1000)
    slot = airt + 2*guard
    duty_cycle_limit_slots = math.ceil(100*airt/slot)
    proc_and_switch = 7000 # time for preparing the packet and switch radio mode (us)
    chrono.reset()
    if (my_slot == -1):
        join_start = chrono.read_us()
        join_request()
    else:
        sync()
    fpas = 1
    repeats = 0
    clock_correct = 0
    default_sync_slot = int(airtime_calc(my_sf,1,sack_bytes,my_bw_plain)*1000+2*guard)
    sync_slot = default_sync_slot
    clocks = [sync_slot]
    corrections = []
    print("-----")
    print("MY SLOT:", my_slot)
    print("Net size:", index)
    print("Time on air (ms):", airt/1000)
    print("Guard time (ms):", guard/1000)
    print("Slot size (us):", slot)
    print("Duty cycle slots:", duty_cycle_limit_slots)
    print("SACK slot length (ms):", sync_slot/1000)
    print("Gw processing time (ms):", proc_gw)
    print("Time after SACK rec (ms):", (chrono.read_us()-sack_rcv)/1000)
    i = 0x0001
    (succeeded, retrans, dropped, active_rx, active_tx) = (0, 0, 0, 0.0, 0.0)
    print("S T A R T")
    while(1):
        print(i, "----------------------------------------------------")
        chrono.reset()
        start = chrono.read_us()
        print("starting a new round at (ms):", start/1000)
        # calculate the time until the sack packet
        if (int(index) > duty_cycle_limit_slots):
            round_length = math.ceil(index*slot + sync_slot + 100000)
        else:
            round_length = math.ceil(duty_cycle_limit_slots*slot + default_sync_slot + 100000)
        # round_length += proc_gw # gw proc time (us)
        print("Round length (ms):", round_length/1000)
        t = int(my_slot*(airt + 2*guard) + guard - proc_and_switch) # sleep time before transmission
        time.sleep_us(t)
        _thread.start_new_thread(generate_msg, ())
        # print("Before encryption:", msg)
        msg = aes(AppSKey, 1).decrypt(msg)
        # led.value(1)
        on_time = chrono.read_us()
        pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
        print("Sending packet of", len(pkg), "bytes at (ms):", (chrono.read_us()-start)/1000)
        lora.send(pkg)
        # led.value(0)
        lora.sleep()
        oled_list.append("Uplink sent!")
        oled_lines()
        active_tx += (chrono.read_us() - on_time)
        t = int(round_length - (sync_slot + 100000) -
                (chrono.read_us() - start) + clock_correct - guard)
        if fpas == 0:
            t -= slot
        if t < 0:
            print("Cannot align clock!")
            t = 0
        print("sleep time after data (ms):", t/1000, "/ round correction (ms):", clock_correct/1000)
        machine.idle()
        time.sleep_us(t)
        rec = 0
        sack_rcv = 0
        clock_correct = 0
        acks = ""
        print("started sync slot at (ms):", chrono.read_ms())
        oled_list.append("Waiting for SACK")
        oled_lines()
        sync_start = chrono.read_us()
        led.value(1)
        lora.on_recv(sack_handler)
        lora.recv()
        while(chrono.read_us() - sync_start < 2*sync_slot):
            # while(1):
            machine.idle()
            if (acks != ""):
                break
        led.value()
        lora.sleep()
        active_rx += (chrono.read_us() - sync_start)
        print("ACK pkt =", acks)
        if (acks != ""): # if a SACK has been received
            bin_ack = ""
            while(len(acks) > 0):
                # ack_ = str(bin(int(acks[:1], 16)))[2:]
                try:
                    bin_ack += zfill(bin(int(acks[:1], 16))[2:], index if index<4 else 4)
                except:
                    print("Bad ack format!")
                else:
                    acks = acks[1:]
            print(bin_ack)
            if (bin_ack[my_slot] == "1"): # if the uplink has been delivered
                succeeded += 0x0001
                repeats = 0
                print("OK!")
                oled_list.append("OK ("+str(succeeded)+"/"+str(i)+")")
                oled_lines()
            else:
                print("I will repeat the last packet")
                oled_list.append("NOT OK")
                oled_lines()
                retrans += 0x0001
                repeats += 0x0001
                i -= 0x0001
                if (repeats == 4):
                    print("Packet dropped!")
                    repeats = 0
                    dropped += 0x0001
                    retrans -= 0x0001
                    i += 0x0001
            machine.idle()
            rec = 1
            fpas = 0
            # adaptive SACK slot length
            ack_lasted = (chrono.read_us()-sync_start)
            if (clocks[-1] == sync_slot):
                clocks = [ack_lasted]
            else:
                if (ack_lasted < clocks[-1]*1.1): # do not take into account very delayed syncs
                    clocks.append(ack_lasted)
                    sync_slot = int(sum(clocks)/len(clocks))
                    if (len(clocks) == 10):
                        clocks = [sync_slot]
            print("new sync slot length (ms):", sync_slot/1000)
            if (chrono.read_us()-sack_rcv) < 99000:
                time.sleep_us(int(100000-(chrono.read_us()-sack_rcv)))
            print("time after SACK (ms):", (chrono.read_us()-sack_rcv)/1000)
        else:
            print("SACK missed!")
            oled_list.append("SACK missed")
            oled_lines()
            retrans += 0x0001
            repeats += 0x0001
            i -= 0x0001
            if (repeats == 4):
                print("Packet dropped!")
                print("Synchronisation lost!")
                repeats = 0
                dropped += 0x0001
                retrans -= 0x0001
                i += 0x0001
                oled_list.append("Packet dropped")
                oled_lines()
                time.sleep_us(int(round_length-sync_slot-int(proc_gw*1000)))
                sync()
                clock_correct = 0
                fpas = 1
        print("sync slot lasted (ms):", (chrono.read_us()-sync_start)/1000)
        print("round lasted (ms):", (chrono.read_us()-start)/1000)
        print("transmitted/delivered/retransmitted/dropped:", i, succeeded, retrans, dropped)
        print("radio active time (rx/tx) (s):", active_rx/1e6, "/", active_tx/1e6)
        i += 0x0001 # are 2 bytes enough? reserve more bytes for long experiments

    # send out stats
    print("I'm sending stats")
    # print("Sending", MY_ID, i-1, "2", succeeded, retrans, dropped, str(active_rx/1e6), str(active_tx/1e6))
    leng = len(bytes(MY_ID+(i-1)+(0x02)+succeeded+retrans+dropped))+8
    pkg = struct.pack("BBBHHHHff", MY_ID, leng, 0x02, int(i-1), int(succeeded), int(retrans),
                        int(dropped), active_rx/1e6, active_tx/1e6) # are 2 bytes enough?
    lora.set_spreading_factor(12)
    lora.set_frequency(freqs[0])
    for x in range(3): # send it out 3 times
        led.value(1)
        lora.send(pkg)
        lora.sleep()
        random_sleep(5)
    led.value(0)

def init_handler(recv_pkg):
    print(recv_pkg)
    if (len(recv_pkg) > 2):
        recv_pkg_id = recv_pkg[0]
        recv_pkg_len = recv_pkg[1]
        if (int(recv_pkg_id) == 1):
            dev_id, leng, ippkts = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
            ippkts = ippkts.decode('utf-8')
            pkts = int(ippkts)
            print("Starting experiment with", pkts, "packets")
            if (my_slot == -1):
                random_sleep(20)
            led.value(1)
            # uncomment the following 2 lines if you use OTA
            # print("OTA over WLAN (IP):", ip)
            # OTA_update(ip)
            chrono.start()
            join_start = chrono.read_us()
            start_transmissions(pkts)
            print("...experiment done!")
            lora.set_spreading_factor(12)
            lora.set_frequency(freqs[0])
            print("ready for a new one...")

def oled_lines():
    global oled_list
    oled.fill(0)
    oled.text("TS-LoRa "+"SF"+str(my_sf)+" ID"+str(MY_ID), 0, 0)
    l = 13
    if len(oled_list) > 6:
        oled_list.pop(0)
    for line in oled_list:
        oled.text(line, 0, l)
        l += 8
    oled.show()
    gc.collect()

### MAIN ###

#led = Pin(25,Pin.OUT) # heltec V2
led = Pin(2,Pin.OUT) # TTGO
rst = Pin(16, Pin.OUT)
rst.value(1)
scl = Pin(15, Pin.OUT, Pin.PULL_UP)
sda = Pin(4, Pin.OUT, Pin.PULL_UP)
i2c = SoftI2C(scl=scl, sda=sda, freq=450000)
oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3c)
oled.poweron()
oled_list = []

# SPI pins
SCK  = 5
MOSI = 27
MISO = 19
CS   = 18
RX   = 26
DIO2 = 34

spi = SPI(
    1,
    baudrate=1000000,
    sck=Pin(SCK, Pin.OUT, Pin.PULL_DOWN),
    mosi=Pin(MOSI, Pin.OUT, Pin.PULL_UP),
    miso=Pin(MISO, Pin.IN, Pin.PULL_UP),
)

lora = LoRa( spi, cs=Pin(CS, Pin.OUT), rx=Pin(RX, Pin.IN), cad=Pin(DIO2, Pin.IN))

spi.init()

MY_ID = 0x0B # to be filled in get_id (1 byte, so up to 256 devices)
my_mac = " "
DevAddr = ""
get_id()

_LORA_PKG_FORMAT = "!BB%ds"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
(my_sf, my_bw_plain, guard, my_slot, packet_size) = (0x07, 125, 15000, -1, 16) # default values
index = 0
S = 1000
active_rx = 0.0
active_tx = 0.0
proc_gw = 4000 # gw default (minimum) processing time (us)
(sack_rcv, sack_bytes) = (0, 0) # will be filled later
msg = random.getrandbits(32) # just a random 4-byte int
msg = hex(msg)[2:]
while (len(msg) < packet_size):
    msg = msg + msg
while (len(msg) > packet_size): # just correct the size
    msg = msg[:-1]
print("Packet size =", len(msg), "bytes")
(retrans, succeeded, dropped) = (0, 0, 0)
freqs = [868.1, 868.3, 868.5, 867.1, 867.3, 867.5, 867.7, 867.9]
# freqs = [903.9, 904.1, 904.3, 904.5, 904.7, 904.9, 905.1, 905.3]
# freqs = [433.175, 433.325, 433.475, 433.625, 433.775, 433.925, 434.075, 434.225] # 433.175 - 434.665 according to heltec

# 25 default AppKeys for testing
AK = ["3878214125442A472D4B615064536756","7234753778217A25432A462D4A614E64","576D5A7134743777217A24432646294A","655368566D5971337436773979244226",
"4B6150645367566B5970337336763979","2A462D4A614E645267556B5870327335","7A24432646294A404E635266556A586E","36763979244226452948404D63516654",
"703373367638792F423F4528482B4D62","556B58703273357638782F413F442847","635266556A586E327235753878214125","48404D635166546A576E5A7234753777",
"3F4528482B4D6251655468576D5A7134","782F413F4428472B4B6250655368566D","35753778214125442A472D4B61506453","6E5A7234753777217A25432A462D4A61",
"5468576D5A7134743677397A24432646","6250655368566D597133743676397924","472D4B6150645367566B597033733676","25432A462D4A614E645267556B587032",
"77397A24432646294A404E635266556A","337336763979244226452948404D6351","6B59703373357638792F423F4528482B","5267556B58703273357538782F413F44",
"404E635266556A586E32723475377821"]

DevEUI = my_mac # make up a DevEUI
while(len(DevEUI) < 16):
    DevEUI = "".join(["f", DevEUI])
DevEUI = int("0x"+DevEUI)
print("LoRa DevEUI:", hex(DevEUI))
JoinEUI = 0xF10FB3DE32960229 # this could be random
AppKey = AK[int(MY_ID)-11]
DevNonce = 0x00000001 # 32 bit
chrono = Chrono()
oled_lines()

# uncomment the following two lines if you don't want to use the init_exp.py script (additional changes on gw_req are needed)
chrono.start()
start_transmissions()
#
print("Waiting for commands...")
oled_list.append("Waiting for commands")
oled_lines()
lora.set_spreading_factor(12)
lora.set_frequency(freqs[0])
lora.on_recv(init_handler)
lora.recv()
while(True):
    machine.idle()
