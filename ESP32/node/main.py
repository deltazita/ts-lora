# TS-LoRa (node side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3
#
# Tested with Heltec LoRa v2 433MHz SX1278 and micropython v1.19.1 for ESP32 LILYGO

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

def zfill(s, width):
    return '{:0>{w}}'.format(s, w=width)

def random_sleep(max_sleep):
    t = random.getrandbits(32)
    time.sleep(1+t%max_sleep) # wake-up at a random time

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
    # print(recv_pkg)
    if (len(recv_pkg) > 2):
        recv_pkg_len = recv_pkg[1]
        recv_pkg_id = recv_pkg[0]
        if (int(recv_pkg_id) == 1):
            # TO DO: check if the packet arrived correctly
            (id, leng, dev_id, DevAddr, JoinNonce) = struct.unpack("BBBII", recv_pkg)
            print('Received response from', id, dev_id, hex(DevAddr), JoinNonce)
            if (int(dev_id) == int(MY_ID)):
                oled_lines("TS-LoRa", "ED SF7", str(MY_ID), "Rcved join accept", " ")

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
    lora.set_spreading_factor(12)
    lora.set_frequency(freqs[0])
    rmsg = hex(DevEUI)+hex(JoinEUI)+str(DevNonce)+str(my_sf)
    print(MY_ID, len(rmsg), "1", hex(DevEUI), hex(JoinEUI), DevNonce, my_sf)
    pkg = struct.pack("BBBQQIB", MY_ID, len(rmsg), 0x01, DevEUI, JoinEUI, DevNonce, my_sf)
    while (True):
        led.value(1)
        lora.standby()
        start = chrono.read_ms()
        lora.send(pkg)
        active_tx += chrono.read_ms()-start
        print("Join request sent!")
        oled_lines("TS-LoRa", "ED SF7", str(MY_ID), "Join req. sent", " ")
        time.sleep_ms(100)
        lora.set_frequency(freqs[1])
        start = chrono.read_ms()
        led.value(0)
        while(chrono.read_ms() - start < 5000):
            lora.recv()
            lora.on_recv(req_handler)
        lora.sleep()
        if (str(DevAddr) == ""):
            print("No answer received!")
            active_rx += chrono.read_ms()-start
            random_sleep(5)
            DevNonce += 1
        else:
            active_rx += chrono.read_ms()-start
            start = -10000
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
    oled_lines("TS-LoRa", "ED SF7", "ID="+str(MY_ID)+", Slot="+str(my_slot), "Going to sync", " ")
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
    print(recv_pkg)
    if (len(recv_pkg) > 2):
        recv_pkg_len = recv_pkg[1]
        recv_pkg_id = recv_pkg[0]
        if (int(recv_pkg_id) == (my_sf-5)):
            sack_rcv = chrono.read_us()
            (id, leng, index, proc_gw, acks) = struct.unpack("BBBfI", recv_pkg)
            (index, proc_gw) = (int(index), (proc_gw)*1000)
            print("ACK!")

def sync():
    global lora
    global index
    global active_tx
    global active_rx
    global chrono
    global sack_rcv
    lora.set_spreading_factor(my_sf)
    lora.set_frequency(freqs[my_sf-5])
    lora.standby()
    sync_start = chrono.read_us()
    sack_rcv = 0
    index = 0
    oled_lines("TS-LoRa", "ED SF7", "ID="+str(MY_ID)+", Slot="+str(my_slot), "Waiting for SACK", " ")
    print("Waiting for sync...")
    while (index == 0):
        machine.idle()
        lora.recv()
        lora.on_recv(sync_handler)
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
    print(recv_pkg[0], recv_pkg[2])
    if (len(recv_pkg) > 2):
        recv_pkg_len = recv_pkg[1]
        recv_pkg_id = recv_pkg[0]
        if (int(recv_pkg_id) == (my_sf-5)):
            sack_rcv = chrono.read_us()
            print("SACK received!", lora.get_rssi())
            oled_lines("TS-LoRa", "ED SF7", "ID="+str(MY_ID), "Waiting for SACK", "SACK received")
            wt = sack_rcv-sync_start-airtime_calc(my_sf,1,recv_pkg_len,my_bw_plain)*1000
            print("Waiting time before receiving SACK (ms):", wt/1000)
            if (wt != guard) and (fpas == 0): # first packet after sync may delay a bit
                clock_correct = wt - guard
            (id, leng, index, proc_gw, acks) = struct.unpack("BBBfI", recv_pkg)

def start_transmissions(_pkts):
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
    global guard
    global sack_rcv
    global msg
    global sync_start
    global fpas
    global acks
    airt = int(airtime_calc(my_sf,1,packet_size+2,my_bw_plain)*1000)
    duty_cycle_limit_slots = math.ceil(100*airt/(airt + 2*guard))
    proc_and_switch = 12000 # time for preparing the packet and switch radio mode (us)
    chrono.reset()
    if (my_slot == -1):
        join_start = chrono.read_us()
        join_request()
    else:
        sync()
    fpas = 1
    repeats = 0
    clock_correct = 0
    sync_slot = int(airtime_calc(my_sf,1,sack_bytes,my_bw_plain)*1000+2*guard)
    clocks = [sync_slot]
    print("-----")
    print("MY SLOT:", my_slot)
    print("Net size:", index)
    print("Time on air (ms):", airt/1000)
    print("Guard time (ms):", guard/1000)
    print("Duty cycle slots:", duty_cycle_limit_slots)
    print("SACK slot length (ms):", sync_slot/1000)
    print("Gw processing time (ms):", proc_gw/1e6)
    print("Time after SACK rec (ms):", (chrono.read_us()-sack_rcv)/1000)
    i = 1
    (succeeded, retrans, dropped, active_rx, active_tx) = (0, 0, 0, 0.0, 0.0)
    print("S T A R T")
    while(i <= _pkts): # stop after pkts # of packets
        print(i, "----------------------------------------------------")
        chrono.reset()
        start = chrono.read_us()
        oled_lines("TS-LoRa", "ED SF7", "ID="+str(MY_ID), "Round"+str(i), " ")
        print("starting a new round at (ms):", start/1000)
        # calculate the time until the sack packet
        if (int(index) > duty_cycle_limit_slots):
            round_length = math.ceil(int(index)*(airt + 2*guard))
        else:
            round_length = math.ceil(duty_cycle_limit_slots*(airt + 2*guard))
        # round_length += proc_gw # gw proc time (us)
        print("Round length (ms):", round_length/1000)
        t = int(my_slot*(airt + 2*guard) + guard - proc_and_switch) # sleep time before transmission
        # print("sleep time (ms):", t/1000)
        led.value(0)
        time.sleep_us(t)
        _thread.start_new_thread(generate_msg, ())
        # print("Before encryption:", msg)
        msg = aes(AppSKey, 1).decrypt(msg)
        led.value(1)
        on_time = chrono.read_us()
        lora.standby()
        pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
        print("Sending packet of", len(pkg), "bytes at (ms):", (chrono.read_us()-start)/1000)
        lora.send(pkg)
        led.value(0)
        lora.sleep()
        oled_lines("TS-LoRa", "ED SF7", "ID="+str(MY_ID), "Round"+str(i), "Uplink sent!")
        active_tx += (chrono.read_us() - on_time)
        t = int(round_length - (chrono.read_us() - start) + clock_correct - 80e3)
        if t < 0:
            t = 0
            print("cannot align clock!")
        print("sleep time after data (s):", t/1e6, "/ clock correction (ms):", clock_correct/1000)
        machine.idle()
        time.sleep_us(t)
        rec = 0
        sack_rcv = 0
        clock_correct = 0
        acks = ""
        lora.standby()
        print("started sync slot at (ms):", chrono.read_ms())
        oled_lines("TS-LoRa", "ED SF7", "ID="+str(MY_ID), "Round"+str(i), "Waiting for SACK")
        sync_start = chrono.read_us()
        # while(chrono.read_us() - sync_start < sync_slot):
        while(1): ### I need to fix this
            lora.recv()
            lora.on_recv(sack_handler)
            if (acks != ""):
                break
        lora.sleep()
        print("ACKS =", acks)
        if (acks != ""): # if a SACK has been received
            active_rx += (chrono.read_us() - sync_start)
            acks = str(bin(acks))[2:]
            acks = zfill(bin(int(acks, 16))[2:], index)
            if (acks[my_slot] == "1"): # if the uplink has been delivered
                print("OK!")
                oled_lines("TS-LoRa", "ED SF7", "ID="+str(MY_ID), "Waiting for SACK", "OK!")
                succeeded += 1
                repeats = 0
            else:
                print("I will repeat the last packet")
                retrans += 1
                repeats += 1
                i -= 1
                if (repeats == 4):
                    print("Packet dropped!")
                    repeats = 0
                    dropped += 1
                    retrans -= 1
                    i += 1
            machine.idle()
            rec = 1
            fpas = 0
            # adaptive SACK slot length
            ack_lasted = (chrono.read_us()-sync_start)
            if (i == 1): # what if the first packet is dropped. I have to fix this
                clocks = [ack_lasted]
            else:
                clocks.append(ack_lasted)
                sync_slot = int(sum(clocks)/len(clocks))
                if (len(clocks) == 10):
                    clocks = [sync_slot]
            print("new sync slot length (ms):", sync_slot/1000)
        else:
            active_rx += (chrono.read_us() - sync_start)
            print("I will repeat the last packet")
            retrans += 1
            repeats += 1
            i -= 1
            # clock_correct = -5000
            if (repeats == 4):
                print("Packet dropped!")
                print("Synchronisation lost!")
                repeats = 0
                dropped += 1
                retrans -= 1
                i += 1
                oled_lines("TS-LoRa", "ED SF7", "ID="+str(MY_ID), "Round"+str(i), "Packet dropped")
                time.sleep_us(int(round_length-sync_slot-proc_gw))
                sync()
                clock_correct = 0
                fpas = 1
        print("sync slot lasted (ms):", (chrono.read_us()-sync_start)/1000)
        print("time after SACK (ms):", (chrono.read_us()-sack_rcv)/1000)
        print("round lasted (ms):", (chrono.read_us()-start)/1000)
        print("transmitted/delivered/retransmitted/dropped:", i, succeeded, retrans, dropped)
        print("radio active time (rx/tx) (s):", active_rx/1e6, "/", active_tx/1e6)
        i += 1

    # send out stats (not ready yet)
    # print("I'm sending stats")
    # stat_msg = str(i-1)+":"+str(succeeded)+":"+str(retrans)+":"+str(dropped)+":"+str(active_rx/1e6)+":"+str(active_tx/1e6)
    # pkg = struct.pack(_LORA_PKG_FORMAT % len(stat_msg), MY_ID, len(stat_msg), stat_msg)
    # for x in range(3): # send it out 3 times
    #     lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.TX_ONLY, bandwidth=LoRa.BW_125KHZ, sf=12, tx_power=7)
    #     led.value(1)
    #     while (lora.ischannel_free(-90) == False):
    #         print("Channel is busy!")
    #         random_sleep(5)
    #     lora_sock.send(pkg)
    #     lora.power_mode(LoRa.SLEEP)
    #     random_sleep(5)
    # led.value(0)

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
            lora.standby()
            print("ready for a new one...")

### MAIN ###

led = Pin(25,Pin.OUT) # heltec V2
rst = Pin(16, Pin.OUT)
rst.value(1)
scl = Pin(15, Pin.OUT, Pin.PULL_UP)
sda = Pin(4, Pin.OUT, Pin.PULL_UP)
i2c = SoftI2C(scl=scl, sda=sda, freq=450000)
oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3c)
oled.poweron()

def oled_lines(line1, line2, line3, line4, line5):
    oled.fill(0)
    oled.text(line1, 0, 0)
    oled.text(line2, 0, 10)
    oled.text(line3, 0, 20)
    oled.text(line4, 0, 30)
    oled.text(line5, 0, 40)
    oled.show()

oled_lines("TS-LoRa", "ED SF7", " ", " ", " ")

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

lora = LoRa( spi, cs=Pin(CS, Pin.OUT), rx=Pin(RX, Pin.IN), )

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
(sack_rcv, sack_bytes) = (0, 11) # will be filled later
msg = random.getrandbits(32) # just a random 4-byte int
msg = hex(msg)[2:]
while (len(msg) < packet_size):
    msg = msg + msg
while (len(msg) > packet_size): # just correct the size
    msg = msg[:-1]
print("Packet size =", len(msg), "bytes")
(retrans, succeeded, dropped) = (0, 0, 0)
# freqs = [868.1, 868.3, 868.5, 867.1, 867.3, 867.5, 867.7, 867.9]
# freqs = [903.9, 904.1, 904.3, 904.5, 904.7, 904.9, 905.1, 905.3]
freqs = [433.175, 433.325, 433.475, 433.625, 433.775, 433.925, 434.075, 434.225] # 433.175 - 434.665 according to heltec

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

# uncomment the following two lines if you don't want to use the init_exp.py script (additional changes on gw_req are needed)
chrono.start()
start_transmissions(100)
#
print("Waiting for commands...")
oled_lines("TS-LoRa", "ED SF7", "Waiting for commands", " ", " ")
while(True):
    lora.recv()
    lora.on_recv(init_handler)
