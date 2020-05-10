# TS-LoRa (node side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3
#
# Tested with firmware v1.20.1

import os
import sys
import socket
import time
import struct
from network import LoRa
from network import WLAN
from network import Bluetooth
from network import Server
import pycom
import machine
import binascii
import ubinascii
import math
import crypto
import uhashlib
from crypto import AES
import _thread
from OTA import WiFiOTA
# from machine import SD
# from pytrack import Pytrack


### FUNCTIONS ###

def OTA_update(_ip):
    ota = WiFiOTA("Guests@Tyndall", "", _ip, 8000)
    print("Performing OTA")
    pycom.rgbled(0xDB7093)
    try:
        ota.connect()
        ota.update()
    except:
        print("Cannot connect to server!")
        pass
    pycom.rgbled(0x000000)

def get_id():
    global MY_ID
    my_py_id = str(binascii.hexlify(machine.unique_id()))[2:][:-1]
    f = open("ids","r")
    for line in f:
        fields = line.split(" ")
        if (fields[0] == my_py_id):
            MY_ID = int(fields[1])
    print("Pycom id:", my_py_id, "My id:", MY_ID)

def zfill(s, width):
    return '{:0>{w}}'.format(s, w=width)

def random_sleep(max_sleep):
    arg = "byteorder='big'"
    t = int.from_bytes(crypto.getrandbits(32), arg)
    machine.sleep((1+t%max_sleep)*1000, 0) # wake-up at a random time

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

def join_request(_sf):
    global lora
    global DevEUI
    global lora_sock
    global active_tx
    global active_rx
    global my_slot
    global AppKey
    global DevNonce
    global AppSKey
    rmsg = DevEUI+":"+JoinEUI+":"+str(DevNonce)+":"+str(_sf)
    pkg = struct.pack(_LORA_PKG_FORMAT % len(rmsg), MY_ID, len(rmsg), rmsg)
    i = 0
    while (i == 0):
        pycom.rgbled(blue)
        lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.TX_ONLY, bandwidth=LoRa.BW_125KHZ, sf=12, tx_power=7)
        start = time.ticks_us()
        # while(lora.ischannel_free(-90) == False):
        #     time.sleep(1)
        lora_sock.send(pkg)
        active_tx += time.ticks_us()-start
        print("Request sent!", pkg)
        time.sleep_ms(100)
        lora_sock.setblocking(True)
        lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[1], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)
        start = time.ticks_us()
        pycom.rgbled(green)
        lora_sock.settimeout(5)
        try:
            while (True):
                recv_pkg = lora_sock.recv(50)
                if (len(recv_pkg) > 2):
                    recv_pkg_len = recv_pkg[1]
                    recv_pkg_id = recv_pkg[0]
                    if (int(recv_pkg_id) == 1):
                        dev_id, leng, rmsg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                        print('Device: %d - Pkg:  %s' % (dev_id, rmsg))
                        rmsg = str(rmsg)[2:]
                        rmsg = rmsg[:-1]
                        (dev_id, DevAddr, JoinNonce) = str(rmsg).split(":")
                        if (int(dev_id) == int(MY_ID)):
                            pycom.rgbled(blue)
                            lora.power_mode(LoRa.SLEEP)
                            active_rx += time.ticks_us()-start
                            start = -10000
                            i = 1
                            break
        except:
            if (i == 0):
                lora.power_mode(LoRa.SLEEP)
                active_rx += time.ticks_us()-start
                random_sleep(5)
                DevNonce += 1

    # AppSKey generation
    text = "".join( [AppKey[:2], JoinNonce, JoinEUI, str(DevNonce)] )
    while (len(str(text)) < 32):
        text = "".join([str(text),"0"])
    encryptor = AES(AppKey, AES.MODE_ECB)
    AppSKey = encryptor.encrypt(binascii.unhexlify(text))
    # slot generation
    text = "".join([DevAddr, DevEUI])
    thash = uhashlib.sha256()
    thash.update(text)
    thash = int(ubinascii.hexlify(thash.digest()), 16)
    my_slot = thash % S
    print("Slot =", my_slot, "DevAddr = ", DevAddr)
    print("joining the network lasted (s):", (time.ticks_us()-join_start)/1e6)
    sync()

def sync():
    global lora
    global index
    global lora_sock
    global active_tx
    global active_rx
    global proc_gw
    global sack_rcv
    global sack_bytes
    lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-5], power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
    sync_start = time.ticks_us()
    sack_rcv = 0
    pycom.rgbled(white)
    print("Waiting for sync...")
    lora_sock.settimeout(None)
    while (True):
        machine.idle()
        recv_pkg = lora_sock.recv(100)
        if (len(recv_pkg) > 2):
            recv_pkg_len = recv_pkg[1]
            recv_pkg_id = recv_pkg[0]
            if (int(recv_pkg_id) == (my_sf-5)):
                sack_rcv = time.ticks_us()
                dev_id, leng, s_msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                s_msg = str(s_msg)[2:]
                s_msg = s_msg[:-1]
                sack_bytes = recv_pkg_len
                (index, proc_gw, acks) = s_msg.split(":")
                (index, proc_gw) = (int(index), int(proc_gw)*1000)
                print("ACK!")
                lora.power_mode(LoRa.SLEEP)
                active_rx += (time.ticks_us() - sync_start)
                break
    print("sync slot lasted (ms):", (time.ticks_us()-sync_start)/1000)
    print("active time during join-sync (rx/tx) (ms):", active_rx/1000, "/", active_tx/1000)

def generate_msg():
    global msg
    msg = crypto.getrandbits(packet_size*8)
    while (len(msg) > packet_size): # just correct the rounding
        msg = msg[:-1]

def start_transmissions(_pkts):
    global lora
    global index
    global lora_sock
    global active_tx
    global active_rx
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
    global sack_bytes
    airt = int(airtime_calc(my_sf,1,packet_size+2,my_bw_plain)*1000)
    duty_cycle_limit_slots = math.ceil(100*airt/(airt + 2*guard))
    proc_and_switch = 12000 # time for preparing the packet and switch radio mode (us)
    if (int(MY_ID) == 22 or int(MY_ID) == 34): # fipy nodes switch faster
        proc_and_switch = 10000
    if (my_slot == -1):
        join_start = time.ticks_us()
        join_request(my_sf)
    else:
        sync()
    repeats = 0
    clock_correct = 0
    sync_slot = int(airtime_calc(my_sf,1,sack_bytes+2,my_bw_plain)*1000+3*guard/2)
    clocks = [sync_slot]
    print("-----")
    print("MY SLOT:", my_slot)
    print("Net size:", index)
    print("Time on air (ms):", airt/1000)
    print("Guard time (ms):", guard/1000)
    print("Duty cycle slots:", duty_cycle_limit_slots)
    print("Default SACK slot length (ms):", sync_slot/1000)
    print("Gw processing time (ms):", int(proc_gw/1000))
    print("Time after SACK rec (ms):", (time.ticks_us()-sack_rcv)/1000)

    i = 1
    delays = []
    delay = 0
    (succeeded, retrans, dropped, active_rx, active_tx) = (0, 0, 0, 0.0, 0.0)
    print("S T A R T")
    while(i <= _pkts): # stop after pkts # of packets
        print(i, "----------------------------------------------------")
        start = time.ticks_us()
        pycom.rgbled(green)
        print("starting a new round at (ms):", start/1000)
        # calculate the time until the sack packet (round_length)
        if (int(index) > duty_cycle_limit_slots):
            round_length = math.ceil(int(index)*(airt + 2*guard))
        else:
            round_length = math.ceil(duty_cycle_limit_slots*(airt + 2*guard))
        if (len(delays) > 0):
            delay = sum(delays) / len(delays)
        round_length += delay + proc_gw # clock delays + gw proc time (us)
        t = (my_slot*(airt + 2*guard) + guard - proc_and_switch) # sleep time before transmission
        print("sleep time (ms):", t/1000)
        pycom.rgbled(off)
        # time.sleep_us(int(t))
        machine.sleep(int(t/1000), 0) # light sleep mode
        _thread.start_new_thread(generate_msg, ())
        pycom.rgbled(red)
        on_time = time.ticks_us()
        lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-5], power_mode=LoRa.TX_ONLY, bandwidth=my_bw, sf=my_sf, tx_power=14)
        pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
        print("Sending packet of", len(pkg), "bytes at (ms):", (time.ticks_us()-start)/1000)
        lora_sock.send(pkg)
        pycom.rgbled(off)
        lora.power_mode(LoRa.SLEEP)
        active_tx += (time.ticks_us() - on_time)
        t = round_length - (time.ticks_us() - start)
        if t < 0:
            t = 0
            print("cannot align clock!")
        print("sleep time after data (s):", t/1e6, "/ clock correction (ms):", clock_correct/1000)
        # time.sleep_us(int(t))
        machine.sleep(int(t/1000), 0) # light sleep mode
        rec = 0
        sack_rcv = 0
        clock_correct = 0
        acks = ""
        lora_sock.setblocking(True)
        lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-5], power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
        sync_start = time.ticks_us()
        print("started sync slot at (ms):", time.ticks_ms())
        pycom.rgbled(white)
        lora_sock.settimeout(sync_slot/1e6)
        try:
            while (rec == 0):
                machine.idle()
                sack_rcv = time.ticks_us()
                recv_pkg = lora_sock.recv(255)
                if (len(recv_pkg) > 2):
                    recv_pkg_len = recv_pkg[1]
                    recv_pkg_id = recv_pkg[0]
                    if (int(recv_pkg_id) == (my_sf-5)):
                        sack_rcv = time.ticks_us()
                        lora.power_mode(LoRa.SLEEP)
                        active_rx += (time.ticks_us() - sync_start)
                        wt = sack_rcv-sync_start-airtime_calc(my_sf,1,recv_pkg_len,my_bw_plain)*1000
                        print("Waiting time before receiving SACK (ms):", wt/1000)
                        if (wt != guard) and (i > 1):
                            delays.append(wt-guard)
                        if (len(delays) > 3):
                            delays.pop(0)
                        clock_correct = wt - guard
                        try:
                            dev_id, leng, s_msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                            s_msg = str(s_msg)[2:][:-1]
                            (index, proc_gw, acks) = s_msg.split(":")
                            (index, proc_gw) = (int(index), int(proc_gw)*1000)
                            print("SACK received!", s_msg)
                            print(lora.stats())
                            if (acks != ""):
                                acks = zfill(bin(int(acks, 16))[2:], index)
                                if (acks[my_slot] == "1"):
                                    print("ACK!")
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
                            ack_lasted = time.ticks_us()-sync_start
                            if (i == 1): # what if the first packet is dropped. I have to fix this
                                clocks = [ack_lasted]
                            else:
                                clocks.append(ack_lasted)
                                sync_slot = int(sum(clocks)/len(clocks))
                                if (len(clocks) == 10):
                                    clocks = pop(0)
                            print("new sync slot length (ms):", sync_slot/1000)
                        except:
                            print("wrong SACK format!")
                            pass
        except:
            if (rec == 0):
                lora.power_mode(LoRa.SLEEP)
                active_rx += time.ticks_us() - sync_start
                print("I will repeat the last packet")
                retrans += 1
                repeats += 1
                i -= 1
                if (repeats == 4):
                    print("Packet dropped!")
                    print("Synchronisation lost!")
                    repeats = 0
                    dropped += 1
                    retrans -= 1
                    i += 1
                    pycom.rgbled(red)
                    time.sleep_us(int(round_length-sync_slot-proc_gw))
                    sync()
                    clock_correct = 0
        print("sync slot lasted (ms):", (time.ticks_us()-sync_start)/1000)
        print("time after SACK (ms):", (time.ticks_us()-sack_rcv)/1000)
        print("round lasted (ms):", (time.ticks_us()-start)/1000)
        print("transmitted/delivered/retransmitted/dropped:", i, succeeded, retrans, dropped)
        print("radio active time (rx/tx) (s):", active_rx/1e6, "/", active_tx/1e6)
        i += 1

    # send out stats
    print("I'm sending stats")
    stat_msg = str(i-1)+":"+str(succeeded)+":"+str(retrans)+":"+str(dropped)+":"+str(active_rx/1e6)+":"+str(active_tx/1e6)
    pkg = struct.pack(_LORA_PKG_FORMAT % len(stat_msg), MY_ID, len(stat_msg), stat_msg)
    for x in range(3): # send it out 3 times
        lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[7], power_mode=LoRa.TX_ONLY, bandwidth=LoRa.BW_125KHZ, sf=12, tx_power=7)
        pycom.rgbled(blue)
        # while (lora.ischannel_free(-90) == False):
        #     print("Channel is busy!")
        #     random_sleep(5)
        lora_sock.send(pkg)
        lora.power_mode(LoRa.SLEEP)
        random_sleep(10)
    pycom.rgbled(off)


### MAIN ###

MY_ID = 0x00 # to be filled in get_id
pycom.heartbeat(False)
get_id()
bt = Bluetooth()
bt.deinit()
server = Server()
server.deinit()
wlan = WLAN(mode=WLAN.STA)
wlan.deinit() # remove these two lines if you activate OTA
pybytes.smart_config(False) # disable provisioning
# py = Pytrack()
# ANSELC_ADDR = const(0x18E)
# py.poke_memory(ANSELC_ADDR, ~(1 << 7))

_LORA_PKG_FORMAT = "!BB%ds"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
(my_sf, my_bw_index, my_bw_plain, guard, my_slot, packet_size) = (7, 0, 0, 15000, -1, 16) # default values
index = 0
S = 1000
active_rx = 0.0
active_tx = 0.0
proc_gw = 4000 # gw default (minimum) processing time (us)
(sack_rcv, sack_bytes) = (0, 0) # will be filled later
msg = crypto.getrandbits(packet_size*8) # just a random packet
while (len(msg) > packet_size): # just correct the rounding
    msg = msg[:-1]
print("Packet size =", len(msg))
(retrans, succeeded, dropped) = (0, 0, 0)
freqs = [868100000, 868300000, 868500000, 867100000, 867300000, 867500000, 867700000, 867900000]
if (my_bw_index == 0):
    my_bw = LoRa.BW_125KHZ
    my_bw_plain = 125
elif (my_bw_index == 1):
    my_bw = LoRa.BW_250KHZ
    my_bw_plain = 250
else:
    my_bw = LoRa.BW_500KHZ
    my_bw_plain = 500

# 25 default AppKeys for testing
AK = ["3878214125442A472D4B615064536756","7234753778217A25432A462D4A614E64","576D5A7134743777217A24432646294A","655368566D5971337436773979244226",
"4B6150645367566B5970337336763979","2A462D4A614E645267556B5870327335","7A24432646294A404E635266556A586E","36763979244226452948404D63516654",
"703373367638792F423F4528482B4D62","556B58703273357638782F413F442847","635266556A586E327235753878214125","48404D635166546A576E5A7234753777",
"3F4528482B4D6251655468576D5A7134","782F413F4428472B4B6250655368566D","35753778214125442A472D4B61506453","6E5A7234753777217A25432A462D4A61",
"5468576D5A7134743677397A24432646","6250655368566D597133743676397924","472D4B6150645367566B597033733676","25432A462D4A614E645267556B587032",
"77397A24432646294A404E635266556A","337336763979244226452948404D6351","6B59703373357638792F423F4528482B","5267556B58703273357538782F413F44",
"404E635266556A586E32723475377821"]

# Colors
off = 0x000000
red = 0xFF0000
green = 0x00FF00
blue = 0x0000FF
white = 0xFFFAFA

lora = LoRa(mode=LoRa.LORA, rx_iq=True, frequency=freqs[my_sf-5], region=LoRa.EU868, power_mode=LoRa.SLEEP, bandwidth=my_bw, sf=my_sf)
DevEUI = str(binascii.hexlify(lora.mac()))[2:][:-1]
print("LoRa DevEUI:", DevEUI)
JoinEUI = "3efd4267ef71836a" # this could be random
AppKey = AK[int(MY_ID)-11]
DevNonce = 1
lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

# uncomment this line if you don't want OTA update and repeated experiments
start_transmissions(1000)

# uncomment those lines if you do want OTA update and repeated experiments
# lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)
# print("Waiting for commands...")
# pycom.rgbled(green)
# while(True):
#     recv_pkg = lora_sock.recv(30)
#     if (len(recv_pkg) > 2):
#         recv_pkg_id = recv_pkg[0]
#         recv_pkg_len = recv_pkg[1]
#         if (int(recv_pkg_id) == 0):
#             try: # the nodes are waiting for an init command of the form: 'IP_of_the_OTA_server:number_of_packets'. Check gateway-stats code
#                 dev_id, leng, ippkts = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
#                 ippkts = str(ippkts)[2:][:-1]
#                 (ip, pkts) = ippkts.split(":")
#                 pkts = int(pkts)
#                 pycom.rgbled(blue)
#                 if (my_slot == -1):
#                     random_sleep(20)
#                 print("OTA over WLAN (IP):", ip)
#                 OTA_update(ip)
#                 time.ticks_us()
#                 join_start = time.ticks_us()
#                 # wlan.deinit()
#                 start_transmissions(pkts)
#                 print("...experiment done!")
#                 lora_sock.setblocking(False)
#                 lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)
#                 print("ready for a new one...")
#                 pycom.rgbled(green)
#             except:
#                 print("something went wrong!")
