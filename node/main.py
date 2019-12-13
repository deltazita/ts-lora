# TS-LoRa (node side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3

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
from machine import Timer
import crypto
import uhashlib
from crypto import AES
import _thread
# from machine import SD
# from pytrack import Pytrack


### FUNCTIONS ###

def zfill(s, width):
    return '{:0>{w}}'.format(s, w=width)

def random_sleep(max_sleep):
    arg = "byteorder='big'"
    t = int.from_bytes(crypto.getrandbits(32), arg)
    time.sleep(1 + t % max_sleep) # wake-up at a random time

def join_request(_sf):
    global lora
    global DevEUI
    global index
    global lora_sock
    global active_tx
    global active_rx
    global chrono
    global my_slot
    global guard
    global AppKey
    global DevNonce
    global AppSKey
    msg = DevEUI+":"+JoinEUI+":"+str(DevNonce)+":"+str(_sf)
    pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
    i = 0
    while (i == 0):
        pycom.rgbled(blue)
        lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.TX_ONLY, bandwidth=LoRa.BW_125KHZ, sf=12, tx_power=7)
        start = chrono.read_ms()
        lora_sock.send(pkg)
        active_tx += chrono.read_ms()-start
        print("Request sent!", pkg)
        lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[1], power_mode=LoRa.ALWAYS_ON, bandwidth=LoRa.BW_125KHZ, sf=12)
        start = chrono.read_ms()
        while ((chrono.read_ms() - start) < 5000):
            pycom.rgbled(green)
            recv_pkg = lora_sock.recv(50)
            if (len(recv_pkg) > 2):
                recv_pkg_len = recv_pkg[1]
                recv_pkg_id = recv_pkg[0]
                if (int(recv_pkg_id) == 1):
                    dev_id, leng, msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                    print('Device: %d - Pkg:  %s' % (dev_id, msg))
                    msg = str(msg)[2:]
                    msg = msg[:-1]
                    (dev_id, DevAddr, JoinNonce) = str(msg).split(":")
                    if (int(dev_id) == int(MY_ID)):
                        start = -10000
                        pycom.rgbled(blue)
                        lora.power_mode(LoRa.SLEEP)
                        active_rx += chrono.read_ms()-start
                        i = 1
                        break
        if (i == 0):
            lora.power_mode(LoRa.SLEEP)
            active_rx += chrono.read_ms()-start
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

    lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-5], power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
    sync_start = chrono.read_ms()
    sack_rcv = 0
    print("Waiting for sync...")
    while (True):
        machine.idle()
        pycom.rgbled(white)
        recv_pkg = lora_sock.recv(100)
        if (len(recv_pkg) > 2):
            recv_pkg_len = recv_pkg[1]
            recv_pkg_id = recv_pkg[0]
            if (int(recv_pkg_id) == (my_sf-5)):
                sack_rcv = chrono.read_ms()
                dev_id, leng, s_msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                s_msg = str(s_msg)[2:]
                s_msg = s_msg[:-1]
                (index, guard, acks) = s_msg.split(":")
                (index, guard) = (int(index), int(guard)*1000)
                print("ACK!")
                # pycom.rgbled(blue)
                lora.power_mode(LoRa.SLEEP)
                active_rx += (chrono.read_ms() - sync_start)
                time.sleep_ms(10)
                break
    print("time after SACK rec (ms):", chrono.read_ms()-sack_rcv)
    print("sync slot lasted (ms):", chrono.read_ms()-sync_start)
    print("joining the network lasted (ms):", chrono.read_ms()-join_start)

def generate_msg():
    global msg
    msg = crypto.getrandbits(100*8)[:-2] # a random 98-byte message

### MAIN ###

wlan = WLAN()
wlan.deinit()
bt = Bluetooth()
bt.deinit()
server = Server()
server.deinit()
# py = Pytrack()
# ANSELC_ADDR = const(0x18E)
# py.poke_memory(ANSELC_ADDR, ~(1 << 7))

# sd = SD()
# os.mount(sd, '/sd')

_LORA_PKG_FORMAT = "!BB%ds"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
MY_ID = 0x1A
(my_sf, my_bw_index, guard, sync_rate, my_slot) = (7, 0, 15, 1, 0) # default values
index = 0
S = 1000
(retrans, succeeded, dropped) = (0, 0, 0)
freqs = [869700000, 869850000, 865000000, 865600000, 866200000, 866800000, 867400000, 868000000] # 2x125, 6x500
# airtimes for 100-byte packets and 8 preamble symbols [BW125, BW250, BW500] x 6 SFs
airtime = [[0.174336, 0.087168, 0.043584], [0.307712, 0.153856, 0.076928], [0.553984, 0.276992, 0.138496], [1.026048, 0.513024, 0.256512], [2.215936, 0.944128, 0.472064], [3.940352, 1.724416, 0.862208]]
if (my_bw_index == 0):
    my_bw = LoRa.BW_125KHZ
elif (my_bw_index == 1):
    my_bw = LoRa.BW_250KHZ
else:
    my_bw = LoRa.BW_500KHZ

# some default AppKeys for 25 nodes
AK = ["3878214125442A472D4B615064536756","7234753778217A25432A462D4A614E64","576D5A7134743777217A24432646294A","655368566D5971337436773979244226",
"4B6150645367566B5970337336763979","2A462D4A614E645267556B5870327335","7A24432646294A404E635266556A586E","36763979244226452948404D63516654",
"703373367638792F423F4528482B4D62","556B58703273357638782F413F442847","635266556A586E327235753878214125","48404D635166546A576E5A7234753777",
"3F4528482B4D6251655468576D5A7134","782F413F4428472B4B6250655368566D","35753778214125442A472D4B61506453","6E5A7234753777217A25432A462D4A61",
"5468576D5A7134743677397A24432646","6250655368566D597133743676397924","472D4B6150645367566B597033733676","25432A462D4A614E645267556B587032",
"77397A24432646294A404E635266556A","337336763979244226452948404D6351","6B59703373357638792F423F4528482B","5267556B58703273357538782F413F44",
"404E635266556A586E32723475377821","452948404D635166546A576E5A723474","2F423F4528482B4D6251655468576D5A","7538782F413F4428472B4B6250655368",
"5A7234753778214125442A472D4B6150","6A576E5A7134743777217A25432A462D","51655468576D5A7133743677397A2443","2B4B6250655368566D59713373367639",
"442A472D4B6150645367566B59703373","217A25432A462D4A614E645267556B58","743677397A24432646294A404E635266","5970337336763979244226452948404D"]

# Colors
pycom.heartbeat(False)
off = 0x000000
red = 0xFF0000
green = 0x00FF00
blue = 0x0000FF
white = 0xFFFAFA

lora = LoRa(mode=LoRa.LORA, rx_iq=True, frequency=freqs[my_sf-5], region=LoRa.EU868, power_mode=LoRa.SLEEP, bandwidth=my_bw, sf=my_sf)
DevEUI = str(binascii.hexlify(lora.mac()))[2:]
DevEUI = DevEUI[:-1]
print("LoRa DevEUI:", DevEUI)
JoinEUI = "3efd4267ef71836a" # this could be random
AppKey = AK[int(MY_ID)-11]
DevNonce = 1
lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
lora_sock.setblocking(False)

random_sleep(5)
chrono = Timer.Chrono()
chrono.start()
join_start = chrono.read_us()
# f = open('/sd/stats.txt', 'w')
active_rx = 0.0
active_tx = 0.0
join_request(my_sf)
repeats = 0
if (guard < 10000): # no guard bellow 10ms is allowed
    guard = 10000
airt = math.ceil(airtime[my_sf-7][my_bw_index]*1000000)
duty_cycle_limit_slots = math.ceil(100*airt/(airt + 2*guard))
sync_slot = 80000 # this depends on SF/BW/index/guard (us)
clock_correct = 0
proc_and_switch = 12000 # time for preparing the packet and switch radio mode (us)
msg = crypto.getrandbits(100*8)[:-2]
clocks = [sync_slot]
print("-----")
print("airtime (ms):", airt/1000)
print("Guard time (ms):", guard/1000)
print("Duty cycle slots:", duty_cycle_limit_slots)
print("SACK slot length (ms):", sync_slot/1000)

# send data
i = 1
sack_rcv = 0.0
print("S T A R T")
while(i <= 1500): # stop after 1500 packets
    print(i, "----------------------------------------------------")
    chrono.reset()
    start = chrono.read_us()
    pycom.rgbled(off)
    print("starting a new round at (ms):", start/1000)
    if (int(index) > duty_cycle_limit_slots):
        round_length = math.ceil(int(index)*(airt + 2*guard))
    else:
        round_length = math.ceil(duty_cycle_limit_slots*(airt + 2*guard))
    round_length += 17500 # gw proc+switch time (us)
    t = int(my_slot*(airt + 2*guard) + guard - proc_and_switch) # sleep time before transmission
    print("sleep time (ms):", t/1000)
    machine.idle()
    time.sleep_us(t)
    _thread.start_new_thread(generate_msg, ())
    pycom.rgbled(red)
    on_time = chrono.read_us()
    lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-5], power_mode=LoRa.TX_ONLY, bandwidth=my_bw, sf=my_sf, tx_power=14)
    pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
    print("Sending packet of", len(pkg), "bytes at (ms):", chrono.read_ms()-start/1000)
    lora_sock.send(pkg)
    # print(lora.stats())
    pycom.rgbled(off)
    lora.power_mode(LoRa.SLEEP)
    active_tx += (chrono.read_us() - on_time)
    t = round_length - int(chrono.read_us() - start) - int(clock_correct)
    print("sleep time after data (ms):", t/1000, "/ clock correction (ms):", clock_correct/1000)
    machine.idle()
    time.sleep_us(t)
    if (i % sync_rate == 0): # SACK
        rec = 0
        clock_correct = 0
        acks = ""
        lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-5], power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
        sync_start = chrono.read_us()
        print("started sync slot at:", sync_start/1000)
        while (rec == 0) and ((chrono.read_us() - sync_start) <= sync_slot):
            machine.idle()
            recv_pkg = lora_sock.recv(30)
            pycom.rgbled(white)
            if (len(recv_pkg) > 2):
                recv_pkg_len = recv_pkg[1]
                recv_pkg_id = recv_pkg[0]
                if (int(recv_pkg_id) == (my_sf-5)):
                    sack_rcv = chrono.read_us()
                    dev_id, leng, s_msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                    s_msg = str(s_msg)[2:]
                    s_msg = s_msg[:-1]
                    (index, guard, acks) = s_msg.split(":")
                    (index, guard) = (int(index), int(guard)*1000)
                    print("SACK received!", s_msg)
                    print(lora.stats())
                    lora.power_mode(LoRa.SLEEP)
                    active_rx += (chrono.read_us() - sync_start)
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
                    ack_lasted = (chrono.read_us()-sync_start)
                    if (ack_lasted > sync_slot*1.5): # sometimes the ack takes more time than it should
                        print("warning! SACK slot lasted longer than expected!")
                        clock_correct = int(chrono.read_us()-sync_start) - sync_slot
                    elif (ack_lasted < sync_slot/3): # normally this should't happen
                        print("warning! very short SACK length (ms):", (chrono.read_us()-sync_start)/1000)
                        time.sleep_us(sync_slot - int(chrono.read_us()-sync_start) + 0) # this must be tuned on
                    else: # adaptive SACK slot length
                        clocks.append(ack_lasted)
                        sync_slot = 0
                        for j in clocks:
                            sync_slot += j
                        sync_slot = int(sync_slot/len(clocks))
                        if (len(clocks) == 10):
                            clocks = [sync_slot]
                        print("new sync slot length (ms):", sync_slot/1000)
        if (rec == 0):
            lora.power_mode(LoRa.SLEEP)
            active_rx += (chrono.read_us() - sync_start)
            time.sleep_ms(5)
            if (clock_correct == 0):
                clock_correct = -guard
            elif (clock_correct == -guard):
                clock_correct = 0
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
                clock_correct = 1
                i += 1
                join_request(my_sf)
        print("sync slot lasted (ms):", (chrono.read_us()-sync_start)/1000)
        print("time after SACK (ms):", (chrono.read_us() - sack_rcv)/1000)
        print("transmitted/delivered/retransmitted/dropped:", i, succeeded, retrans, dropped)
        # f.write('transmitted/delivered/retransmitted: %d / %d / %d\n' % (i, succeeded, retrans))
    print("round lasted (ms):", (chrono.read_us()-start)/1000)
    print("radio active time (rx/tx) (s):", active_rx/1e6, "/", active_tx/1e6)
    if (chrono.read_us()-start-round_length-sync_slot-12000 > guard) and (clock_correct == 0):
        print("warning! frame lasted longer than expected!")
        clock_correct = chrono.read_us()-start-round_length-sync_slot-12000-guard
    i += 1
# f = open('/sd/stats.txt', 'w')
# f.write('Total packets transmitted: %d\n' % i)
# f.write('Total packets acknowledged: %d\n' % succeeded)
# f.write('Total packets retransmitted: %d\n' % retrans)
# f.close()

# send out stats
print("I'm sending stats")
msg = str(i-1)+":"+str(succeeded)+":"+str(retrans)+":"+str(dropped)
pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
for x in range(3): # send it out 3 times
    lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[7], power_mode=LoRa.TX_ONLY, bandwidth=LoRa.BW_125KHZ, sf=7, tx_power=14)
    pycom.rgbled(blue)
    lora_sock.send(pkg)
    lora.power_mode(LoRa.SLEEP)
    random_sleep(5)
pycom.rgbled(off)
