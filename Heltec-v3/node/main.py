# TS-LoRa (node side)
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3
#
# Tested with Heltec LoRa v3 SX126x
# micropython v1.20.0 for Generic ESP32-S3

from time import ticks_ms, ticks_us, ticks_diff, sleep, sleep_us, sleep_ms
from machine import SoftI2C, Pin
from sx1262 import SX1262
from struct import pack, unpack
from ubinascii import crc32, hexlify
from math import ceil
from uhashlib import sha256
import _thread
from chrono import Chrono
import ssd1306
import network
import random
from cryptolib import aes

### FUNCTIONS ###

# ids are used for the experiments convenience but are not required for the protocol operation
def get_id():
    global MY_ID
    global my_mac
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    my_mac = hexlify(wlan.config('mac')).decode()
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
    sleep(1+t%max_sleep) # wake-up at a random time

# this is borrowed from LoRaSim (https://www.lancaster.ac.uk/scc/sites/lora/lorasim.html)
def airtime_calc(sf,cr,pl,bw):
    H = 0        # implicit header disabled (H=0) or not (H=1)
    DE = 0       # low data rate optimization enabled (=1) or not (=0)
    Npream = 8
    if bw == 125 and sf in [11, 12]:
        DE = 1
    Tsym = (2.0**sf)/bw
    Tpream = (Npream + 4.25)*Tsym
    payloadSymbNB = 8 + max(ceil((8.0*pl-4.0*sf+28+16-20*H)/(4.0*(sf-2*DE)))*(cr+4),0)
    Tpayload = payloadSymbNB * Tsym
    return Tpream + Tpayload

def req_handler(event):
    global DevAddr
    global JoinNonce
    global oled_list
    global join_accept
    if event & SX1262.RX_DONE:
        recv_pkg, err = lora.recv()
        print(recv_pkg)
        if (len(recv_pkg) > 2):
            recv_pkg_len = recv_pkg[1]
            recv_pkg_id = recv_pkg[0]
            if (int(recv_pkg_id) == 1):
                (id, dev_id, DevAddr, JoinNonce, rcrc) = unpack("BBIII", recv_pkg)
                # print('Received response from', id, dev_id, hex(DevAddr), JoinNonce, rcrc)
                msg = b''.join([int(dev_id).to_bytes(1, 'big'), DevAddr.to_bytes(4, 'big'), JoinNonce.to_bytes(4, 'big')])
                if (int(id) == 1 and int(dev_id) == int(MY_ID) and crc32(msg) == rcrc):
                    join_accept = 1
                    print("...join accept packet OK")
                    oled_list.append("Rcved join accept")
                    oled_lines()
                else:
                    print("...join accept packet FAILED")

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
    global join_accept
    # lora.set_implicit(True)
    # lora.set_crc(False)
    # lora.set_preamble_length(10)
    # lora.set_payload_length(14)
    # lora.set_coding_rate(5)
    lora.sleep()
    lora.setSpreadingFactor(j_sf)
    lora.setFrequency(freqs[-1])
    attempts = 0
    while (True):
        join_accept = 0
        rmsg = b''.join([b'1', DevEUI.to_bytes(8, 'big'), JoinEUI.to_bytes(8, 'big'), DevNonce.to_bytes(4, 'big'), my_sf.to_bytes(1, 'big')])
        crc = crc32(rmsg)
        print(MY_ID, "1", hex(DevEUI), hex(JoinEUI), DevNonce, my_sf, crc)
        pkg = pack("BBQQIBI", MY_ID, 0x01, DevEUI, JoinEUI, DevNonce, my_sf, crc)
        led.value(1)
        start = chrono.read_ms()
        lora.send(pkg)
        led.value(0)
        active_tx += chrono.read_ms()-start
        print("Join request sent!")
        oled_list.append("Join req. sent")
        oled_lines()
        start = chrono.read_ms()
        lora.setBlockingCallback(False, req_handler)
        # lora.recv()
        while(chrono.read_ms() - start < 8000):
            sleep(0.1)
            if(join_accept == 1):
                break
        lora.sleep()
        active_rx += chrono.read_ms()-start
        if (join_accept == 0):
            print("No answer received!")
            attempts += 1
            random_sleep(5*attempts)
            DevNonce += 0x00000001
        else:
            break

    # AppSKey generation
    text = pack("BIiI", 0x02, JoinNonce, 0xFFFFFF, DevNonce)
    while (len(text) % 16 != 0):
        text = b''.join([text, 0x00])
    cipher_en = aes(AppKey, 1)
    AppSKey = cipher_en.encrypt(text)
    print("Length of AppSKey:", len(AppSKey), hexlify(AppSKey).decode())
    # slot generation
    text = "".join([hex(DevAddr)[2:], hex(DevEUI)[2:]])
    thash = sha256()
    thash.update(text)
    thash = int(hexlify(thash.digest()), 16)
    my_slot = thash % S
    print("Slot =", my_slot, "DevAddr =", hex(DevAddr))
    oled_list.append("Slot="+str(my_slot))
    oled_lines()
    print("joining the network lasted (ms):", chrono.read_ms()-join_start)
    sync()

def sync_handler(event):
    global lora
    global index
    global active_tx
    global active_rx
    global chrono
    global proc_gw
    global sack_rcv
    global sack_bytes
    if event & SX1262.RX_DONE:
        recv_pkg, err = lora.recv()
        sack_rcv = chrono.read_us()
        # print(recv_pkg)
        if (len(recv_pkg) > 2):
            ack_len = recv_pkg[1]
            recv_pkg_id = recv_pkg[0]
            if (int(recv_pkg_id) == (my_sf-5)):
                try:
                    (id, leng, index, proc_gw, acks, rcrc) = unpack("BBBB%dsI" % ack_len, recv_pkg)
                except:
                    print("Couldn't unpack")
                else:
                    crc = b''.join([int(index).to_bytes(1, 'big'), int(proc_gw).to_bytes(1, 'big'), acks])
                    if (crc32(crc) == rcrc):
                        sack_bytes = 4 + ack_len + 4
                        print("ACK!")
                        oled_list.append("Synchronized!")
                        oled_lines()
                    else:
                        print("Sync CRC failed")

def sync():
    global lora
    global index
    global active_tx
    global active_rx
    global chrono
    global sack_rcv
    global oled_list
    lora.setSpreadingFactor(my_sf)
    lora.setFrequency(freqs[my_sf-7])
    # lora.set_implicit(False) # switch back to explicit mode without CRC
    # lora.set_preamble_length(8)
    sync_start = chrono.read_us()
    sack_rcv = 0
    index = 0
    oled_list.append("Waiting for SACK")
    oled_lines()
    print("Waiting for SACK...")
    lora.setBlockingCallback(False, sync_handler)
    # lora.recv()
    while (index == 0):
        if (index > 0):
            active_rx += (chrono.read_us() - sync_start)
    print("sync slot lasted (ms):", (chrono.read_us()-sync_start)/1000)
    print("active time during join-sync rx/tx (ms):", active_rx/1000, active_tx/1000)

def generate_msg():
    global msg
    msg = ""
    while (len(msg) < packet_size):
        r = random.getrandbits(32) # just a random 4-byte int
        msg += hex(r)[2:]
    while (len(msg) > packet_size): # just correct the size
        msg = msg[:-1]

def sack_handler(event):
    global index
    global chrono
    global proc_gw
    global sack_rcv
    global clock_correct
    global sync_start
    global acks
    global sack
    global oled_list
    global corrections
    global ci
    global sack_bytes
    if event & SX1262.RX_DONE:
        recv_pkg, err = lora.recv()
        sack_rcv = chrono.read_us()
        # print(recv_pkg[0], recv_pkg[2])
        if (len(recv_pkg) > 2):
            ack_len = recv_pkg[1]
            recv_pkg_id = recv_pkg[0]
            if (int(recv_pkg_id) == (my_sf-5)):
                rssi = lora.getRSSI()
                oled_list.append("SACK rcved ("+str(rssi)+")")
                oled_lines()
                sack_bytes = 4 + ack_len + 4
                airt = airtime_calc(my_sf,1,sack_bytes,my_bw_plain)
                wt = sack_rcv-sync_start-airt*1000
                # print("Waiting time before receiving SACK (ms):", wt/1000)
                print("SACK received with RSS:", rssi, "dBm (waiting time:", wt/1000, "ms)")
                clock_correct = wt - ci
                corrections.append(clock_correct)
                if (len(corrections) > 10):
                    corrections.pop(0)
                clock_correct = sum(corrections) / len(corrections)
                try:
                    (id, leng, index, proc_gw, acks, rcrc) = unpack("BBBB%dsI" % ack_len, recv_pkg)
                    crc = b''.join([int(index).to_bytes(1, 'big'), int(proc_gw).to_bytes(1, 'big'), acks])
                    if (crc32(crc) == rcrc):
                        acks = acks.decode()
                        sack = 1
                    else:
                        print("...SACK CRC failed")
                except:
                    print("...Could not unpack!")

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
    global sack_rcv
    global msg
    global sync_start
    global acks
    global sack
    global clock_correct
    global oled_list
    global corrections
    global ci
    global sack_bytes
    # airtime is calculated including CRC even though we don't use it. This will result in some more empty space in the slot
    airt = int(airtime_calc(my_sf,1,packet_size+6,my_bw_plain)*1000)
    slot = airt + 2*guard
    duty_cycle_limit_slots = ceil(100*airt/slot)
    proc_and_switch = 7000 # time for preparing the packet and switch radio mode (us)
    ci = 31000 # confidence interval for clock correction (needs callibration!)
    chrono.reset()
    if (my_slot == -1):
        join_start = chrono.read_us()
        join_request()
    else:
        sync()
    repeats = 0
    clock_correct = 0
    sync_slot = int(airtime_calc(my_sf,1,sack_bytes,my_bw_plain)*1000) + guard
    corrections = []
    misses = 0
    print("-----")
    print("MY SLOT:", my_slot)
    print("Net size:", index)
    print("Time on air (ms):", airt/1000)
    print("Guard time (ms):", guard/1000)
    print("Slot size (us):", slot)
    print("Duty cycle slots:", duty_cycle_limit_slots)
    print("SACK slot length (ms):", sync_slot/1000)
    print("Gw processing time (ms):", proc_gw)
    fpas_time = (chrono.read_us()-sack_rcv)/1000
    print("Time after SACK rec (ms):", fpas_time)
    i = 0x00000001
    (succeeded, retrans, dropped, active_rx, active_tx) = (0x00000000, 0x00000000, 0x00000000, 0.0, 0.0)
    print("S T A R T")
    print("Aligning round length...", end='')
    while (chrono.read_us() - sack_rcv) < 100000:
        print(".", end='')
        pass
    print("\n")
    while(_pkts == -1 or i <= _pkts):
        print(i, "----------------------------------------------------")
        chrono.reset()
        start = chrono.read_us()
        # calculate the time until the sack packet
        if (int(index) > duty_cycle_limit_slots):
            data_length = ceil(index*slot)
        else:
            data_length = ceil(duty_cycle_limit_slots*slot)
        print("All data slots length (ms):", data_length/1000)
        t = int(my_slot*(airt + 2*guard) + guard - proc_and_switch) # sleep time before transmission
        sleep_us(t)
        en_msg = aes(AppSKey, 1).decrypt(msg)
        crc = crc32(en_msg)
        on_time = chrono.read_us()
        pkg = pack('BB%dsI' % len(msg), MY_ID, len(msg), en_msg, crc)
        print("Sending a packet of", len(b''.join([int(MY_ID).to_bytes(1, 'big'), int(len(en_msg)).to_bytes(1, 'big'), en_msg, crc.to_bytes(4, 'big')])), "bytes at (ms):", (chrono.read_us()-start)/1000)
        lora.send(pkg)
        # lora.sleep()
        print("Sent:", msg)
        oled_list.append("Uplink sent! s"+str(my_slot))
        oled_lines()
        active_tx += (chrono.read_us() - on_time)
        t = int(data_length - (chrono.read_us() - start) + proc_gw + clock_correct - 2*ci - misses*guard) # wake up 2*ci before the sack transmission
        if t < 0:
            print("Cannot align clock!")
            t = 0
        print("Sleep time after data (ms):", t/1000, "/ round correction (ms):", clock_correct/1000)
        _thread.start_new_thread(generate_msg, ()) # this will take some time to be executed
        sleep_us(t)
        sack_rcv = 0
        clock_correct = 0
        acks = ""
        sack = 0
        print("Started sync slot at (ms):", chrono.read_ms())
        # oled_list.append("Waiting for SACK")
        oled_lines()
        led.value(1)
        lora.setBlockingCallback(False, sack_handler)
        # lora.recv()
        sync_start = chrono.read_us()
        while(chrono.read_us() - sync_start < ci+sync_slot+misses*guard):
            if (sack == 1):
                break
        led.value(0)
        lora.sleep()
        active_rx += (chrono.read_us() - sync_start)
        if (sack == 1): # if a SACK has been received
            print("...ACK data =", acks)
            bin_ack = ""
            bin_ack = bin(int(acks, 16))[2:]
            while len(bin_ack) < index:
                bin_ack = "0"+bin_ack
            if len(bin_ack) > index:
                bin_ack = bin_ack[-index:]
            print("...binary:", bin_ack)
            if (bin_ack[my_slot] == "1"): # if the uplink has been delivered
                succeeded += 0x00000001
                repeats = 0
                print("...ACK OK!")
                oled_list.append("OK ("+str(i)+"/"+str(succeeded)+"/"+str(retrans)+")")
                oled_list.append("Trx:"+str(active_rx/1e6))
                oled_list.append("Ttx:"+str(active_tx/1e6))
                oled_lines()
            else:
                print("I will repeat the last packet")
                oled_list.append("NOT OK")
                oled_lines()
                retrans += 0x00000001
                repeats += 0x00000001
                i -= 0x00000001
                if (repeats == 4):
                    print("Packet dropped!")
                    repeats = 0
                    dropped += 0x00000001
                    retrans -= 0x00000001
                    i += 0x00000001
            if (misses > 0):
                corrections = []
            misses = 0
            print("time after SACK (ms):", (chrono.read_us()-sack_rcv)/1000)
        else:
            print("SACK missed!")
            oled_list.append("SACK missed")
            oled_lines()
            retrans += 0x00000001
            repeats += 0x00000001
            i -= 0x00000001
            misses += 1
            if (repeats == 4):
                print("Packet dropped!")
                print("Synchronisation lost!")
                repeats = 0
                dropped += 0x00000001
                retrans -= 0x00000001
                i += 0x00000001
                oled_list.append("Packet dropped")
                oled_lines()
                sleep_us(int(data_length-sync_slot-3*ci))
                sync()
            print("Aligning round length...", end='')
            while (chrono.read_us()-start) < data_length+sync_slot+proc_gw+105000:
                print(".", end='')
                pass
        print("round lasted (ms):", (chrono.read_us()-start)/1000)
        print("transmitted/delivered/retransmitted/dropped:", i, succeeded, retrans, dropped)
        print("radio active time (rx/tx) (s):", active_rx/1e6, "/", active_tx/1e6)
        i += 0x00000001

    # send out stats
    print("I'm sending stats")
    # print("Sending", MY_ID, i-1, "2", succeeded, retrans, dropped, str(active_rx/1e6), str(active_tx/1e6))
    leng = len(bytes(MY_ID+(i-1)+(0x02)+succeeded+retrans+dropped))+8
    pkg = pack("BBBiiiiff", MY_ID, leng, 0x02, int(i-1), int(succeeded), int(retrans),
                        int(dropped), active_rx/1e6, active_tx/1e6) # 27 bytes
    lora.setSpreadingFactor(j_sf)
    lora.setFrequency(freqs[-1])
    for x in range(3): # send it out 3 times
        led.value(1)
        lora.send(pkg)
        lora.sleep()
        random_sleep(10)
    led.value(0)

def init_handler(recv_pkg):
    global lora
    global chrono
    if (len(recv_pkg) > 2):
        recv_pkg_id = recv_pkg[0]
        if (int(recv_pkg_id) == 1):
            dev_id, pkts = unpack('Bi', recv_pkg)
            print("Starting experiment with", pkts, "packets")
            oled_list.append("New experiment")
            oled_lines()
            if (my_slot == -1):
                random_sleep(20)
            led.value(1)
            lora.sleep()
            chrono.start()
            join_start = chrono.read_us()
            start_transmissions(pkts)
            print("...experiment done!")
            # lora.set_crc(True)
            lora.setSpreadingFactor(j_sf)
            lora.setFrequency(freqs[-1])
            lora.setBlockingCallback(False, init_handler)
            # lora.recv()
            print("ready for a new one...")

def oled_lines():
    global oled_list
    oled.fill(0)
    oled.text("TS-LoRa "+"SF"+str(my_sf)+" ID"+str(MY_ID), 0, 0)
    l = 11
    while len(oled_list) > 6:
        oled_list.pop(0)
    for line in oled_list:
        oled.text(line, 0, l)
        l += 8
    oled.show()
    gc.collect()

### MAIN ###

led = Pin(35, Pin.OUT) # v3
rst = Pin(21, Pin.OUT)
rst.value(1)
scl = Pin(18, Pin.OUT, Pin.PULL_UP)
sda = Pin(17, Pin.OUT, Pin.PULL_UP)
i2c = SoftI2C(scl=scl, sda=sda, freq=450000)
oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3c)
oled.poweron()
oled_list = []

# SPI pins
SCK  = 9
MOSI = 10
MISO = 11
CS   = 8
RX   = 14
RST  = 12
BUSY = 13

lora = SX1262(1, SCK, MOSI, MISO, CS, RX, RST, BUSY)


MY_ID = 0x0B # to be filled in get_id (1 byte, so up to 256 devices)
my_mac = " "
DevAddr = ""
get_id()

(my_sf, j_sf, my_bw_plain, guard, my_slot, packet_size) = (0x07, 10, 125, 15000, -1, 16) # default values
# lora.set_preamble_length(8)
# lora.set_crc(False)
# lora.set_implicit(False)
index = 0
S = 251
active_rx = 0.0
active_tx = 0.0
proc_gw = 4000 # gw default (minimum) processing time (us)
(sack_rcv, sack_bytes) = (0, 0) # will be filled later
generate_msg()
print("Packet size =", len(msg), "bytes")
(retrans, succeeded, dropped) = (0, 0, 0)
freqs = [868.1, 868.3, 868.5, 867.1, 867.3, 867.5, 867.7, 867.9, 869.525]
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
chrono.start()
pkts = -1
lora.begin(freq=freqs[0], bw=125.0, sf=7, cr=5)

def commands():
    global lora
    print("Waiting for commands...")
    oled_list.append("Waiting for commands")
    oled_lines()
    lora.setSpreadingFactor(j_sf)
    lora.setFrequency(freqs[-1])
    lora.setBlockingCallback(False, init_handler)
    # lora.recv()

# uncomment the following line if you don't want to use the init_exp.py script (additional changes on gw_req are required)
start_transmissions(pkts)
#
# commands()
