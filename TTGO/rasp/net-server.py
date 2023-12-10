#!/usr/bin/python3

import socket
import threading
import random
import hashlib
import binascii
import os
from Crypto.Cipher import AES
import struct
from datetime import datetime

S = 251
bind_ip = '192.168.0.254'
bind_port = 8000

AK = ["3878214125442A472D4B615064536756","7234753778217A25432A462D4A614E64","576D5A7134743777217A24432646294A","655368566D5971337436773979244226",
"4B6150645367566B5970337336763979","2A462D4A614E645267556B5870327335","7A24432646294A404E635266556A586E","36763979244226452948404D63516654",
"703373367638792F423F4528482B4D62","556B58703273357638782F413F442847","635266556A586E327235753878214125","48404D635166546A576E5A7234753777",
"3F4528482B4D6251655468576D5A7134","782F413F4428472B4B6250655368566D","35753778214125442A472D4B61506453","6E5A7234753777217A25432A462D4A61",
"5468576D5A7134743677397A24432646","6250655368566D597133743676397924","472D4B6150645367566B597033733676","25432A462D4A614E645267556B587032",
"77397A24432646294A404E635266556A","337336763979244226452948404D6351","6B59703373357638792F423F4528482B","5267556B58703273357538782F413F44",
"404E635266556A586E32723475377821","452948404D635166546A576E5A723474","2F423F4528482B4D6251655468576D5A","7538782F413F4428472B4B6250655368",
"5A7234753778214125442A472D4B6150","6A576E5A7134743777217A25432A462D","51655468576D5A7133743677397A2443","2B4B6250655368566D59713373367639",
"442A472D4B6150645367566B59703373","217A25432A462D4A614E645267556B58","743677397A24432646294A404E635266","5970337336763979244226452948404D"]

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((bind_ip, bind_port))
server.listen(5)  # max backlog of connections

f = open("server.log", "a")
print ("Listening on:", bind_ip, bind_port)
f.write(str(datetime.now())+" Listening on: "+str(bind_ip)+" "+str(bind_port)+"\n")
f.flush()

def handle_client_connection(client_socket):
    request = client_socket.recv(1024)
    try:
        (id, index, mac, JoinNonce, JoinEUI, DevNonce, rcrc) = struct.unpack('BBQIQII', request)
    except:
        print ("Could not unpack")
    else:
        rmsg = b''.join([int(id).to_bytes(1, 'big'), int(index).to_bytes(1, 'big'), mac.to_bytes(8, 'big'), JoinNonce.to_bytes(4, 'big'), JoinEUI.to_bytes(8, 'big'), DevNonce.to_bytes(4, 'big')])
        crc = binascii.crc32(rmsg)
        print (datetime.now(), "-- Received:", id, index, hex(mac), JoinNonce, JoinEUI, DevNonce, hex(rcrc), hex(crc))
        f.write(str(datetime.now())+" -- Received request from "+str(id)+"\n")
        f.flush()
        if (crc == rcrc):
            # compute a DevAddr
            DevAddr = hex(random.getrandbits(32))[2:][:-1]
            text = "".join([DevAddr, hex(mac)[2:]])
            while (len(text) < 8):
                text = "".join([text,"0"])
            thash = hashlib.sha256()
            thash.update(text.encode('utf-8'))
            thash = thash.digest()
            slot = (int(binascii.hexlify(thash), 16)) % S
            while(slot != int(index)):
                DevAddr = hex(random.getrandbits(32))[2:][:-1]
                text = "".join([DevAddr, hex(mac)[2:]])
                while (len(text) < 8):
                    text = "".join([text,"0"])
                thash = hashlib.sha256()
                thash.update(text.encode('utf-8'))
                thash = thash.digest()
                slot = (int(binascii.hexlify(thash), 16)) % S
            # compute the AppSKey
            AppKey = AK[int(id)-11]
            text = struct.pack("BIiI", 0x02, JoinNonce, 0xFFFFFF, DevNonce)
            while (len(text) % 16 != 0):
                text = b''.join([text, 0x00])
            encryptor = AES.new(AppKey.encode("utf-8"), AES.MODE_ECB)
            AppSKey = encryptor.encrypt(text)
            msg = struct.pack("BBI%ds" % len(AppSKey), int(id), len(AppSKey), int(DevAddr,16), AppSKey)
            client_socket.send( msg )
            print (datetime.now(), "-- Responded: "+str(id)+" "+str(DevAddr)+" "+binascii.hexlify(AppSKey).decode())
            f.write(str(datetime.now())+" -- Responded: "+str(id)+" "+str(DevAddr)+" "+binascii.hexlify(AppSKey).decode()+"\n")
            f.flush()
            client_socket.close()
        else:
            print("CRC check failed")

while True:
    client_sock, address = server.accept()
    print ("Accepted connection from:", address[0], address[1])
    client_handler = threading.Thread(
        target=handle_client_connection,
        args=(client_sock,)
    )
    client_handler.start()

f.close()
