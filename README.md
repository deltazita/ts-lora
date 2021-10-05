# TS-LoRa 
Time-Slotted LoRa(WAN) for the Industrial Internet of Things
Corresponding papers: https://doi.org/10.1016/j.comcom.2020.01.056, https://cora.ucc.ie/handle/10468/9723

This is a stand-alone implementation of the TS-LoRa protocol as it is presented in the paper "TS-LoRa: Time-Slotted LoRaWAN for the Industrial Internet of Things". The implementation consists of 4 parts; the node part, the gateway request part, the gateway data part, and the Raspberry Pi part. The code has been designed for and tested on Pycom Lopy4 and Fipy devices with SF7-9. To run this code you will need at least 3 Pycom devices with LoRa and Wifi support (2 for the gateways + 1 node). Modifications may be needed to adjust the scenarios to your own needs. All the code except of the Raspberry Pi TCP server script is strictly licensed under the GNU GPL v3 (see LICENSE file).
The implementation constitutes a proof-of-concept and it can be used by researchers to duplicate the experimental setup used in the papers above.
You can modify the implementation according to your needs (see SETUP file)

Features:
- Time-slotted LoRa transmissions
- Inter and intra-SF collision-free transmissions
- Synchronisation and ACKnowledgements (SACK)
- Retransmissions
- 1% radio duty cycle
- Multiple SFs/BWs using multiple paraller frames
- A simple registration mechanism (similar to LoRaWAN OTAA) including the generation of AppSKeys
- Statistics per node
- Over 99.9% Packet Delivery Ratio (excluding path-loss losses)

Limitations / Work in progress:
- Two or more 1-channel gateways must currently be used. 
- No data encryption is currently performed but the keys are calculated. 
- Python's pack and unpack struct functions are currently used to send data. 
- Downlink transmissions are not encrypted

For any serious inquires please contact the author at dimzorbas@ieee.org
