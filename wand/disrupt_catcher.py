import socket
import struct
from controller_table import table
from pythonosc import osc_message_builder
from pythonosc import udp_client

UDP_IP = "0.0.0.0"
UDP_PORT = 50060

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

sender = udp_client.SimpleUDPClient('127.0.0.1', 4559)

while True:
  print('listening')
  data, addr = sock.recvfrom(16) # buffer size is 1024 bytes
  print(len(data))
  if len(data) > 0:
    (level, mac, millis, distance) = struct.unpack('biif', data)
    for i, controller in enumerate(table):
      print("received disruption from %x" % (mac,))
      if controller[0] == mac:
        print("controller found", i)
        sender.send_message('/trigger/coral', [i]) 
        break
