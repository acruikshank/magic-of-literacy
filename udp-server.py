import socket
 
UDP_IP = "10.0.1.8"
UDP_PORT = 4210
 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
 
while True:
  data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
  print("received message:", data)
