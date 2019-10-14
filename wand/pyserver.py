import socket
import struct
import time

UDP_IP = "192.168.0.255"
UDP_PORT = 50050

descriptions = [
  ('BUCKET_WEIGHT', 'f', 0.01),
  ('DAMPNING', 'f', .95),
  ('INERTIA', 'f', .10),
  ('SCALE', 'f', 20),
  ('GAIN', 'f', .008),
  ('MIN_HUE', 'f', 140.0),
  ('MAX_HUE', 'f', 0.0),
  ('SCINT_AMP', 'f', 10.0),
  ('SCINT_T_FREQ', 'f', .008),
  ('SCINT_S_FREQ', 'f', 1.2),
  ('MAX_DISTURBANCE', 'd', 600.0),
  ('MIN_BRIGHTNESS', 'f', 50.0),
  ('MAX_BRIGHTNESS', 'f', 255.0),
  ('MIN_DISRUPTION', 'f', 400.0),
  ('DISRUPTION_DELAY', 'I', 5000),
  ('DISRUPT_VELOCITY', 'f', 0.006),
  ('DISRUPT_AMP', 'f', 1.3),
  ('DISRUPT_DAMPING', 'f', 0.4),
  ('DISRUPT_FREQ', 'f', 2.0),
  ('DISRUPT_DONE', 'I', 10000)
]

table = [
  ("4550e4", 4, 6.92),
  ("041ec0", 3, 5.20),
  ("059e54", 5, 5.20),
  ("44ddda", 2, 3.46),
  ("455aef", 4, 3.46),
  ("0599c3", 6, 3.46),
  ("44e507", 1, 1.73),
  ("059e92", 3, 1.73),
  ("32c9bb", 5, 1.73),
  ("454eda", 7, 1.73),
  ("4557cd", 0, 0),
  ("4550c4", 2, 0),
  ("455004", 4, 0),
  ("4551d7", 6, 0),
  ("455243", 8, 0)
]
table = [(int(x[0],16),) + x[1:] for x in table]
print(table[0][0])

while True:
  sock = socket.socket(socket.AF_INET, # Internet
                       socket.SOCK_DGRAM) # UDP
  params = [item for pair in table for item in pair] + [desc[2] for desc in descriptions]
  MESSAGE = struct.pack(('Iff'*len(table)) + ''.join([desc[1] for desc in descriptions]), *params)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
  time.sleep(3)
