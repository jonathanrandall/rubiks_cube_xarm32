from machine import Pin, I2C, Timer
import time, gc
from micropython import const

###########

from Oled import OLED_I2C
import utime
oled_flag = True
i2c = I2C(0, scl=Pin(27), sda=Pin(14), freq=100000)
try:
  oled = OLED_I2C(128, 64, i2c)
except OSError:
  print('没有检测到OLED模块')
  oled_flag = False

###################


try:
  import usocket as socket
except:
  import socket
  
from time import sleep

from machine import Pin, I2C
import network

from wifi_stuff import ssid, password
from wifi_stuff import html

import gc
gc.collect()

# ssid = "ssid"
# password = "password"
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)
while not wlan.isconnected():
    pass

ip_configuration = wlan.ifconfig()[0]
# ip_configuration = ip_configuration.decode('utf-8')
print("ip address ", ip_configuration)

oled.fill(0)
oled.text("RED", 10, 11)
oled.text(ip_configuration, 10,32)
oled.show()
# 

########################################

from BusServo import BusServo

bus_servo = BusServo(tx=26, rx=35, tx_en=25, rx_en=12)

# Setup the web server
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)

print('listening on', addr)

def refresh_sevos():
   for i in range(6):
      bus_servo.run(i+1,500)

while True:
    cl, addr = s.accept()
    print('client connected from', addr)
    request = cl.recv(1024)
    request = str(request)
    print('request:', request)
    
    if 'GET / ' in request:
        response = html
        refresh_sevos()
    elif 'GET /slider?value=' in request:
        info = (request.split('value=')[1].split(' ')[0])
        servo_id = info.split('_')[0]
        servo_id = int(servo_id)
        angle = info.split('_')[1]
        angle = int(angle)

        bus_servo.run(servo_id,angle)
        # servo.write_angle(angle)
        response = "Servo ID set to " + str(servo_id) + " angle is " + str(angle)
        print("repsonse", response)
    else:
        response = "Invalid Request"
    
    cl.send('HTTP/1.1 200 OK\n')
    cl.send('Content-Type: text/html\n')
    cl.send('Connection: close\n\n')
    cl.sendall(response)
    cl.close()

