#!/usr/bin/env python3
'''=========== ***doc description @ yyp*** ===========
xbox手柄
pip install pygame
======================================================'''
import socket
import pygame
import struct
from threading import Thread
import time
import sys

ip='192.168.54.110'
# ip='0.0.0.0'
port=8001
sk=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
# struct cmdStruct{
# 	int head=0x63;
# 	int size;//32
# 	int id;
# 	int key;
# 	float joy[4];
# };
cmd=bytearray([63,0,0,0,   20,0,0,0,   1,0,0,0,
				0,0,0,0,
				0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0])

vx=0;vy=0;wz=0
def thresh(input):
    if(abs(input)<0.02):
        input=0
    return input
def clearV():
    global vx,vy,wz
    vx=0;vy=0;wz=0
    updateVcmd()
def updateVcmd():
    cmd[16:20]=struct.pack('f',vx)
    cmd[20:24]=struct.pack('f',vy)
    cmd[24:28]=struct.pack('f',wz)
# def skLoop():
#     while(1):
#         sk.sendto(cmd,(ip,port))
#         time.sleep(0.5)

# # root.bind('<FocusIn>',focus)
# th=Thread(target=skLoop)
# th.daemon=1
# th.start()


pygame.init()
# pygame.joystick.init()
clock=pygame.time.Clock()
js=pygame.joystick.Joystick(0)
js.init()
numBut=js.get_numbuttons()

while 1:
    pygame.event.get()
    vx=thresh(-js.get_axis(1))
    vy=thresh(-js.get_axis(0))
    wz=thresh(-js.get_axis(3))
    
    for i in range(numBut):
        if(js.get_button(i)):
            print(i)
            cmd[12]=i
    
    clock.tick(5)