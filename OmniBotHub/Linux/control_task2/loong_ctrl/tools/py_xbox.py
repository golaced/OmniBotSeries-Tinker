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

ip='192.168.1.246'
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


    

def handle_button_press(button_id):
    if button_id == 3:  # 按钮Y，键值3 收腿站立Rc
        clearV()
        cmd[12]=2
        print(f"收腿")
    elif button_id == 2:  # 按钮X，键值2 学习网络Rl
        clearV()
        cmd[12]=21
        print(f"学习步态")
    elif button_id == 0:  # 按钮A，键值0 阻尼Damp
        clearV()
        cmd[12]=20
        print(f"阻尼")
    elif button_id == 1:  # 按钮B，键值1 停止站立Idle
        clearV()
        cmd[12]=3
        print(f"停止")

def skLoop():
    while True:
        sk.sendto(cmd,(ip,port))
        time.sleep(0.5)

pygame.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    sys.exit(1)

js=pygame.joystick.Joystick(0)
js.init()
clock=pygame.time.Clock()
numBut=js.get_numbuttons()

# # root.bind('<FocusIn>',focus)
th=Thread(target=skLoop)
th.daemon=True
th.start()

while True:
    # pygame.event.get()
    pygame.event.pump()
    vx=thresh(-js.get_axis(1))
    vy=thresh(-js.get_axis(0))
    if js.get_axis(2) > 0:
        wz = 1
    elif js.get_axis(5) > 0:
        wz = -1
    else:
        wz = 0
    print(f"vx = {vx:.2f}, vy = {vy:.2f}, wz = {wz:.2f}")

    for i in range(numBut):
        if(js.get_button(i)):
            handle_button_press(i)

    hat = js.get_hat(0)
    if hat==(0,1):
        clearV()
        cmd[12]=1
        print(f"上使能")
    elif hat==(0,-1):
        clearV()
        cmd[12]=13
        sk.sendto(cmd,(ip,port))
        print(f"下使能")

    clock.tick(5)