#!/usr/bin/env python3
'''=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

======================================================'''
import socket
import tkinter as tk
import struct
from threading import Thread
import time
import sys

ip='192.168.1.246'
# ip='0.0.0.0'
port=8001

import subprocess

def switch_input_method(input_method):
    subprocess.run(['fcitx-remote', '-s', input_method], shell=False)

# 切换到英文键盘布局
switch_input_method('us(english)')


sk=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

cmd=bytearray([63,0,0,0,   20,0,0,0,   1,0,0,0,
				0,0,0,0,
				0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0])

def skLoop():
    while(1):
        sk.sendto(cmd,(ip,port))
        time.sleep(0.5)

vx=0;vy=0;wz=0
def clearV():
    global vx,vy,wz
    vx=0;vy=0;wz=0
    updateVcmd()
def updateVcmd():
    cmd[16:20]=struct.pack('f',vx)
    cmd[20:24]=struct.pack('f',vy)
    cmd[24:28]=struct.pack('f',wz)
    
def butDis():
    clearV()
    cmd[12]=13
    sk.sendto(cmd,(ip,port))
def butClearErr():
    clearV()
    tmp=cmd[12]
    cmd[12]=29
    sk.sendto(cmd,(ip,port))
    cmd[12]=tmp
def butEn():
    clearV()
    cmd[12]=1
def butRc():
    clearV()
    cmd[12]=2
def butIdle():
    clearV()
    cmd[12]=3
def butWk():
    clearV()
    cmd[12]=4
def butOp():
    clearV()
    cmd[12]=16
def butRl():
    clearV()
    cmd[12]=21
def butDamp():
    clearV()
    cmd[12]=20

def butStart():
    clearV()
    cmd[12]=15
def butStop():
    clearV()
    cmd[12]=17

def keyPress(ev):
    global vx,vy,wz
    key=ev.keysym
    print(key)
    if(key=='KP_7'):
        butEn()
    elif(key=='KP_8'):
        butRc()
    elif(key=='KP_9'):
        butIdle()
    elif(key=='KP_4'):
        butWk()
    elif(key=='KP_5'):
        butStart()
    elif(key=='KP_6'):
        butStop()
    elif(key=='KP_1'):
        butOp()
    elif(key=='KP_2'):
        butRl()
    elif(key=='KP_3'):
        butDamp()
    elif(key=='KP_0'):
        butDis()
    elif(key=='w'):
        vx+=0.1
        updateVcmd()
    elif(key=='s'):
        vx-=0.1
        updateVcmd()
    elif(key=='a'):
        vy+=0.1
        updateVcmd()
    elif(key=='d'):
        vy-=0.1
        updateVcmd()
    elif(key=='j'):
        wz+=0.1
        updateVcmd()
    elif(key=='l'):
        wz-=0.1
        updateVcmd()
    elif(key=='space'):
        clearV()

root=tk.Tk()
root.configure(background='#ccc')
if sys.platform.startswith('win'):
    root.geometry('800x400')
    length=100
else:
    if(root.winfo_screenwidth()>5000):
        root.geometry('1200x600+4600+1000')
        length=120
    else:
        root.geometry('1000x500+2600+200')
        length=100
tk.Button(root,text='清错',command=butClearErr).place(anchor='center',relx=0.1,rely=0.1,width=length,height=length)

tk.Button(root,text='Y\nen',command=butEn).place(anchor='center',relx=0.7,rely=0.3,width=length,height=length)
tk.Button(root,text='X\nrc',command=butRc).place(anchor='center',relx=0.6,rely=0.5,width=length,height=length)
tk.Button(root,text='A\nidle',command=butIdle).place(anchor='center',relx=0.7,rely=0.7,width=length,height=length)
tk.Button(root,text='B\nwk',command=butWk).place(anchor='center',relx=0.8,rely=0.5,width=length,height=length)

tk.Button(root,text='↑\nstart',command=butStart).place(anchor='center',relx=0.3,rely=0.3,width=length,height=length)
tk.Button(root,text='←\nstop',command=butStop).place(anchor='center',relx=0.2,rely=0.5,width=length,height=length)
tk.Button(root,text='↓\ndis',command=butDis).place(anchor='center',relx=0.3,rely=0.7,width=length,height=length)
tk.Button(root,text='→\nop',command=butOp).place(anchor='center',relx=0.4,rely=0.5,width=length,height=length)
root.bind('<Key>',keyPress)
# root.bind('<FocusIn>',focus)
th=Thread(target=skLoop)
th.daemon=1
th.start()
tk.mainloop()


# struct cmdStruct{
# 	int head=99;
# 	int size;//32
# 	int id;
# 	int key;
# 	float joy[4];
# };
