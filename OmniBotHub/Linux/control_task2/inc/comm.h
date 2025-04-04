 #ifndef __COMM_H_
 #define __COMM_H_
 #include<stdio.h>
 #include<sys/types.h>
 #include<sys/ipc.h>
 #include<sys/shm.h>
 #define PATHNAME "."
 #define PROJ_ID 0X6666
 
 int createshm(int sz);//创建
 int destroyshm(int shmid);//销毁
 int getshm(int sz);//获得
 #endif// __COMM_H_  