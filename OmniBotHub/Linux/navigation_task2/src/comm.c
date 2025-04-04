#include "comm.h"
 static int commshm(int sz,int flag)
 {
     key_t key = ftok(PATHNAME,PROJ_ID);
     if(key < 0){
          perror("ftok");
         return -1;
     }
     int shmid=0;
     if((shmid=shmget(key,sz,flag)) < 0){
         perror("shmget");
         return -2;
     }
     return shmid;
 }
 int destroyshm(int shmid)
 {
     if(shmctl(shmid,IPC_RMID,NULL)<0){
         perror("shmctl");
         return -1;
     }
     return 0;                                                                                                             
 }
 int createshm(int sz)
 {
     return commshm(sz,IPC_CREAT|IPC_EXCL|0666);
 }
 int getshm(int sz)
 {
     return commshm(sz,IPC_CREAT);
 }