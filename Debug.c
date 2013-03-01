#include "lm3s8962.h"

#define DUMPSIZE 100
unsigned long Debug_time[DUMPSIZE];
unsigned char Debug_place[DUMPSIZE];
unsigned long n=0;
void Debug_Profile(unsigned char p){
if(n<DUMPSIZE){
Debug_time[n] = NVIC_ST_CURRENT_R; //record current time
Debug_place[n] = p;
n++;
}
}
