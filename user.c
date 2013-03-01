//*****************************************************************************
// Real Time Operating System 

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to the Arm Cortex M3",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2011

   Programs 6.4 through 6.12, section 6.2

 Copyright 2011 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
 
#include "inc/hw_types.h"
#include "os.h"
#include "lm3s8962.h"
#include "ADC.h"
#include "uart2.h"
#include "PolledButton.h"
#include "rit128x96x4.h"
#include "driverlib/sysctl.h"
#include "Output.h"
#include "Debug.h"

#define TIMESLICE               TIME_1MS    // thread switch time in system time units
#define MAXBUFFERSIZE           64          // maximum number of samples
#define ADC_CHANNEL             1
#define ADC_SAMPLING_FS         2000        
#define CLOCKFREQ               50000000    // PLL clock frequency
#define GPIO_PORTF0     (*((volatile unsigned long *)0x40025004))
#define GPIO_PORTF2     (*((volatile unsigned long *)0x40025010))
#define GPIO_PORTF3     (*((volatile unsigned long *)0x40025020))
#define SAMPLEFREQ              8000        // sampling frequency (min. 763 Hz)
#define GPIO_PORTC5  (*((volatile unsigned long *)0x40006080))
#define GPIO_PORTC7  (*((volatile unsigned long *)0x40006200))
#define GPIO_PORTC2  (*((volatile unsigned long *)0x40006010))
#define GPIO_PORTC3  (*((volatile unsigned long *)0x40006220))
#define GPIO_PORTA7  (*((volatile unsigned long *)0x40004200))
#define GPIO_PORTG0  (*((volatile unsigned long *)0x40026004))
#define GPIO_PORTB7  (*((volatile unsigned long *)0x40005200))

unsigned long NumCreated;   // number of foreground threads created
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every sample
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
long MaxJitter;             // largest time jitter between interrupts in usec
long MinJitter;             // smallest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};

short Debug=1;


#define PERIOD TIME_1MS/2     // 2kHz sampling period in system time units
// 10-sec finite time experiment duration 
#define RUNLENGTH 10000   // display results and quit when NumSamples==RUNLENGTH
long x[64],y[64];         // input and output arrays for FFT
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);



unsigned long Count1;   // number of times thread1 loops
unsigned long Count2;   // number of times thread2 loops
unsigned long Count3;   // number of times thread3 loops
unsigned long Count4;   // number of times thread4 loops
unsigned long Count5;   // number of times thread5 loops

int testcount = 0;
int testcount2 =0;
/*
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD1             (*((volatile unsigned long *)0x40007008))
#define GPIO_PORTD2             (*((volatile unsigned long *)0x40007010))
#define GPIO_PORTD3             (*((volatile unsigned long *)0x40007020))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOD      0x00000008  // port D Clock Gating Control

*/


//------------------Task 1--------------------------------
// 2 kHz sampling ADC channel 1, using software start trigger
// background thread executed at 2 kHz
// 60-Hz notch IIR filter, assuming fs=2000 Hz
// y(n) = (256x(n) -503x(n-1) + 256x(n-2) + 498y(n-1)-251y(n-2))/256
short Filter(short data){

static short x[6]; // this MACQ needs twice
static short y[6];
static unsigned int n=3;   // 3, 4, or 5
if(Debug==1){
Debug_Profile('f');
}		
  n++;
  if(n==6) n=3;     
  x[n] = x[n-3] = data;  // two copies of new data
  y[n] = (256*(x[n]+x[n-2])-503*x[1]+498*y[1]-251*y[n-2]+128)/256;
  y[n-3] = y[n];         // two copies of filter outputs too
  return y[n];
} 
//******** DAS *************** 
// background thread, calculates 60Hz notch filter
// runs 2000 times/sec
// inputs:  none
// outputs: none
unsigned short DASoutput;

void DAS(void){ int index;
unsigned short input;  
unsigned static long LastTime;  // time at previous ADC sample
unsigned long thisTime;         // time at current ADC sample
long jitter;                    // time between measured and expected


if(Debug==1){
Debug_Profile('d');
}	
  if(NumSamples < RUNLENGTH){   // finite time run
  input = ADC_In(1);
    thisTime = OS_Time();       // current time, 20 ns
    DASoutput = Filter(input);
    FilterWork++;        // calculation finished
    if(FilterWork>1){    // ignore timing of first interrupt
      jitter = OS_TimeDifference(thisTime,LastTime)/50-PERIOD/50;  // in usec
      if(jitter > MaxJitter){
        MaxJitter = jitter;
      }
      if(jitter < MinJitter){
        MinJitter = jitter;
      }        // jitter should be 0
      index = jitter+JITTERSIZE/2;   // us units
      if(index<0)index = 0;
      if(index>=JitterSize) index = JITTERSIZE-1;
      JitterHistogram[index]++; 
    }
    LastTime = thisTime;
  }
	GPIO_PORTC3 = 0x00;
}
//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with select button
// one foreground task created with button push
// foreground treads run for 2 sec and die
// ***********ButtonWork*************
void ButtonWork(void){
unsigned long i;
//unsigned long myId = OS_Id(); 
	if(Debug==1){
Debug_Profile('W');
}	
  oLED_Message(1,0,"NumCreated =",NumCreated); 
  if(NumSamples < RUNLENGTH){   // finite time run
    for(i=0;i<20;i++){  // runs for 2 seconds
      OS_Sleep(50);     // set this to sleep for 0.1 sec
    }
  }
  oLED_Message(1,1,"PIDWork    =",PIDWork);
  oLED_Message(1,2,"DataLost   =",DataLost);
  oLED_Message(1,3,"Jitter(us) =",MaxJitter-MinJitter);
  OS_Kill();  // done
} 

//************ButtonPush*************
// Called when Select Button pushed
// Adds another foreground task
// background threads execute once and return
void ButtonPush(void){
	GPIO_PORTC7 = 0x80;
	if(Debug==1){
Debug_Profile('B');
}	
  if(OS_AddThread(&ButtonWork,100,4)){
    NumCreated++; 
  }
	GPIO_PORTC7 = 0x80;
}
//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------
// hardware timer-triggered ADC sampling at 1 kHz
// Producer runs as part of ADC ISR
// Producer uses fifo to transmit 1000 samples/sec to Consumer
// every 64 samples, Consumer calculates FFT
// every 64 ms, consumer sends data to Display via mailbox
// Display thread updates oLED with measurement

//******** Producer *************** 
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 1 kHz, started by your ADC_Collect
// The timer triggers the ADC, creating the 1 kHz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 10-bit sample 
// sends data to the consumer, runs periodically at 1 kHz
// inputs:  none
// outputs: none
void Producer(unsigned short data){  
	if(Debug==1){
Debug_Profile('p');
}	
  if(NumSamples < RUNLENGTH){   // finite time run
    NumSamples++;               // number of samples
    if(OS_Fifo_Put(data) == 0){ // send to consumer
      DataLost++;
		
    } 
  } 
}
void Display(void); 

//******** Consumer *************** 
// foreground thread, accepts data from producer
// calculates FFT, sends DC component to Display
// inputs:  none
// outputs: none
void Consumer(void){ 
unsigned long data,DCcomponent; // 10-bit raw ADC sample, 0 to 1023
unsigned long t;  // time in ms
	GPIO_PORTC2 = 0x04;
	if(Debug==1){
Debug_Profile('C');
}	
//unsigned long myId = OS_Id(); 
 ADC_Collect(0, 1000, &Producer); // start ADC sampling, channel 0, 1000 Hz
  NumCreated += OS_AddThread(&Display,128,0); 
  while(NumSamples < RUNLENGTH) { 
    for(t = 0; t < 64; t++){   // collect 64 ADC samples
      data = OS_Fifo_Get();    // get from producer
      x[t] = data;             // real part is 0 to 1023, imaginary part is 0
    }
    cr4_fft_64_stm32(y,x,64);  // complex FFT of last 64 ADC values
    DCcomponent = y[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
    OS_MailBox_Send(DCcomponent);
  }
	GPIO_PORTC2 = 0x04;
  OS_Kill();  // done
}
//******** Display *************** 
// foreground thread, accepts data from consumer
// displays calculated results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){ 
unsigned long data,voltage;
	GPIO_PORTA7= 0x80;
	if(Debug==1){
Debug_Profile('D');
}	
  oLED_Message(0,0,"Run length is",(RUNLENGTH)/1000);   // top half used for Display
OutCRLF();
  while(NumSamples <= RUNLENGTH) { 
    oLED_Message(0,1,"Time left is",(RUNLENGTH-NumSamples)/1000);   // top half used for Display
    data = OS_MailBox_Recv();
    voltage = 3000*data/1024;               // calibrate your device so voltage is in mV
    oLED_Message(0,2,"v(mV) =",voltage);  
  } 
		GPIO_PORTA7= 0x00;
  OS_Kill();  // done
} 

//--------------end of Task 3-----------------------------

//------------------Task 4--------------------------------
// foreground thread that runs without waiting or sleeping
// it executes a digital controller 
//******** PID *************** 
// foreground thread, runs a PID controller
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
short IntTerm;     // accumulated error, RPM-sec
short PrevError;   // previous error, RPM
short Coeff[3];    // PID coefficients
short PID_stm32(short Error, short *Coeff);
short Actuator;
void PID(void){ 
short err;  // speed error, range -100 to 100 RPM
//unsigned long myId = OS_Id(); 
	GPIO_PORTG0=0x01;
	if(Debug==1){
Debug_Profile('P');
}	
  PIDWork = 0;
  IntTerm = 0;
  PrevError = 0;
  Coeff[0] = 384;   // 1.5 = 384/256 proportional coefficient
  Coeff[1] = 128;   // 0.5 = 128/256 integral coefficient
  Coeff[2] = 64;    // 0.25 = 64/256 derivative coefficient*
  while(NumSamples < RUNLENGTH) { 
    for(err = -1000; err <= 1000; err++){    // made-up data
      Actuator = PID_stm32(err,Coeff)/256;
    }
    PIDWork++;        // calculation finished
  }
	GPIO_PORTG0=0x00;
  for(;;){ }          // done
}
//--------------end of Task 4-----------------------------

//------------------Task 5--------------------------------
// UART background ISR performs serial input/output
// two fifos are used to pass I/O data to foreground
// Lab 1 interpreter runs as a foreground thread
// the UART driver should call OS_Wait(&RxDataAvailable) when foreground tries to receive
// the UART ISR should call OS_Signal(&RxDataAvailable) when it receives data from Rx
// similarly, the transmit channel waits on a semaphore in the foreground
// and the UART ISR signals this semaphore (TxRoomLeft) when getting data from fifo
// it executes a digital controller 
// your intepreter from Lab 1, with additional commands to help debug 
// foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none
void Interpreter(void){

	//while(1);

unsigned char i;
  char string[19];  // global to assist in debugging

 
	long number;
	int line;
	int half;

	//unsigned short ADCbuffer[MAXBUFFERSIZE];
	extern volatile int counter;

	GPIO_PORTB7=0x080;
if(Debug==1){
Debug_Profile('I');
}	
  UART_Init();     	// initialize UART
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC; // activate port C
//	ADC_InitTimer0ATriggerSeq0(0, 0, (CLOCKFREQ/SAMPLEFREQ)-1);
	GPIO_PORTC_DIR_R |= 0x20;   // make PC5 out
  GPIO_PORTC_DEN_R |= 0x20;   // enable digital I/O on PC5
  OutCRLF();
	
  while(1){
	
	UART_OutString("Choose an option to test:\r\n");
	UART_OutString("[1] oLED_OutPut\r\n");
	UART_OutString("[2] Clear Screen\r\n");

	
	switch(UART_InUDec()){
		
		case 1:
		UART_OutString("\r\nInString: ");
    UART_InString(string,19);
		OutCRLF();
		UART_OutString("Top[Enter 0] or Bottom[Enter 1]of screen? ");
		half=UART_InUDec();
		OutCRLF();
		UART_OutString("Line 0,1,2,3? ");
		line=UART_InUDec();
		OutCRLF();
		UART_OutString("Number to display: ");
		number=UART_InUDec();
		OutCRLF();
		oLED_Message(half, line, string, number);
		
		for(i=0;i<19;i++){
		string[i]=0;
		}
		
		
		number=0;
		
		break;
		
		case 2:
   Output_Clear();
		break;
		
		
	}
	

  }
	//	GPIO_PORTB7=0x00;
}    // just a prototype, link to your interpreter
// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumSamples, NumCreated, MaxJitter-MinJitter, DataLost, FilterWork, PIDwork
      
// 2) print debugging parameters 
//    i.e., x[], y[] 
//--------------end of Task 5-----------------------------


/*
int main(void){ int delay; // testmain2
  OS_Init();           // initialize, disable interrupts
	ADC_Open(ADC_CHANNEL,0,(CLOCKFREQ/ADC_SAMPLING_FS)-1);
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port D
	delay++;
	delay++;
  GPIO_PORTF_DIR_R |= 0x0D;   // make PD3-1 out
  GPIO_PORTF_DEN_R |= 0x0D;   // enable digital I/O on PD3-1
  PolledButtons_Init();
  OS_AddThreads(&Task1, &Task2, &Task3);
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}
*/


//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
// 
/*
// *******************Initial TEST**********
// This is the simplest configuration, test this first
// run this with 
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no select interrupts
// no ADC serial port or oLED output
// no calls to semaphores

void Thread1(void){
  Count1 = 0;          
  for(;;){
    Count1++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread2(void){
  Count2 = 0;          
  for(;;){
    Count2++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread3(void){
  Count3 = 0;          
  for(;;){
    Count3++;
    OS_Suspend();      // cooperative multitasking
  }
}
Sema4Type Free;       // used for mutual exclusion

int main(void){ //testmain1
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,1); 
  NumCreated += OS_AddThread(&Thread2,128,2); 
  NumCreated += OS_AddThread(&Thread3,128,3); 
 
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}
*/

/*
// *******************Second TEST**********
// Once the initalize test runs, test this 
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no select switch interrupts
// no ADC serial port or oLED output
// no calls to semaphores
void Thread1b(void){
  Count1 = 0;          
  for(;;){
    Count1++;
  }
}
void Thread2b(void){
  Count2 = 0;          
  for(;;){
    Count2++;
  }
}
void Thread3b(void){
  Count3 = 0;          
  for(;;){
    Count3++;
  }
}
int main(void){  // testmain2
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1b,128,1); 
  NumCreated += OS_AddThread(&Thread2b,128,2); 
  NumCreated += OS_AddThread(&Thread3b,128,3); 
//	OS_AddButtonTask(*test, 7);
 
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

*/
/*

// *******************Third TEST**********
// Once the second test runs, test this 
// no UART1 interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer2 interrupts, with or without period established by OS_AddPeriodicThread
// PortF GPIO interrupts, active low
// no ADC serial port or oLED output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyc;        // set in background
int Lost;
void BackgroundThread1c(void){   // called at 1000 Hz
  Count1++;
  OS_Signal(&Readyc);
}
void Thread5c(void){
  for(;;){
    OS_Wait(&Readyc);
    Count5++;   // Count2 + Count5 should equal Count1 
    Lost = Count1-Count5-Count2;
  }
}
void Thread2c(void){
  OS_InitSemaphore(&Readyc,0);
  Count1 = 0;    // number of times signal is called      
  Count2 = 0;    
  Count5 = 0;    // Count2 + Count5 should equal Count1  
  NumCreated += OS_AddThread(&Thread5c,128,3); 
  OS_AddPeriodicThread(&BackgroundThread1c,TIME_1MS,0); 
  for(;;){
    OS_Wait(&Readyc);
    Count2++;   // Count2 + Count5 should equal Count1
  }
}

void Thread3c(void){
  Count3 = 0;          
  for(;;){
    Count3++;
  }
}
void Thread4c(void){ int i;
  for(i=0;i<64;i++){
    Count4++;
    OS_Sleep(10);
  }
  OS_Kill();
  Count4 = 0;
}
void BackgroundThread5c(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4c,128,3); 
}
      
int main(void){   // Testmain3
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  OS_AddButtonTask(&BackgroundThread5c,2);
  NumCreated += OS_AddThread(&Thread2c,128,2); 
  NumCreated += OS_AddThread(&Thread3c,128,3); 
  NumCreated += OS_AddThread(&Thread4c,128,3); 
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;  // this never executes
}
*/
/*
// *******************Fourth TEST**********
// Once the third test runs, run this example
// Count1 should exactly equal Count2
// Count3 should be very large
// Count4 increases by 640 every time select is pressed
// NumCreated increase by 1 every time select is pressed

// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// Select switch interrupts, active low
// no ADC serial port or oLED output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyd;        // set in background
void BackgroundThread1d(void){   // called at 1000 Hz
static int i=0;
  i++;
  if(i==50){
    i = 0;         //every 50 ms
    Count1++;
    OS_bSignal(&Readyd);
  }
}
void Thread2d(void){
  OS_InitSemaphore(&Readyd,0);
  Count1 = 0;          
  Count2 = 0;          
  for(;;){
    OS_bWait(&Readyd);
    Count2++;     
  }
}
void Thread3d(void){
  Count3 = 0;          
  for(;;){
    Count3++;
  }
}
void Thread4d(void){ int i;
  for(i=0;i<640;i++){
    Count4++;
    OS_Sleep(1);
  }
  OS_Kill();
}
void BackgroundThread5d(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4d,128,3); 
}
int main(void){   // Testmain4
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  OS_AddPeriodicThread(&BackgroundThread1d,PERIOD,0); 
  OS_AddButtonTask(&BackgroundThread5d,2);
  NumCreated += OS_AddThread(&Thread2d,128,2); 
  NumCreated += OS_AddThread(&Thread3d,128,3); 
  NumCreated += OS_AddThread(&Thread4d,128,3); 
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;  // this never executes
}
*/

// *******************final user main DEMONTRATE THIS TO TA**********
int main(void){ 
  OS_Init();           // initialize, disable interrupts
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC; // activate port C
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port C
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOG; // activate port C
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; // activate port C
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port C
	Output_Init();
  Output_Color(15);
//  ADC_InitTimer0ATriggerSeq0(0, 49, 1000,0x02); //Prescale for units of us
  ADC_InitSWTriggerSeq3(1);
	//GPIO_PORTC_DIR_R |= 0xAC;   // make PC5 out
  //GPIO_PORTC_DEN_R |= 0xAC;   // enable digital I/O on PC5
	GPIO_PORTC_DIR_R |= 0x20;   // make PC5 out
  GPIO_PORTC_DEN_R |= 0x20;   // enable digital I/O on PC5
	GPIO_PORTA_DIR_R |= 0x80;   // make PC5 out
  GPIO_PORTA_DEN_R |= 0x80;   // enable digital I/O on PC5
	GPIO_PORTG_DIR_R |= 0x01;   // make PC5 out
  GPIO_PORTG_DEN_R |= 0x01;   // enable digital I/O on PC5
	GPIO_PORTB_DIR_R |= 0x80;   // make PC5 out
  GPIO_PORTB_DEN_R |= 0x80;   // enable digital I/O on PC5
	GPIO_PORTG_DIR_R |= 0x04;   // make PC5 out
  GPIO_PORTG_DEN_R |= 0x04;   // enable digital I/O on PC5
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
  MaxJitter = 0;       // OS_Time in 20ns units
  MinJitter = 10000000;

// ********initialize communication channels
  OS_MailBox_Init();
  OS_Fifo_Init();    // ***note*** 4 is not big enough*****

// *******attach background tasks***********
  OS_AddButtonTask(&ButtonPush,2);
  OS_AddPeriodicThread(&DAS,PERIOD,1); // 2 kHz real time sampling

  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&Interpreter,128,2); 
  NumCreated += OS_AddThread(&Consumer,128,1); 
  NumCreated += OS_AddThread(&PID,128,3); 
 
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}



