// filename **********OS.C***********
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
 
#include "os.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "lm3s8962.h"

/*
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOC      0x00000004  // port C Clock Gating Control
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile unsigned long *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority
*/
// function definitions in osasm.s
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
long StartCritical(void);
void EndCritical(long primask);
void StartOS(void);
void Connect_Pointers(void);
extern short Count;

  



Sema4Type RoomLeft; 
Sema4Type CurrentSize;
Sema4Type mutex;
Sema4Type Send;
Sema4Type Ack;
Sema4Type GlobalCount;

#define PERIOD TIME_1MS/2     // 2kHz sampling period in system time units
#define NUMTHREADS  5        // maximum number of threads
#define MAXNUMTHREADS  5        // maximum number of threads

#define STACKSIZE   100      // number of 32-bit words in stack

#define GPIO_PORTC5  (*((volatile unsigned long *)0x40006080))


#define FIFOSIZE 8
#define GPIO_PORTC5  (*((volatile unsigned long *)0x40006080))
#define GPIO_PORTC7  (*((volatile unsigned long *)0x40006200))
#define GPIO_PORTC2  (*((volatile unsigned long *)0x40006010))
#define GPIO_PORTC3  (*((volatile unsigned long *)0x40006220))
#define GPIO_PORTA7  (*((volatile unsigned long *)0x40004200))
#define GPIO_PORTG0  (*((volatile unsigned long *)0x40026004))
#define GPIO_PORTB7  (*((volatile unsigned long *)0x40005200))



unsigned long volatile *OS_PutPt;// put next
unsigned long volatile *OS_GetPt;// get next
unsigned long static OS_Fifo[FIFOSIZE];  

long MaxFifoSize=128;
unsigned long Mail = 0;

void (*PeriodicThreadTask)(void);
void(*ButtonTask)(void);            //user function on the rising edge of PF1


struct tcb{
	long *sp;          // pointer to stack (valid for threads not running
	struct tcb *next;  // linked-list pointer
	long sleepCount;	 // number of time quanta to sleep
	short alive;			 // 0 if dead
	long *status;			 // points to null if not blocked
										 // if blocked points to blocking resource

	//long id;					 // task number
};



typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
long Stacks[NUMTHREADS][STACKSIZE];

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: serial, ADC, systick, select switch and timer2 
// input:  none
// output: none
void OS_Init(void){  
  OS_DisableInterrupts();   // set processor clock to 50 MHz
  SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL |
                 SYSCTL_XTAL_8MHZ | SYSCTL_OSC_MAIN);
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7
}

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}
int OS_AddThreads(void(*task0)(void),
                 void(*task1)(void), 
                 void(*task2)(void)){ long status;
  status = StartCritical();
  tcbs[0].next = &tcbs[1]; // 0 points to 1
  tcbs[1].next = &tcbs[2]; // 1 points to 2
  tcbs[2].next = &tcbs[0]; // 2 points to 0
  SetInitialStack(0); Stacks[0][STACKSIZE-2] = (long)(task0); // PC
  SetInitialStack(1); Stacks[1][STACKSIZE-2] = (long)(task1); // PC
  SetInitialStack(2); Stacks[2][STACKSIZE-2] = (long)(task2); // PC
  RunPt = &tcbs[0];       // thread 0 will run first
  EndCritical(status);
  return 1;               // successful
}

/*
/// ******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(unsigned long theTimeSlice){
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}
*/
//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
void OS_Launch(unsigned long theTimeSlice){
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
	RunPt = &tcbs[0];
  StartOS();                   // start on the first task
}

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// You are free to select the time resolution for this function
// This task does not have a Thread ID
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), unsigned long period, unsigned long priority){

	volatile unsigned long delay;
	long status;
	//globalcount=globalcount+1;
	
	status=StartCritical();
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER2;    // activate timer2
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;    // activate port F
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC; // activate port C
	//GPIO_PORTF_DIR_R |= 0x01;                // make PC5 out (PC5 built-in LED)
  //GPIO_PORTF_DEN_R |= 0x01;
	PeriodicThreadTask = task;
  delay = SYSCTL_RCGC1_R;                   // allow time to finish 
	GPIO_PORTC_DIR_R |= 0x20;   // make PC5 out
  GPIO_PORTC_DEN_R |= 0x20;   // enable digital I/O on PC5
  TIMER2_CTL_R &= ~TIMER_CTL_TAEN;          // disable timer0A during setup
  TIMER2_CFG_R = TIMER_CFG_16_BIT;          // configure for 16-bit timer mode
	
	TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;// configure for periodic mode
  TIMER2_TAILR_R = period;          // interrupt every 10 ms
  TIMER2_TAPR_R = 0;               // interrupt every 20 ms (with prescale)
  TIMER2_IMR_R |= TIMER_IMR_TATOIM;// enable timeout (rollover) interrupt
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;// clear timer0A timeout flag
  TIMER2_CTL_R |= TIMER_CTL_TAEN;  // enable timer0A 16-b, periodic, interrupts
                                   // Timer0A=priority 2
	priority = priority << 29;																 
  //NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x40000000; // top 3 bits
	NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|priority;
  NVIC_EN0_R |= NVIC_EN0_INT23;    // enable interrupt 19 in NVIC
	
  EndCritical(status);
	return 0;
}

void Timer2A_Handler(void){
	GPIO_PORTC5 = 0x20;
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer0A timeout
	(*PeriodicThreadTask)();
  GPIO_PORTC5 = 0x00;
//	globalcount=globalcount+1;
}


void OS_Timer3AInit(void){

	volatile unsigned long delay;
	long status;
	
	status=StartCritical();
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER3;    // activate timer1B

  delay = SYSCTL_RCGC1_R;                   // allow time to finish 
  TIMER3_CTL_R &= ~TIMER_CTL_TAEN;          // disable timer1B during setup
  TIMER3_CFG_R = TIMER_CFG_16_BIT;          // configure for 16-bit timer mode
	
	TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;// configure for periodic mode
  TIMER3_TAILR_R = 499999;          // interrupt every 10 ms
  TIMER3_TAPR_R = 0;               // interrupt every 20 ms (with prescale)
  TIMER3_IMR_R |= TIMER_IMR_TATOIM;// enable timeout (rollover) interrupt
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// clear timer1A timeout flag
  TIMER3_CTL_R |= TIMER_CTL_TAEN;  // enable timer1A 16-b, periodic, interrupts
                                   // Timer1A=priority 7
															 
  //NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x40000000; // top 3 bits
	NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|(0x8FFFFFF);
  NVIC_EN1_R |= NVIC_EN1_INT35;    // enable interrupt 21 in NVIC
	
	GlobalCount.Value=0;
	
  EndCritical(status);
}

void Timer3A_Handler(void){
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout

	GlobalCount.Value=GlobalCount.Value+1;
}



void OS_ClearMsTime(void){
//	globalcount=0;
}

//unsigned long OS_MsTime(void){
//	return globalcount;
//}

int OS_AddFirstThread(void(*task)(void)){ long st;
	st = StartCritical();
	SetInitialStack(0);  // sets sp
	Stacks[0][STACKSIZE-2] = (long)(task); // PC
	tcbs[0].status = 0;  // not blocked when first initialized
	tcbs[0].next = &tcbs[0]; // linked to itself
	EndCritical(st);
	return 1;								// successful
}
/*
// ******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), 
								 unsigned long stackSize,
								 unsigned long priority){
	long st;
	int i = 0;
	int lowest_dead = MAXNUMTHREADS;
	st = StartCritical();
	if((RunPt) == 0){
		RunPt = &tcbs[0];
		SetInitialStack(0);  // sets sp
		Stacks[0][STACKSIZE-2] = (long)(task); // PC
		tcbs[0].sleepCount=0;
		tcbs[0].alive=1; 
		tcbs[0].status = 0;  // not blocked when first initialized
		tcbs[0].next = &tcbs[0]; // linked to itself
		
	}
	else{
	for(i = 0; i < MAXNUMTHREADS; i++){
		if(tcbs[i].alive == 0 && i < lowest_dead){
			lowest_dead = i; // this is where the new thread will be added
		}
	}
	SetInitialStack(lowest_dead);  // sets sp
	Stacks[lowest_dead][STACKSIZE-2] = (long)(task); // PC
	tcbs[lowest_dead].sleepCount = 0;  // initialize to 0 eventually will be a param
	tcbs[lowest_dead].alive = 1; // thread is born
	tcbs[lowest_dead].status = 0;  // not blocked when first initialized
	
	
}
  Connect_Pointers();
	EndCritical(st);
	return 1;								// successful

}
*/
//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), 
								 unsigned long stackSize,
								 unsigned long priority){
	long st;
	int i = 0;
	int lowest_dead = 9999;
	st = StartCritical();
	for(i = 0; i < MAXNUMTHREADS; i++){
		if(tcbs[i].alive != 1 && i < lowest_dead){
				lowest_dead = i; // this is where the new thread will be added
		}
	}
	if(lowest_dead == 9999){
			return 0; // no dead threads available
	}
	
	SetInitialStack(lowest_dead);  // sets sp
	Stacks[lowest_dead][STACKSIZE-2] = (long)(task); // PC
	tcbs[lowest_dead].sleepCount = 0;  // initialize to 0 eventually will be a param
	tcbs[lowest_dead].alive = 1; // thread is born
	tcbs[lowest_dead].status = 0;  // not blocked when first initialized
	Connect_Pointers();	
	EndCritical(st);
	return 1;								// successful
}
// ******* Connect_Pointers *********
// Connect pointers in order
// Highest live thread points
// to lowest live thread to maintain
// circular list
void Connect_Pointers(void){
	int i, j;
	for(i = 0; i < MAXNUMTHREADS; i++){
		if(tcbs[i].alive == 1){
			j = (i+1)%MAXNUMTHREADS;
			while(tcbs[j].alive == 0){
				j = (j+1)%MAXNUMTHREADS;
			}
			tcbs[i].next = &tcbs[j];
		}
	}
}



// ******** OS_Suspend ************
// Triggers Systick
// input:  none
// output: none
void OS_Suspend(void){
	NVIC_ST_CURRENT_R = 0;  // clear counter
	NVIC_INT_CTRL_R = 0x04000000;  // trigger Systick
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore( Sema4Type *semaPt, long value){
	semaPt->Value = value;
}

// ******** OS_Wait ************
// decrement semaphore and spin/block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
	OS_DisableInterrupts();
	while(semaPt->Value <= 0){
		OS_Suspend();  // efficient spinlock
		OS_EnableInterrupts();
		OS_DisableInterrupts();
	}
	semaPt->Value -= 1;
	OS_EnableInterrupts();
} 
// ******** OS_bWait ************
// if the semaphore is 0 then spin/block
// if the semaphore is 1, then clear semaphore to 0
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
	OS_DisableInterrupts();
	while(semaPt->Value == 0){
		OS_Suspend();  // efficient spinlock
		OS_EnableInterrupts();
		OS_DisableInterrupts();
	}
	semaPt->Value = 0;
	OS_EnableInterrupts();
} 

// ******** OS_bSignal ************
// set semaphore to 1, wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){ long status;
	status = StartCritical();
	semaPt->Value = 1;
	EndCritical(status);
}

//******** OS_AddButtonTask *************** 
// add a background task to run whenever the Select button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddButtonTask(void(*task)(void), unsigned long priority){int delay;long status;
	
	status=StartCritical();
	
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
  delay++;
  delay++;
	ButtonTask =  task;
  GPIO_PORTF_DIR_R &= ~0x02;  // make PF1 in (PF1 buttons)
  GPIO_PORTF_DEN_R |= 0x02;   // enable digital I/O on PF1
	GPIO_PORTF_AFSEL_R = 0x00;   // 3) regular port function
	GPIO_PORTF_PUR_R |= 0x02;	      // 5) set pull-up resistors
  GPIO_PORTF_IS_R &= ~0x02;   // PF3 is edge-sensitive (default setting)
  GPIO_PORTF_IBE_R &= ~0x02;  // PF3 is not both edges (default setting)
	GPIO_PORTF_IEV_R |= 0x00;  // PF3 rising edge event (default setting)
  GPIO_PORTF_ICR_R = 0x02;    // clear flag 3
  GPIO_PORTF_IM_R |= 0x02;    // enable interrupt on PF3
                              // GPIO PortF=priority 
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|(priority<<21); // bits 21-23
  NVIC_EN0_R |= NVIC_EN0_INT30;// enable interrupt 30 in NVIC
	EndCritical(status);
	return 1;  //successful
}

void GPIOPortF_Handler(void){
	
	if(GPIO_PORTF_RIS_R&0x02){
		GPIO_PORTF_ICR_R = 0x02;  // acknowledge flag3
    (*ButtonTask)();
		
	}
	
}


// ******** OS_Signal ************
// increment semaphore, wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){ long status;
	status = StartCritical();
	semaPt->Value += 1;
	EndCritical(status);
}

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void){return 1;}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){
	RunPt->sleepCount += sleepTime;
}

// ******** Dec_SleepCounters ****
void Dec_SleepCounts(void){
	int i;
	for(i = 0; i < MAXNUMTHREADS; i++){
		if(tcbs[i].sleepCount != 0){
			tcbs[i].sleepCount--;
		}
	}
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB memory
// input:  none
// output: none
void OS_Kill(void){long status;
	status=StartCritical();
	RunPt->alive = 0;
	Connect_Pointers();
	EndCritical(status);
	OS_Suspend();
	while(1);
}
// e.g.,
// AddIndexFifo(Tx,32,unsigned char, 1,0)
// SIZE must be a power of two
// creates TxFifo_Init() TxFifo_Get() and TxFifo_Put()

// macro to create a pointer FIFO
#define AddPointerFifo(NAME,SIZE,TYPE,SUCCESS,FAIL) \
TYPE volatile *NAME ## PutPt;    \
TYPE volatile *NAME ## GetPt;    \
TYPE static NAME ## Fifo [SIZE];        \
void NAME ## Fifo_Init(void){ long sr;  \
  sr = StartCritical();                 \
  NAME ## PutPt = NAME ## GetPt = &NAME ## Fifo[0]; \
  EndCritical(sr);                      \
}                                       \
int NAME ## Fifo_Put (TYPE data){       \
  TYPE volatile *nextPutPt;             \
  nextPutPt = NAME ## PutPt + 1;        \
  if(nextPutPt == &NAME ## Fifo[SIZE]){ \
    nextPutPt = &NAME ## Fifo[0];       \
  }                                     \
  if(nextPutPt == NAME ## GetPt ){      \
    return(FAIL);                       \
  }                                     \
  else{                                 \
    *( NAME ## PutPt ) = data;          \
    NAME ## PutPt = nextPutPt;          \
    return(SUCCESS);                    \
  }                                     \
}                                       \
int NAME ## Fifo_Get (TYPE *datapt){    \
  if( NAME ## PutPt == NAME ## GetPt ){ \
    return(FAIL);                       \
  }                                     \
  *datapt = *( NAME ## GetPt ## ++);    \
  if( NAME ## GetPt == &NAME ## Fifo[SIZE]){ \
    NAME ## GetPt = &NAME ## Fifo[0];   \
  }                                     \
  return(SUCCESS);                      \
}                                       \
unsigned short NAME ## Fifo_Size (void){\
  if( NAME ## PutPt < NAME ## GetPt ){  \
    return ((unsigned short)( NAME ## PutPt - NAME ## GetPt + (SIZE*sizeof(TYPE)))/sizeof(TYPE)); \
  }                                     \
  return ((unsigned short)( NAME ## PutPt - NAME ## GetPt )/sizeof(TYPE)); \
}

// e.g.,
// AddPointerFifo(Rx,32,unsigned char, 1,0)
// SIZE can be any size
// creates RxFifo_Init() RxFifo_Get() and RxFifo_Put()

// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(void){long sr=0;
	sr = StartCritical(); // make atomic
	RoomLeft.Value=FIFOSIZE; 
	CurrentSize.Value=0;
	mutex.Value=1;
	OS_PutPt = OS_GetPt = &OS_Fifo[0];  // Empty
	EndCritical(sr);
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data){
	/*
	OS_Wait(&RoomLeft);
	OS_Wait(&mutex);
	*(OS_PutPt++) = data;    //Put
	if(OS_PutPt == &OS_Fifo[FIFOSIZE]){
		OS_PutPt = &OS_Fifo[0]; //wrap

	}
	OS_Signal(&mutex);
	OS_Signal(&CurrentSize);
	return 1; 
	*/          
	
	unsigned long volatile *nextPutPt;
	nextPutPt = OS_PutPt+1;
	if(nextPutPt == &OS_Fifo[FIFOSIZE]){
		nextPutPt = &OS_Fifo[0]; //wrap
	}
	if(nextPutPt== OS_GetPt){
		return(0); 
	}
	else{
		*(OS_PutPt) = data; //Put
		OS_PutPt = nextPutPt;
		OS_Signal(&CurrentSize);
		return(1);
	}
	
	
}  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void){long data;
	/*
	OS_Wait(&CurrentSize);
	OS_Wait(&mutex);
	data=*(OS_GetPt++);
	if(OS_GetPt == &OS_Fifo[FIFOSIZE]){
		OS_GetPt = &OS_Fifo[0]; //wrap
		
	}
	OS_Signal(&mutex);
	OS_Signal(&RoomLeft);
	return data;
	*/
	
	OS_Wait(&CurrentSize);
	OS_Wait(&mutex);
	if( OS_PutPt == OS_GetPt ){ 
    return(0);                       
  }                                     
	data = *(OS_GetPt++);
	if(OS_GetPt == &OS_Fifo[FIFOSIZE]){
		OS_GetPt = &OS_Fifo[0]; //wrap
	}
	OS_Signal(&mutex);
	return data;
	

}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero  if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void){return CurrentSize.Value;}

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){long sr=0;
sr=StartCritical();
Send.Value=0;
Ack.Value=0;
EndCritical(sr);
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data){

Mail = data;    //Put

OS_Signal(&Send);
OS_Wait(&Ack);
}

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void){unsigned long data;

OS_Wait(&Send);
data=Mail;
OS_Signal(&Ack);

return data;

}

// ******** OS_Time ************
// reads a timer value 
// Inputs:  none
// Outputs: time in 20ns units, 0 to max
// The time resolution should be at least 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void){
	unsigned long time = NVIC_ST_CURRENT_R;
	return time;
	
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 20ns units 
// The time resolution should be at least 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start,
																unsigned long stop){
			
																	
	long diff;

	diff = stop - start;
	if(diff<0){
		diff=diff+NVIC_ST_RELOAD_R;

	}
	return diff;
}

