// ADCT0ATrigger.c
// Runs on LM3S8962
// Provide a function that initializes Timer0A to trigger ADC
// SS3 conversions and request an interrupt when the conversion
// is complete.
// Daniel Valvano
// June 30, 2011

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to the Arm Cortex M3",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2011

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

// Thumbwheel potentiometer with scaling resistor connected to ADC0

#include "lm3s8962.h"
#include "os.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"


/*
#define NVIC_EN0_INT17          0x00020000  // Interrupt 17 enable
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI4_R             (*((volatile unsigned long *)0xE000E410))  // IRQ 16 to 19 Priority Register
#define TIMER0_CFG_R            (*((volatile unsigned long *)0x40030000))
#define TIMER0_TAMR_R           (*((volatile unsigned long *)0x40030004))
#define TIMER0_CTL_R            (*((volatile unsigned long *)0x4003000C))
#define TIMER0_IMR_R            (*((volatile unsigned long *)0x40030018))
#define TIMER0_TAILR_R          (*((volatile unsigned long *)0x40030028))
#define TIMER0_TAPR_R           (*((volatile unsigned long *)0x40030038))
#define TIMER_CFG_16_BIT        0x00000004  // 16-bit timer configuration,
                                            // function is controlled by bits
                                            // 1:0 of GPTMTAMR and GPTMTBMR
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_CTL_TAOTE         0x00000020  // GPTM TimerA Output Trigger
                                            // Enable
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_IMR_TATOIM        0x00000001  // GPTM TimerA Time-Out Interrupt
                                            // Mask
#define TIMER_TAILR_TAILRL_M    0x0000FFFF  // GPTM TimerA Interval Load
                                            // Register Low
#define ADC_ACTSS_R             (*((volatile unsigned long *)0x40038000))
#define ADC0_RIS_R              (*((volatile unsigned long *)0x40038004))
#define ADC0_IM_R               (*((volatile unsigned long *)0x40038008))
#define ADC0_ISC_R              (*((volatile unsigned long *)0x4003800C))
#define ADC0_EMUX_R             (*((volatile unsigned long *)0x40038014))
#define ADC0_SSPRI_R            (*((volatile unsigned long *)0x40038020))
#define ADC0_PSSI_R             (*((volatile unsigned long *)0x40038028))
#define ADC0_SSMUX3_R           (*((volatile unsigned long *)0x400380A0))
#define ADC0_SSCTL3_R           (*((volatile unsigned long *)0x400380A4))
#define ADC0_SSFIFO3_R          (*((volatile unsigned long *)0x400380A8))
#define ADC_ACTSS_ASEN3         0x00000008  // ADC SS3 Enable
#define ADC_RIS_INR3            0x00000008  // SS3 Raw Interrupt Status
#define ADC_IM_MASK3            0x00000008  // SS3 Interrupt Mask
#define ADC_ISC_IN3             0x00000008  // SS3 Interrupt Status and Clear
#define ADC_EMUX_EM3_M          0x0000F000  // SS3 Trigger Select mask
#define ADC_EMUX_EM3_TIMER      0x00005000  // Timer
#define ADC_SSPRI_SS3_4TH       0x00003000  // fourth priority
#define ADC_SSPRI_SS2_3RD       0x00000200  // third priority
#define ADC_SSPRI_SS1_2ND       0x00000010  // second priority
#define ADC_SSPRI_SS0_1ST       0x00000000  // first priority
#define ADC_PSSI_SS3            0x00000008  // SS3 Initiate
#define ADC_SSMUX3_MUX0_M       0x00000003  // 1st Sample Input Select mask
#define ADC_SSMUX3_MUX0_S       0           // 1st Sample Input Select lshift
#define ADC_SSCTL3_TS0          0x00000008  // 1st Sample Temp Sensor Select
#define ADC_SSCTL3_IE0          0x00000004  // 1st Sample Interrupt Enable
#define ADC_SSCTL3_END0         0x00000002  // 1st Sample is End of Sequence
#define ADC_SSCTL3_D0           0x00000001  // 1st Sample Diff Input Select
#define ADC_SSFIFO3_DATA_M      0x000003FF  // Conversion Result Data mask
#define SYSCTL_RCGC0_R          (*((volatile unsigned long *)0x400FE100))
#define SYSCTL_RCGC1_R          (*((volatile unsigned long *)0x400FE104))
#define SYSCTL_RCGC0_ADC        0x00010000  // ADC0 Clock Gating Control
#define SYSCTL_RCGC0_ADCSPD_M   0x00000300  // ADC Sample Speed mask
#define SYSCTL_RCGC0_ADCSPD125K 0x00000000  // 125K samples/second
#define SYSCTL_RCGC1_TIMER0     0x00010000  // timer 0 Clock Gating Control
*/
#define MAXBUFFERSIZE           64          // maximum number of samples
#define SAMPLEFREQ              8000        // sampling frequency (min. 763 Hz)
#define CLOCKFREQ               50000000    // PLL clock frequency
#define RUNLENGTH 10000 

#define ADC0_SS0 30

#define GPIO_PORTC5  (*((volatile unsigned long *)0x40006080))
#define GPIO_PORTC7  (*((volatile unsigned long *)0x40006200))
#define GPIO_PORTC2  (*((volatile unsigned long *)0x40006010))
#define GPIO_PORTC3  (*((volatile unsigned long *)0x40006220))
#define GPIO_PORTA7  (*((volatile unsigned long *)0x40004200))
#define GPIO_PORTG0  (*((volatile unsigned long *)0x40026004))
#define GPIO_PORTB7  (*((volatile unsigned long *)0x40005200))
#define GPIO_PORTF2     (*((volatile unsigned long *)0x40025010))

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void(*ADCTask)(unsigned short);

/*
    // This array is used for storing the data read from the ADC FIFO. It
    // must be as large as the FIFO for the sequencer in use.  This example
    // uses sequence 3 which has a FIFO depth of 1.  If another sequence
    // was used with a deeper FIFO, then the array size must be changed.
    */
    unsigned long ulADC0_Value[1];
		
		
// There are many choices to make when using the ADC, and many
// different combinations of settings will all do basically the
// same thing.  For simplicity, this function makes some choices
// for you.  When calling this function, be sure that it does
// not conflict with any other software that may be running on
// the microcontroller.  Particularly, ADC sample sequencer 3
// is used here because it only takes one sample, and only one
// sample is absolutely needed.  Sample sequencer 3 generates a
// raw interrupt when the conversion is complete, and it is then
// promoted to an ADC controller interrupt.  Hardware Timer0A
// triggers the ADC conversion at the programmed interval, and
// software handles the interrupt to process the measurement
// when it is complete.
//
// A simpler approach would be to use software to trigger the
// ADC conversion, wait for it to complete, and then process the
// measurement.
//
// This initialization function sets up the ADC according to the
// following parameters.  Any parameters not explicitly listed
// below are not modified:
// Timer0A: enabled
// Mode: 16-bit, down counting
// One-shot or periodic: periodic
// Prescale value: programmable using variable 'prescale' [0:255]
// Interval value: programmable using variable 'period' [0:65535]
// Sample time is busPeriod*(prescale+1)*(period+1)
// Max sample rate: <=125,000 samples/second
// Sequencer 0 priority: 1st (highest)
// Sequencer 1 priority: 2nd
// Sequencer 2 priority: 3rd
// Sequencer 3 priority: 4th (lowest)
// SS3 triggering event: Timer0A
// SS3 1st sample source: programmable using variable 'channelNum' [0:3]
// SS3 interrupts: enabled and promoted to controller


//Self Written ADC
/*
void ADC_InitTimer0ATriggerSeq0(unsigned char channelNum, unsigned char prescale, unsigned short period){
  volatile unsigned long delay;
  // channelNum must be 0-3 (inclusive) corresponding to ADC0 through ADC3
  if(channelNum > 3){
    return;                                 // invalid input, do nothing
  }
  DisableInterrupts();
  // **** general initialization ****
  SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC;       // activate ADC
  SYSCTL_RCGC0_R &= ~SYSCTL_RCGC0_ADCSPD_M; // clear ADC sample speed field
  SYSCTL_RCGC0_R += SYSCTL_RCGC0_ADCSPD125K;// configure for 125K ADC max sample rate (default setting)
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0;    // activate timer0
  delay = SYSCTL_RCGC1_R;                   // allow time to finish activating
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN;          // disable timer0A during setup
  TIMER0_CTL_R |= TIMER_CTL_TAOTE;          // enable timer0A trigger to ADC
  TIMER0_CFG_R = TIMER_CFG_16_BIT;          // configure for 16-bit timer mode
  // **** timer0A initialization ****
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;   // configure for periodic mode
  TIMER0_TAPR_R = prescale;                 // prescale value for trigger
  TIMER0_TAILR_R = period;                  // start value for trigger
  TIMER0_IMR_R &= ~TIMER_IMR_TATOIM;        // disable timeout (rollover) interrupt
  TIMER0_CTL_R |= TIMER_CTL_TAEN;           // enable timer0A 16-b, periodic, no interrupts
  // **** ADC initialization ****
                                            // sequencer 0 is highest priority (default setting)
                                            // sequencer 1 is second-highest priority (default setting)
                                            // sequencer 2 is third-highest priority (default setting)
                                            // sequencer 3 is lowest priority (default setting)
 // ADC0_SSPRI_R = (ADC_SSPRI_SS0_1ST|ADC_SSPRI_SS1_2ND|ADC_SSPRI_SS2_3RD|ADC_SSPRI_SS3_4TH);
	ADC0_SSPRI_R = (ADC_SSPRI_SS3_4TH|ADC_SSPRI_SS0_1ST|ADC_SSPRI_SS1_2ND|ADC_SSPRI_SS2_3RD|ADC_SSPRI_SS0_1ST);
  ADC_ACTSS_R &= ~ADC_ACTSS_ASEN0;          // disable sample sequencer 0
  ADC0_EMUX_R &= ~ADC_EMUX_EM0_M;           // clear SS0 trigger select field
  ADC0_EMUX_R += ADC_EMUX_EM0_TIMER;        // configure for timer trigger event
  ADC0_SSMUX0_R &= ~ADC_SSMUX0_MUX0_M;      // clear SS0 1st sample input select field
                                            // configure for 'channelNum' as first sample input
  ADC0_SSMUX0_R += (channelNum<<ADC_SSMUX0_MUX0_S);
  ADC0_SSCTL0_R = (0                        // settings for 1st sample:
                   & ~ADC_SSCTL0_TS0        // read pin specified by ADC0_SSMUX0_R (default setting)
                   | ADC_SSCTL0_IE0         // raw interrupt asserted here
                   | ADC_SSCTL0_END0        // sample is end of sequence (default setting, hardwired)
                   & ~ADC_SSCTL0_D0);       // differential mode not used (default setting)
  ADC0_IM_R |= ADC_IM_MASK0;                // enable SS3 interrupts
  //ADC_ACTSS_R |= ADC_ACTSS_ASEN0;           // enable sample sequencer 0
  // **** interrupt initialization ****
                                            // ADC0=priority 2
  NVIC_PRI3_R = (NVIC_PRI3_R&0xFF00FFFF)|0x00800000; // bits 21-23
  NVIC_EN0_R |= NVIC_EN0_INT14;             // enable interrupt 14 in NVIC
  EnableInterrupts();
}
*/
void ADC_InitTimer0ATriggerSeq0(unsigned char channelNum, unsigned long prescale, unsigned short period, unsigned char priority){
	volatile unsigned long delay;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
 // SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

  
	
	TimerConfigure(TIMER0_BASE  , TIMER_CFG_16_BIT_PAIR|TIMER_CFG_A_PERIODIC);
	TimerPrescaleSet(TIMER0_BASE  , TIMER_A, prescale);
	TimerLoadSet(TIMER0_BASE, TIMER_A, period-1);
  TimerControlTrigger(TIMER0_BASE  , TIMER_A, true);
  TimerEnable(TIMER0_BASE , TIMER_A);
	
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | channelNum);
  ADCSequenceEnable(ADC0_BASE, 0);
	ADCIntEnable(ADC0_BASE, 0);
	IntPrioritySet(ADC0_SS0, priority);
  IntEnable(ADC0_SS0);
  
}


unsigned long ADCvalue[8];

void ADC0_Handler(void){
	GPIO_PORTC5 = 0x80;
	ADCIntClear(ADC0_BASE, 0);
	ADCSequenceDataGet(ADC0_BASE, 0,&ADCvalue[0]);     //function returns the number of samples copied into buffer
  //ADC_ISC_R = ADC_ISC_IN0;             // acknowledge ADC sequence 0 completion
  //ADCvalue = ADC_SSFIFO0_R&ADC_SSFIFO0_DATA_M;
	(*ADCTask)(ADCvalue[0]);
GPIO_PORTC5 = 0x00;
}




	
/*
void ADC_Collect2(unsigned int channelNum, unsigned int period, unsigned short buffer[], unsigned int numberOfSamples){
	ADC0_SSMUX3_R &= 0xFFFFFF8;                      //clear MUX0 field
	ADC0_SSMUX3_R += (channelNum<<ADC_SSMUX3_MUX0_S);
	TIMER0_TAILR_R = period; 
	DisableInterrupts();
  counter=0;
	EnableInterrupts();
  while(counter < numberOfSamples){
  buffer[counter]=ADCvalue;
  }
	ADC_Status();
	
}
*/

void ADC_Collect(unsigned int channelNum, unsigned int period, void(*task)(unsigned short)){long sr=0;
	
	sr=StartCritical();
	ADC_InitTimer0ATriggerSeq0(channelNum, 49, period,0x02); //Prescale for units of us
	//ADC_ACTSS_R |= ADC_ACTSS_ASEN0; 

	ADCTask=task;
  
	EndCritical(sr);
}	
//Self Written ADC_InitTriggerSeq3

/*
void ADC_InitSWTriggerSeq3(unsigned int channelNum){
  if(channelNum > 7){
    return;   // 0 to 7 are valid channels on the LM3S1968
  }
	
	
	SYSCTL_RCGC0_R |= 0x00010000;   // 1) activate ADC
 SYSCTL_RCGC0_R &= ~0x00000300;  // 2) configure for 125K
	//ADC0_SSPRI_R = (ADC_SSPRI_SS0_1ST|ADC_SSPRI_SS1_2ND|ADC_SSPRI_SS2_3RD|ADC_SSPRI_SS3_4TH);
	ADC0_SSPRI_R = (ADC_SSPRI_SS3_4TH|ADC_SSPRI_SS0_1ST|ADC_SSPRI_SS1_2ND|ADC_SSPRI_SS2_3RD|ADC_SSPRI_SS0_1ST);
  ADC_ACTSS_R &= ~ADC_ACTSS_ASEN3;          // disable sample sequencer 3
 ADC0_EMUX_R &= ~ADC_EMUX_EM3_M;           // clear SS3 trigger select field
  //ADC_EMUX_R &= ~0xF000;          // 5) seq3 is software trigger
  ADC0_EMUX_R += ADC_EMUX_EM3_TIMER;        // configure for timer trigger event
  ADC0_SSMUX3_R &= ~ADC_SSMUX3_MUX0_M;      // clear SS3 1st sample input select field
                                            // configure for 'channelNum' as first sample input
  ADC0_SSMUX3_R += (channelNum<<ADC_SSMUX3_MUX0_S);
  ADC0_SSCTL3_R = (0                        // settings for 1st sample:
                   & ~ADC_SSCTL3_TS0        // read pin specified by ADC0_SSMUX3_R (default setting)
                   | ADC_SSCTL3_IE0         // raw interrupt asserted here
                   | ADC_SSCTL3_END0        // sample is end of sequence (default setting, hardwired)
                   & ~ADC_SSCTL3_D0);       // differential mode not used (default setting)
  ADC0_IM_R |= ADC_IM_MASK3;                // enable SS3 interrupts
  ADC_ACTSS_R |= ADC_ACTSS_ASEN3;           // enable sample sequencer 3
}
*/

//DriverLib ADC_TriggerSeq3 single ended
void ADC_InitSWTriggerSeq3(unsigned int channelNum){
  
	 
  if(channelNum > 7){
    return;   // 0 to 7 are valid channels on the LM3S1968
  }
    // The ADC0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    /*
    // For this example ADC0 is used with AIN0 on port E7.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.  GPIO port E needs to be enabled
    // so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    /*
    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    */
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_7);
    /*
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    */
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    /*
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    */
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);

    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);
    /*
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    */
    ADCIntClear(ADC0_BASE, 3);
	
}
/* Self Written ADC_In
//------------ADC_In------------
// Busy-wait Analog to digital conversion
// Input: Channel Number
// Output: 10-bit result of ADC conversion

unsigned long ADC_In(unsigned int channelNum){  unsigned long result;
	ADC_SSMUX3_R += channelNum;     //    set channel
  ADC_PSSI_R = 0x0008;             // 1) initiate SS3
  while((ADC_RIS_R&0x08)==0){};    // 2) wait for conversion done
  result = ADC_SSFIFO3_R&0x3FF;    // 3) read result
  ADC_ISC_R = 0x0008;              // 4) acknowledge completion
  return result;

	
}
*/
//Driverlib ADC Single Ended
//------------ADC_In------------
// Busy-wait Analog to digital conversion
// Input: Channel Number
// Output: 10-bit result of ADC conversion

unsigned long ADC_In(unsigned int channelNum){  unsigned long result;long sr;
	sr=StartCritical();
	GPIO_PORTC2 = 0x04;
	// Trigger the ADC conversion.
        ADCProcessorTrigger(ADC0_BASE, 3);

        // Wait for conversion to be completed.
        while(!ADCIntStatus(ADC0_BASE, 3, false)){}

        // Clear the ADC interrupt flag.
        ADCIntClear(ADC0_BASE, 3);

        // Read ADC Value.
        ADCSequenceDataGet(ADC0_BASE, 3, ulADC0_Value);
					result = ulADC0_Value[0];

  GPIO_PORTC2 = 0x00;
					
EndCritical(sr);
  return result;

	
}

void PLL_Init(void){    // program 4.6 volume 1
  // 1) bypass PLL and system clock divider while initializing
  SYSCTL_RCC_R |=  0x00000800;   // PLL bypass
  SYSCTL_RCC_R &= ~0x00400000;   // do not use system divider
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R &= ~0x000003C0;   // clear XTAL field, bits 9-6
  SYSCTL_RCC_R +=  0x00000380;   // 0x0E, configure for 8 MHz crystal
  SYSCTL_RCC_R &= ~0x00000030;   // clear oscillator source field
  SYSCTL_RCC_R +=  0x00000000;   // configure for main oscillator source
  // 3) activate PLL by clearing PWRDN and OEN
  SYSCTL_RCC_R &= ~(0x00002000|0x00001000);
  // 4) set the desired system divider and the USESYSDIV bit
  SYSCTL_RCC_R &= ~0x07800000;   // system clock divider field
  SYSCTL_RCC_R +=  0x03800000;   // configure for 25 MHz clock
  SYSCTL_RCC_R |=  0x00400000;   // Enable System Clock Divider
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&0x00000040)==0){};  // wait for PLLRIS bit
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC_R &= ~0x00000800;
}



