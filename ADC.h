// ADCT0ATrigger.h
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
// channelNum must be 0-3 (inclusive) corresponding to ADC0 through ADC3
void ADC_InitTimer0ATriggerSeq0(unsigned char channelNum, unsigned char prescale, unsigned short period,unsigned char priority);

//------------ADC_In------------
//Takes one sample from a ADC with specific channel 
// Input: Channel Number
// Output: ADC Sample

unsigned long ADC_In(unsigned int channelNum);

//------------ADC_Collect------------
//Takes a specific number of samples from ADC 
// Input: Channel Number, sampling rate, storage array, number of samples
// Output: none

void ADC_Collect(unsigned int channelNum, unsigned int period, void(*task)(unsigned short));
//void ADC_Collect2(unsigned int channelNum, unsigned int period, unsigned short buffer[], unsigned int numberOfSamples);


//------------ADC_Status------------
//Returns 0 when ADC_Collect finishes 
// Input: none
// Output: 0

int ADC_Status(void);

void ADC_InitSWTriggerSeq3(unsigned char channelNum);

//------------ADC_InSeq3------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 10-bit result of ADC conversion
unsigned long ADC_InSeq3(void);

void PLL_Init(void);
