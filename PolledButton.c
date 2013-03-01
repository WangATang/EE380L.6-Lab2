

#include "lm3s8962.h"


void(*ButtonTask)(void);            //user function on the rising edge of PF1


int OS_AddButtonTask(void(*task)(void), unsigned long priority){int delay;
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
  delay++;
  delay++;
	ButtonTask =  task;
  GPIO_PORTF_DIR_R &= ~0x02;  // make PE1 in (PE1 buttons)
  GPIO_PORTF_DEN_R |= 0x02;   // enable digital I/O on PE1
	GPIO_PORTF_AFSEL_R = 0x00;   // 3) regular port function
	GPIO_PORTF_PUR_R |= 0x02;	      // 5) set pull-up resistors
  GPIO_PORTF_IS_R &= ~0x02;   // PE3 is edge-sensitive (default setting)
  GPIO_PORTF_IBE_R &= ~0x02;  // PE3 is not both edges (default setting)
	GPIO_PORTF_IEV_R |= 0x00;  // PE3 rising edge event (default setting)
  GPIO_PORTF_ICR_R = 0x02;    // clear flag 3
  GPIO_PORTF_IM_R |= 0x02;    // enable interrupt on PE3
                              // GPIO PortF=priority 2
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|(priority<<21); // bits 21-23
  NVIC_EN0_R |= NVIC_EN0_INT30;// enable interrupt 30 in NVIC
	
	return 1;  //successful
}

void GPIOPortF_Handler(void){
	
	if(GPIO_PORTF_RIS_R&0x02){
		GPIO_PORTF_ICR_R = 0x02;  // acknowledge flag3
    (*ButtonTask)();
	//	ButtonPush();
		
	}
	
}
