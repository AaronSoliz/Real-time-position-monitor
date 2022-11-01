// ADC.c
// Runs on LM4F120/TM4C123
// Provide functions that initialize ADC0
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

// ADC initialization function 
// Input: none
// Output: none
void ADC_Init(void){ 
    unsigned long volatile delay;
    SYSCTL_RCGCADC_R |= 0x01; //PORTD
    SYSCTL_RCGCGPIO_R |= 0x08;
    
    while((SYSCTL_RCGCGPIO_R & 0x08) != 8) {};
    GPIO_PORTD_DEN_R &= ~0x04; //PE2 
    GPIO_PORTD_AFSEL_R |= 0x04;
    GPIO_PORTD_AMSEL_R |= 0x04;
    GPIO_PORTD_DIR_R &= ~0x04;
    
    ADC0_PC_R |= 0x1;
    ADC0_SSPRI_R = 0x0123;
    ADC0_ACTSS_R &= ~0x0008;
    ADC0_EMUX_R &= ~0xF000; 
        
    SYSCTL_RCGCADC_R |= 0x01;
    delay = SYSCTL_RCGCADC_R; 
    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;




    ADC0_SSMUX3_R = 5; //(ADC0_SSMUX3_R & 0xFFFFFFF0)+9;
    ADC0_SSCTL3_R = 0x0006;
    ADC0_IM_R &= ~0x0008;
    ADC0_ACTSS_R |= 0x0008;
        
}
//------------ADC_In------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion
uint32_t ADC_In(void){  
    // 1) initiate SS3
  // 2) wait for conversion done
  // 3) read result
  // 4) acknowledge completion
  uint32_t Data = 0;
    ADC0_PSSI_R = 0x0008;
    while((ADC0_RIS_R & 0x08) == 0) {};
        Data = ADC0_SSFIFO3_R & 0xFFF; //DATA from ADC 
        ADC0_ISC_R = 0x0008;
        return Data;
}
