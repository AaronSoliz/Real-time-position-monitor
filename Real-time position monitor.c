// Runs on LM4F120 or TM4C123
// Specifications:
// Measure distance using slide pot, sample at 10 Hz
// maximum distance can be any value from 1.5 to 2cm
// minimum distance is 0 cm
// Calculate distance in fixed point, 0.001cm
// Analog Input connected to PD2=ADC5
// displays distance on Sitronox ST7735
// PF3, PF2, PF1 are heartbeats 

#include <stdint.h>

#include "ST7735.h"
#include "TExaS.h"
#include "ADC.h"
#include "print.h"
#include "../inc/tm4c123gh6pm.h" 


uint32_t Mail;
uint32_t Status;
uint32_t Position;
uint32_t Data;
uint32_t Avg;



//*****the first three main programs are for debugging *****
// main1 tests just the ADC and slide pot, use debugger to see data
// main2 adds the LCD to the ADC and slide pot, ADC data is on ST7735
// main3 adds your convert function, position data is no ST7735

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
// Initialize Port F so PF1, PF2 and PF3 are heartbeats
void PortF_Init(void){
  volatile int delay;
  SYSCTL_RCGCGPIO_R |= 0x20;//clock for portf
  delay = SYSCTL_RCGCGPIO_R; 
  GPIO_PORTF_DIR_R |= 0x0E;  //setp pfs
  GPIO_PORTF_DEN_R |= 0x0E;
    GPIO_PORTF_AFSEL_R &= ~0x0E;
    GPIO_PORTF_AMSEL_R &= ~0x0E;
    GPIO_PORTF_PUR_R |= 0x0E;
    GPIO_PORTF_PCTL_R &= ~0x0000FFF0;
}
//uint32_t Data;        // 12-bit ADC
//uint32_t Position;    // 32-bit fixed-point 0.001 cm
//int main(void){       // single step this program and look at Data
  //TExaS_Init(SCOPE);       // Bus clock is 80 MHz 
  //ADC_Init();         // turn on ADC, set channel to 5
  //while(1){                
    //Data = ADC_In();  // sample 12-bit channel 5
  //}
//}
//uint32_t startTime,stopTime;
//uint32_t ADCtime,OutDectime; // in usec
//int main2(void){
  //TExaS_Init(SCOPE);  // Bus clock is 80 MHz 
  //ADC_Init();         // turn on ADC, set channel to 5
  //ST7735_InitR(INITR_REDTAB); 
    //NVIC_ST_RELOAD_R = 0x00FFFFFF; // maximum reload value
  //NVIC_ST_CURRENT_R = 0;    // any write to current clears it
  //NVIC_ST_CTRL_R = 5;

  //while(1){           // use scope to measure execution time for ADC_In and LCD_OutDec           
    //startTime= NVIC_ST_CURRENT_R;
    //Data = ADC_In();  // sample 12-bit channel 5
    //stopTime = NVIC_ST_CURRENT_R;
    //ADCtime = ((startTime-stopTime)&0x0FFFFFF)/80; // usec

    //ST7735_SetCursor(0,0);
    //startTime= NVIC_ST_CURRENT_R;
    //LCD_OutDec(Data); 
    //ST7735_OutString("    ");  // spaces cover up characters from last output
    //stopTime = NVIC_ST_CURRENT_R;
    //OutDectime = ((startTime-stopTime)&0x0FFFFFF)/80; // usec
  //}
//}

// your function to convert ADC sample to distance (0.001cm)
uint32_t Convert(uint32_t input){
  uint32_t Pos = 0;
    Pos = ((input*(439))/1000)+178;
    return Pos;
}
//int main3(void){ uint32_t time=0;
  //TExaS_Init(SCOPE);         // Bus clock is 80 MHz 
  //ST7735_InitR(INITR_REDTAB); 
  //PortF_Init();
  //ADC_Init();         // turn on ADC, set channel to 5
  //ST7735_PlotClear(0,2000); 
  //while(1){  
    //PF2 ^= 0x04;      // Heartbeat
    //Data = ADC_In();  // sample 12-bit channel 5
    //PF3 = 0x08;       // Profile Convert
    //Position = Convert(Data); 
    //PF3 = 0;          // end of Convert Profile
    //PF1 = 0x02;       // Profile LCD
    //ST7735_SetCursor(0,0);
    //LCD_OutDec(Data); ST7735_OutString("    "); 
    //ST7735_SetCursor(6,0);
    //LCD_OutFix(Position);
    //PF1 = 0;          // end of LCD Profile
    //ST7735_PlotPoint(Position);
    //if((time%8)==0){
      //ST7735_PlotNextErase();
    //}
    //time++;    
  //}
//}
uint32_t counter;
void SysTick_Init(uint32_t period){
    SYSCTL_RCGCGPIO_R |=0x08;
    counter=0;
    
    
    GPIO_PORTD_AMSEL_R &= ~0x01;
    GPIO_PORTD_PCTL_R &= ~0x000F;
    GPIO_PORTD_DIR_R |= ~0x01;
    GPIO_PORTD_AFSEL_R &= ~0x01;
    GPIO_PORTD_DEN_R |= ~0x01;
    
    
    
  NVIC_ST_CTRL_R = 0; //disable systick during setup
    NVIC_ST_RELOAD_R = period -1;
    NVIC_ST_CURRENT_R = 0;
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x40000000;
    NVIC_ST_CTRL_R =7;
    EnableInterrupts();
}

void SysTick_Handler(void){
    GPIO_PORTF_DATA_R ^=0x02;
    GPIO_PORTF_DATA_R ^=0x02;
    Data = ADC_In();
    Mail = Data;
    Status = 1;
    GPIO_PORTF_DATA_R ^=0x02;
    }
int main(void){ // this is real lab 8 main
    // 10 Hz sampling in SysTick ISR
    uint32_t time =0;
    TExaS_Init(SCOPE);
    ST7735_InitR(INITR_REDTAB);
    PortF_Init();
    ADC_Init();
    ST7735_PlotClear(0, 2000);
    SysTick_Init(8000000);
    uint32_t Output;
    while(1){
    
    while (Status==0){}
        PF2 ^= 0x04;
        PF3 = 0X08;
        Position = Convert(Mail);
        PF3 = 0;
        PF1 = 0x02;
        ST7735_SetCursor(0,0);
        LCD_OutDec(Mail); ST7735_OutString("  ");
        ST7735_SetCursor(6,0);
        LCD_OutFix(Position); ST7735_OutString(" cm");
        PF1 = 0;
        ST7735_PlotPoint(Position);
        Status = 0;
        if((time%8)==0){
            ST7735_PlotNextErase();
            
        }
        time++;

    }
}
