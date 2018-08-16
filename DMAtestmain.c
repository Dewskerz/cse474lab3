// DMAtestmain.c
// the software triggerred DMA block transfer.

/*
Finished by Andrew Cunningham
andyham@uw.edu
CSE 474
Summer 2018

code was copied from Canvas, I claim no ownership
*/


#include <stdint.h>
#include "PLL.h"
#include "DMASoftware.h"

#include "tm4c123gh6pm.h"

//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
//const int32_t COLORWHEEL[8] = {RED, RED+GREEN, GREEN, GREEN+BLUE, BLUE, BLUE+RED, RED+GREEN+BLUE, 0};
const int32_t COLORWHEEL[3] = {RED+GREEN, GREEN, RED};  // yellow = red + green
#define SIZE 128
uint32_t SrcBuf[SIZE],DestBuf[SIZE];
int DMAtestmain(void){  
  volatile uint32_t delay; uint32_t i,t;
  PLL_Init();  // now running at 80 MHz
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOF; // enable Port F clock
  delay = SYSCTL_RCGCGPIO_R;              // allow time to finish 
  GPIO_PORTF_DIR_R |= 0x0E;    // make PF3-1 output (PF3-1 built-in LEDs)
  GPIO_PORTF_AFSEL_R &= ~0x0E; // disable alt funct on PF3-1
  GPIO_PORTF_DEN_R |= 0x0E;    // enable digital I/O on PF3-1
                               // configure PF3-1 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFF000F)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;      // disable analog functionality on PF
  LEDS = 0;                    // turn all LEDs off
  DMA_Init();  // initialize DMA channel 30 for software transfer
  t = 0;
  while(1){
  for(i=0;i<SIZE;i++){
    SrcBuf[i] = i;
    DestBuf[i] = 0;
  }
  while(DMA_Status()); // wait for idle
    DMA_Transfer(SrcBuf,DestBuf,SIZE);
    
    if (t % 3 == 0) {
      LEDS = COLORWHEEL[0];    
    } else if (t % 3 == 1) {
      LEDS = COLORWHEEL[1];
    } else {
      LEDS = COLORWHEEL[2];
    }
  //  LEDS = COLORWHEEL[t&0x07];
    t = t+1;
    
    // 80MHz - 8 bits => 80MHz/2^8 = 312500 = 0.3125MHz
    // 1/0.3125MHz = 3.2us => 1 count = 0.0000032 s
    // 1s = 1/0.0000032 = 312500
    for(delay = 0; delay < 312500; delay++){      // for 1 sec
    }
  }
}
