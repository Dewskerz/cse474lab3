/**
  Copyright 2018 
  Andrew Cunningham,  andyham@uw.edu, 1610973
  Abhyudaya Gupta

  CSE 474 SU 2018
  Lab 3

  Defines many constants needed for initialization and declares many functions
*/

#include "header.h"

// defines some header functions used in main.c

// initlize the onboard portF LED
// initialize the two onboard switches
void PortF_LED_Init() {
  CONTROL_REGISTER = 0x20; //power on port F
  F_LOCK = 0x4C4F434B; //unlock value defined on datasheet
  F_CR = 0xFF;  //enables us to write to PUR
  F_DATA_DIRECTION = 0x0E; //0b0110 switches in, LED out
  F_PUR = 0x11; //0b100
  F_D_A = 0x1F; //0b11111 set all the ports to digital
}

// initialize the offboard switches in PA5 and PA6
void Switch_Init(void) {
  volatile unsigned long delay;

  CONTROL_REGISTER |= 0x0000001;          // activate the clock for port a
  delay = CONTROL_REGISTER;               // allow time for the clock to start
                                        // no need to unlock GPIO PORTA
  PORTA_ANALOG &= ~0x60;          // disable analog on PA5 and PA6
  PORTA_CONTROL &= 0x00F00000;      // PCTL GPIO on PA5 and PA6
  PORTA_DIRECTION &= ~0x60;            // set PA5, PA6 Direction to input
  PORTA_FUNCTION &= ~0x60;          // PA5 PA6 regular port function
  PORTA_DIGITAL |= 0x60;             // PA5 PA6 port set to digital
}

// initialize the offboard LED in PA2, PA3, PA4
void LED_Init(void) {
  volatile unsigned long delay;

  CONTROL_REGISTER |= 0x01;          // activate clock for Port A
  delay = CONTROL_REGISTER;           // allows time for clock to start
                                   // no need to unlock PA2
  PORTA_CONTROL &= 0x00000F00; // regular GPIO
  PORTA_ANALOG &= ~0x1C;     // disable analog function for PA2
  PORTA_DIRECTION |= 0x1C;        // PA2 set direction to output
  PORTA_FUNCTION &= ~0x1C;     // PA2 regular port funcion
  PORTA_DIGITAL |= 0x1C;        // PA2 enable digital port
}

// initialize timer0
void Timer_Init(void) {
  RCGCTIMER |= 0x01; // enable the timer
  Timer0_CTL &= ~0x00000001; // lock the timer
  Timer0_CFG = 0x00000000; // set the 32 bit configuration
  Timer0_TnMR = 0x00000002; // configure TnMR field to A
  Timer0_TnILR = 0x00F42400; // start interval 16,000000
  Timer0_INTERRUPT |= 0x11;  // turn on the interrupt
  Timer0_FLAG |= 0x00000001;  // clear timeout flag
  Timer0_CTL |= 0x00000001;  // unlock the timer timer
}

void Interrupt_Init(void) {
  INTERRUPT_ENABLE |= (1<<19);  // enable timer interrupt
  PORTF_EDGE_SENSITIVE &= ~0x11;  // makes bit 0 and 4 edge sensitive
  PORTF_TRIGGER_CONTROL &= ~0x11;  // trigger is controlled by iev
  PORTF_TRIGGER_SIDE = ~0x11;  // falling trigger level
  INTERRUPT_ENABLE |= (1<<30);  // enable switch interrupt
  INTERRUPT_PRIORITY = (3<<28);  // set switch interrupt priority 3
  PORTF_FLAG |= 0x11;  // clear any prior interrupts
  PORTF_MASK |= 0x11;  // unmask interrupts
}

void PLL_Init(void) {
  //RCC2TEST = 0xDEADBEEF;
  
  // totally cheated for 80MHz
  //RCC = 0x024E3540;
  //RCC2 = 0xC2404000; // 80MHZ
  // RCC2 = 0xC4C04000; // 4 MHZ
  // RCC2 = 0xC0404000; // 400 MHZ
  // RCC2 = 0xCC404000;  // 16 MHZ
  //return;
  
  //i think this is alllllllll wrong
  
  // step 1, enable the 31 bit
  RCC2 |= (0x80000000);
  // step 2, bypass PLL, bit 11 = 0
  RCC |= (1<<11);
  RCC2 |= (1<<11);
  // step 3, select crystal, bits 10-6 10101
  RCC |= (0x15<<6);
  RCC &= ~(0xA<<6);
  // step 4, select oscillator source, bits 6-4 000
  RCC &= ~(3<<4);
  RCC2 &= ~(0x7<<4);
  // step 5, activate PLL by bit 13 to 0
  RCC2 &= ~(1<13);
  // step 6, set system divider, bit 30 to 1
  RCC2 |= (1<<30);
  // step 7, set sys divider 28-22 to 0x4
  RCC2 &= ~(0x7C<<22);
  RCC2 |= (0xC<<22);
  // step 8, wait for PLL to lock
  do {} while ((RCC & 0x20));
  // step 8, disable bypass
  RCC &= ~(1<<11);
  RCC2 &= ~(1<<11);
}

void ADC_Andrew_Init(void) {
  // step 1 enable clock to ADC & GPIO module
  ADC_CONTROL |= 0x01;  // want 0x01
  // step 2 choose and disable sample sequence ADCACTSS
  ADCACTSS_SS3 = 0x0;  // want 0b100
  // step 3 choose software trigger ADCEMUX
  ADC_MUX |= 0x1;
  // step 4 skip
  // step 5 set ACD control to 0b1100
  ADC_SSCTL3 = 0xD;
  // set ADCIM to 0b100
  ADC_IM |= 0x8;
  ADC_IM &= ~0x7;
  
  TempReadStart();
}

