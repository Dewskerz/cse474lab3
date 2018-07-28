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
void Timer_Init(int clockspeedmhz) {
  RCGCTIMER |= 0x01; // enable the timer
  Timer0_CTL &= ~0x00000001; // lock the timer
  Timer0_CFG = 0x00000000; // set the 32 bit configuration
  Timer0_TnMR = 0x00000002; // configure TnMR field to A
  //Timer0_TnILR = 0x00F42400; // start interval 16,000000
  Timer0_TnILR = (clockspeedmhz * 1000000);
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

void PLL_Init(int speed) {
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
  // RCC |= (1<<11);
  RCC2 |= (1<<11);
  // step 3, select crystal, bits 10-6 10101
  RCC |= 0x540;  // 0b10101000000
  RCC &= 0x57F;  // 0b10101111111
  // step 4, select oscillator source
  // RCC &= ~(3<<4);  // 0b001111
  RCC2 &= ~(7<<4); // 0b0001111
  // step 5, activate PLL by bit 13 to 0
  RCC2 &= ~(1<13);
  // step 6, set system divider, bit 30 to 1
  RCC2 |= (1<<30);
  // step 7, set sys divider 28-22 to 0x4
  // step 7a, this is followup
  RCC |= (1<<22);
  
  // sart the mask
  // int rccmask =  ~(0x7C00000);
  int rcc2mask = ~(0x1FC00000);
  // rccmask = rccmask | (4<<22);
  // rcc2mask = rcc2mask | (24<<22);
  // RCC &= rccmask;
  RCC2 &= rcc2mask;
  int n = (400 / speed) - 1;
  // RCC |= (n<<22);
  RCC2 |= (n<<22);
  // step 8, wait for PLL to lock
  do {} while ((RCC & 0x20) == 0x20);
  // step 8b, disable bypass
  // RCC &= ~(1<<11);
  RCC2 &= ~(1<<11);
  //Timer0_TnILR = 0x4C4B400; // start interval 80,000,000
  //Timer0_TnILR = (speed * 1000000);
  Timer_Init(speed);
}

void ADC_Andrew_Init(void) {
  /*
  // step 1 enable clock on RCGCADC
  SYSCTL_RCGCADC_ADC |= 0x01;
  // step 2 enable appropriate RCGCGPIO register
  // in this case we need PE3
  // set that pin, pin6, to 1 in AFSEL
  // so clear that GPIODEN
  // and setting corresponding AMSEL in GPIOAMSEL
  SYSCTL_RCGCGPIO_ADC |= 0x10;
  GPIO_PORTE_AFSEL_ADC |= 0x8;
  GPIO_PORTE_DEN_ADC &= ~(0x8);
  GPIO_PORTE_AMSEL_ADC |= (0x8);
  
  //  disable the sample sequencer
  ADC0_ACTSS_ADC = 0;
  // configure the multiplexer
  ADC0_EMUX_ADC = (5<<11);
  ADC0_SSMUX3_ADC &= ~(0xF);
  ADC0_SSCTL3_ADC = 0xA;
  ADC0_ACTSS_ADC = 1;
  */
  // old shit
  // step 1 enable clock to ADC & GPIO module
  ADC_CONTROL |= 0x01;  // want 0x01
  // step 2 choose and disable sample sequence ADCACTSS
  // ADCACTSS_SS3 = 0x0;  // want 0b100
  ADCACTSS_SS3 &= ~(1<<2);  // want 0b0xx
  // step 3 choose software trigger ADCEMUX
  ADC_MUX = (5<<12);
  ADC_MUX |= (1<<12);
  ADC_MUX &= ~(1<<13);
  ADC_MUX |= (1<<14);
  ADC_MUX &= ~(1<<15);
  // step 3b
  // TAOTE bit 5 of general purpose timer control to true
  Timer0_CTL |= (1<<5);
  // step 4 skip
  // step 5 set ACD control to 0b1110
  ADC_SSCTL3 = 0xE;
  // set ADCIM to 0b100
  ADC_IM |= 0x8;
  //ADC_IM &= ~0x7;
  
  // start of things to do every time
  // ADCISC to 0b100
  ADC_ISC |= (1<<3);
  // ADC_ISC &= ~0x7;
  // ADCDCISC to 0b100
  ADC_D_ISC |= (1<<3);
  // ADC_D_ISC &= ~0x7;
  // step 8: turn on sequencer
  ADC_SSCTL3 &= ~(1<<1);
  ADC_SSCTL3 |= 0xF;
  // ADC_SAMPL_SEQ &= ~0x7;
  ADCACTSS_SS3 |= (1<<2);
  // step 9:  turn on sequencer ADCPSSI
  ADC_SEQ_INIT |= 0x4; // want 0b100
  ADC_SEQ_INIT &= ~0x3;
  
  //TakeTemperature();
}