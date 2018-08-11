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

void Enable_All_GPIO(void) {
 SYSCTL_RCGCGPIO |= 0x1F;
 int n = 500;
 while(n--){}
}

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

void PortC_LED_Init(void) {
  volatile unsigned long delay = 0;
  CONTROL_REGISTER |= 0x04;          // activate clock for Port C
  while(delay < 100) {delay++;}
  GPIO_PORTC_AMSEL_R &= ~(0x70); // 0000xxxx analog
  GPIO_PORTC_DEN_R |= 0x70;      // 1111xxxx digital enable
  GPIO_PORTC_DIR_R |= 0x70;      // 1111xxxx output
  GPIO_PORTC_PCTL_R &= ~(0xFFFF0000); 
  GPIO_PORTC_AFSEL_R &= ~(0x70);
  GPIO_PORTC_DATA_R &= ~(0x70); // turn them off
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
  Timer0_CTL &= ~0x01; // lock the timer
  Timer0_CFG &= ~07; // set the 32 bit configuration
  Timer0_TnMR |= 0x02; // configure TnMR field to A
  Timer0_TnMR &= ~0x01;
  Timer0_TnMR &= ~0x20;
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
  // step 1, enable the 31 bit
  RCC2 |= (0x80000000);
  // step 2, bypass PLL, bit 11 = 0
  RCC2 |= (1<<11);
  // step 3, select crystal, bits 10-6 10101
  RCC |= 0x540;  // 0b10101000000
  RCC &= 0x57F;  // 0b10101111111
  // step 4, select oscillator source
  RCC2 &= ~(0x70); // 0b0001111
  // step 5, activate PLL by bit 13 to 0
  RCC2 &= ~(1<<13);
  // step 6, set system divider, bit 30 to 1
  RCC2 |= (1<<30);
  // step 7, set sys divider 28-22 to 0x4
  // step 7a, this is followup
  RCC |= (1<<22);
  
  // sart the mask
  int rcc2mask = ~(0x1FC00000);
  RCC2 &= rcc2mask;
  int n = (400 / speed) - 1;
  RCC2 |= (n<<22);
  // step 8, wait for PLL to lock
  do {} while ((RCC & 0x20) == 0x20);
  // step 8b, disable bypass
  RCC2 &= ~(1<<11);
}

void ADC_Andrew_Init(void) {
  
  // enable clock on RCGCADC
  SYSCTL_RCGCADC_ADC |= 0x01;
  //  disable the sample sequencer
  ADC0_ACTSS_ADC &= ~(1<<3);
  // configure the multiplexer to be 0b0101
  ADC0_EMUX_ADC &= ~(15<<12);
  ADC0_EMUX_ADC |= (5<<12);
  // update bit 5 of the timer to allow ADC to be triggered by timer
  Timer0_CTL |= (1<<5);
  // update SSCTL3 to be 0b0111
  ADC0_SSCTL3_ADC &= ~(0xF);
  ADC0_SSCTL3_ADC |= 0xE;
  // update IM to mask seq 3
  ADC0_IM_ADC |= (1<<3);
  // Jeff's extra step
  ADC0_ISC_ADC |= (1<<3);
  ADC0_DCISC_ADC |= (1<<3);
  // enable the sample sequencer
  ADC0_ACTSS_ADC |= (1<<3);
  // start a new conversion
  ADC0_PSSI_ADC |= (1<<3);
}

void UART0_INIT(int ibrd, int fbrd) {
  SYSCTL_RCGCUART_R |= 0x1;  // enable UART 0
  SYSCTL_RCGCGPIO |= (1<<0);
  GPIO_PORTA_AFSEL_R |= (1<<1)|(1<<0);
  GPIO_PORTA_PCTL_R |= (1<<0)|(1<<4);
  GPIO_PORTA_DEN_R |= (1<<0)|(1<<1);
  
  // setup UART0
  UART0_CTL_R &= ~(1<<0);     // disable while configuring
  UART0_IBRD_R = ibrd; // set the integer part of the BRD
  UART0_FBRD_R = fbrd;  // set the fractional part of the BRD
  
  UART0_LCRH_R |= (0x3<<5);            // write 0x06
  UART0_CC_R = 0x0;                     // sets source clock to 16MHZ
  UART0_CTL_R = (1<<0)|(1<<8)|(1<<9);  // enable the uart
  
}
char UART0_ReadChar() {
  while ((UART0_FR_R & (1<<4)) != 0);     // flag check the receive is complete
  return UART0_DR_R;                      // read that data
}
void UART0_WriteChar(char c) {
  while ((UART0_FR_R & (1<<5)) != 0);     // flag check the transmitter is complete
  UART0_DR_R = c;                          // write that char to the data 
}
void UART0_WriteString(char* s) {
  while (*s) {     // while not null terminator
    UART0_WriteChar(*(s++));     // write/print that char to the data
  }
}

