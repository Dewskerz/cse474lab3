/*
  Copyright 2018 
  Andrew Cunningham,  andyham@uw.edu, 1610973
  Abhyudaya Gupta

  CSE 474 SU 2018
  Lab 3

  using Texas Instruments tm4c123gh6pm

*/

#include "header.h"


/*****   Test Code Guide ***********************************************
  1: ADCTermometer(void)
      LAB3 part A
*/
#define TEST_CODE 1

bool timerledonoffswitch = false;

void main(void) {
  PortF_LED_Init();     // initialize onboard port F LEDs
                        // initialize onboard buttons
  /* 
  not needed for lab3 part a
  Switch_Init();        // initializes PA5 and PA6 to interface with
                        // offboard buttoms
  LED_Init();           // initializes PA2, PA3, PA4 to interface with
                        // offboard LED
  */
  Timer_Init();         // enables timer 0 
  PLL_Init();           // sets cpu clock to 80MHz
  ADC_Andrew_Init();    // starts the ADC to take cpu temperatures and starts first sample
  welcomeFlash();       // display a friendly start-up flash
  Interrupt_Init();
  
  switch (TEST_CODE) {
  case 1:
    ADCThermometer();
    break;
  } 
}

void ADCThermometer(void) {
  __enable_interrupt();
  while(1) {
    //F_DATA = (timerledonoffswitch ? BLUE : 0);
    // Sit here and do nothing
    // while the handlers do all the work
    
  }
}




void Timer0_Handler(void) {
  timerledonoffswitch = !timerledonoffswitch;
  
  // take temperature
  int temp = (int) (147.5 - (247.5 * ADC_OUTPUT) / 4096.0);
  // convert the temperature
  if (temp > 0 && temp < 26) F_DATA = BLUE;
  else if (temp > 26 && temp < 60) F_DATA = GREEN;
  else F_DATA = RED;
  // display the appropriate LED
  // resets
  TempReadStart();
  Timer0_FLAG |= 0x00000001;  // clear timeout flag
}


// hander for the onboard switches
void GPIOPortF_Handler(void) {
  // TODO: configure to change the clock speed
  // PF0 - SW2 - 4MHz
  // PF4 - SW1 - 80MHz
  PORTF_FLAG |= 0x11;
}

void TempReadStart() {
  // start of things to do every time
  // ADCISC to 0b100
  ADC_ISC |= (1<<3);
  // ADC_ISC &= ~0x7;
  // ADCDCISC to 0b100
  ADC_D_ISC |= (1<<3);
  // ADC_D_ISC &= ~0x7;
  // step 8: turn on sequencer
  ADC_SSCTL3 &= ~(0x2);
  // ADC_SAMPL_SEQ &= ~0x7;
  ADCACTSS_SS3 |= (1<<3);
  // step 9:  turn on sequencer ADCPSSI
  ADC_SEQ_INIT |= 0x8; // want 0b100
  ADC_SEQ_INIT &= ~0x7;
}

// a small welcome flash, to acknowledge start of the program
void welcomeFlash() {
  // FSM_LED_Off();

  F_DATA = YELLOW;
  waitn(2000000);
  F_DATA = GREEN;
  waitn(2000000);
  F_DATA = BLUE;
  waitn(2000000);
  F_DATA = RED;
  waitn(5000000);
  F_DATA = 0x0;
  waitn(100000);
}

// stalls the processor for about n clock cycles
void waitn(int n) {
  while(n--){};
}


