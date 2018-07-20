/*
  Copyright 2018 Andrew Cunningham
  andyham@uw.edu
  1610973

  CSE 474 SU 2018
  Lab 3

  using Texas Instruments tm4c123gh6pm

*/

#include "header.h"


/*****   Test Code Guide ***********************************************
  1: Blinking()
    Satisfies section A part 1, blinking LED in Port F
    // does not use tm4c123gh6pm.h
  2: FSMPartA()
    runs the FSM from part A
    satisfying the second requirement for Part A
  3: timer and switch from part B
    Sets the interrupt and timer for Part B
    Satisfies part B task B.4
  4: FSMPartB()
    runs the FSM but with the timer and button interrupts.
    allows the timer to be started and stopped with the buttons and the fsm
    is also paused and started
    
*/
#define TEST_CODE 1

bool timerledonoffswitch = false;

void main(void) {
  PortF_LED_Init();     // initialize onboard port F LEDs
                        // initialize onboard buttons
                        // does not use tm4c123gh6pm.h
  Switch_Init();        // initializes PA5 and PA6 to interface with
                        // offboard buttoms
  LED_Init();           // initializes PA2, PA3, PA4 to interface with
                        // offboard LED
  Timer_Init();         // enables functionality for 
  PLL_Init();
  ADC_Andrew_Init();
  welcomeFlash();       // display a friendly start-up flash
  
  switch (TEST_CODE) {
  case 1:
    Interrupt_Init();
    __enable_interrupt();
    ADCThermometer();
    break;
  } 
}

void ADCThermometer(void) {
  while(1) {
    //F_DATA = (timerledonoffswitch ? BLUE : 0);
    // read the temperature
    
  }
}




void Timer0_Handler(void) {
  timerledonoffswitch = !timerledonoffswitch;
  UpdatePart2TimerSwitch(timerledonoffswitch);
  Timer0_FLAG |= 0x00000001;  // clear timeout flag
  
  // take temperature
  int temp = (147.5 - (247.5 * ADC_OUTPUT) / 4096.0);
  // convert the temperature
  if (temp > 0 && temp < 26) F_DATA = BLUE;
  else if (temp > 26 && temp < 60) F_DATA = GREEN;
  else F_DATA = RED;
  // display the appropriate LED
  // resets
  Temp_Read_Start();
}

void GPIOPortF_Handler(void) {
  UpdatePortFAndTimerPartB();
  PORTF_FLAG |= 0x11;
}



// turn  on PA4
void RED_ON (void) {
  //PA4
  PORTA_DATA |= 0x10;
  // F_DATA = RED;
}

// turn of PA4
void RED_OFF(void) {
  //F_DATA = WHITE;
   PORTA_DATA &= ~0x10;
}

// turn on PA2
void GREEN_ON (void) {
  //PA2
  PORTA_DATA |= 0x04;
  //F_DATA = GREEN;
}

// turn off PA2
void GREEN_OFF(void) {
  PORTA_DATA &= ~0x04;
  // F_DATA = WHITE;
}

// turn on PA3
void YELLOW_ON (void) {
  //PA3
  PORTA_DATA |= 0x08;
  //F_DATA = YELLOW;
}


// turn off PA3
void YELLOW_OFF(void) {
   PORTA_DATA &= ~0x08;
  //F_DATA = WHITE;
}

// a small welcome flash, to acknowledge start of the program
void welcomeFlash() {
  FSM_LED_Off();

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

// stalls the process for about n clock cycles
void waitn(int n) {
  while(n--){};
}


