<<<<<<< HEAD
/*
  Copyright 2018 
  Andrew Cunningham,  andyham@uw.edu, 1610973
  Abhyudaya Gupta

  CSE 474 SU 2018
  Lab 3

  using Texas Instruments tm4c123gh6pm

*/

#include "header.h"
#include <stdio.h>


/*****   Test Code Guide ***********************************************
  1: ADCTermometer(void)
      LAB3 part A
*/
#define TEST_CODE 1

bool timerbool = false;

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
  PLL_Init(16);           // sets cpu clock to 80MHz
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
    //F_DATA = (timerbool ? BLUE : 0);
    // Sit here and do nothing
    // while the handlers do all the work
    
  }
}




void Timer0_Handler(void) {
  timerbool = !timerbool;
  
  int temp = TakeTemperature();
  if (temp != 0) {
    SetLED_Temp(temp);
  } else { 
    F_DATA = 0x0;
  }
  // resets
  Timer0_FLAG |= 0x00000001;  // clear timeout flag
}


// hander for the onboard switches
void GPIOPortF_Handler(void) {
  switch(F_DATA & 0x11) {
        case 0x10: //switch 1 is pressed
          // PF4 - SW1 - 80MHz
          PLL_Init(80);
          break;
        case 0x01: //switch 2 is pressed
          // PF0 - SW2 - 4MHz
          PLL_Init(4);
          break;
          // This is abhyudaya doing craxy stuff
  }
  
  
  PORTF_FLAG |= 0x11;
}

int TakeTemperature(void) {
  int temp =  (int) (147.5 - (247.5 * ADC_OUTPUT) / 4096.0);
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
  
  return temp;
}

void SetLED_Temp(int temp) {
  printf("Temp: %d\n", temp);
  if (temp >= 0 && temp < 17) {
    F_DATA = RED;
  } else if (temp >= 17 && temp < 19) {
    F_DATA = BLUE;
  } else if (temp >= 19 && temp < 21) {
    F_DATA = VIOLET;
  } else if (temp >= 21 && temp < 23) {
    F_DATA = GREEN;
  } else if (temp >= 23 && temp < 25) {
    F_DATA = YELLOW;
  } else if (temp >= 25 && temp < 27) {
    F_DATA = LIGHTBLUE;
  } else if (temp >= 27 && temp < 40) {
    F_DATA = WHITE;
  } else {
    // debugger
    //while(1) {};
  }
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


=======
/*
  Copyright 2018 
  Andrew Cunningham,  andyham@uw.edu, 1610973
  Abhyudaya Gupta

  CSE 474 SU 2018
  Lab 3

  using Texas Instruments tm4c123gh6pm

*/

#include "header.h"
#include <stdio.h>


/*****   Test Code Guide ***********************************************
  1: ADCTermometer(void)
      LAB3 part A
*/
#define TEST_CODE 1

bool timerbool = false;
int counter = 0;

void main(void) {
  PortF_LED_Init();     // initialize onboard port F LEDs
                        // initialize onboard buttons
  
  //not needed for lab3 part a
  Switch_Init();        // initializes PA5 and PA6 to interface with
                        // offboard buttoms
  //LED_Init();           // initializes PA2, PA3, PA4 to interface with
                        // offboard LED
  Timer_Init();         // enables timer 0 
  PLL_Init(16);           // sets cpu clock to 80MHz
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
    //F_DATA = (timerbool ? BLUE : 0);
    // Sit here and do nothing
    // while the handlers do all the work
    
  }
}




void Timer0_Handler(void) {
  timerbool = !timerbool;
  
  int temp = TakeTemperature();
  if (temp != 0) {
    SetLED_Temp(temp);
  } else { 
    F_DATA = 0x0;
  }
  // resets
  Timer0_FLAG |= 0x00000001;  // clear timeout flag
  // Andrew's comment
}


// hander for the onboard switches
void GPIOPortF_Handler(void) {
  switch(F_DATA & 0x11) {
        case 0x10: //switch 1 is pressed
          // PF4 - SW1 - 80MHz
          PLL_Init(80);
          break;
        case 0x01: //switch 2 is pressed
          // PF0 - SW2 - 4MHz
          PLL_Init(4);
          break;
  }
  
  
  PORTF_FLAG |= 0x11;
}

int TakeTemperature(void) {
  int temp =  (int) (147.5 - (247.5 * ADC_OUTPUT) / 4096.0);
  printf("Time %d TempRAW: %f TempConverted: %d\n", ++counter, (float) ADC_OUTPUT, temp);
  // start of things to do every time
  // ADCISC to 0b100
  ADC_ISC |= (1<<3);
  // ADC_ISC &= ~0x7;
  // ADCDCISC to 0b100
  ADC_D_ISC |= (1<<3);
  // ADC_D_ISC &= ~0x7;
  // step 8: turn on sequencer
  ADC_SSCTL3 &= ~(1<<1);
  // ADC_SAMPL_SEQ &= ~0x7;
  ADCACTSS_SS3 |= (1<<3);
  // step 9:  turn on sequencer ADCPSSI
  ADC_SEQ_INIT |= 0x4; // want 0b100
  ADC_SEQ_INIT &= ~0x3;
  
  return temp;
}

void SetLED_Temp(int temp) {
  //printf("Time %d Temp: %d\n", ++counter, temp);
  if (temp >= 0 && temp < 17) {
    F_DATA = RED;
  } else if (temp >= 17 && temp < 19) {
    F_DATA = BLUE;
  } else if (temp >= 19 && temp < 21) {
    F_DATA = VIOLET;
  } else if (temp >= 21 && temp < 23) {
    F_DATA = GREEN;
  } else if (temp >= 23 && temp < 25) {
    F_DATA = YELLOW;
  } else if (temp >= 25 && temp < 27) {
    F_DATA = LIGHTBLUE;
  } else if (temp >= 27 && temp < 40) {
    F_DATA = WHITE;
  } else {
    // debugger
    //while(1) {};
  }
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


>>>>>>> 42ecae5e2bc9a7fcd1cd15205f8242fc551d345a
