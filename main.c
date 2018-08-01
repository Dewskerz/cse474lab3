/*
  Copyright 2018 
  Andrew Cunningham,  andyham@uw.edu, 1610973
  Abhyudaya Gupta

  CSE 474 SU 2018
  Lab 3

  using Texas Instruments tm4c123gh6pm

*/

#include "header.h"
#include "SSD2119.h"
#include <stdio.h>

unsigned short const Color4_Andrew[16] = {
  0,                                            //0 – black                   (#000000) 	000000 	0
 ((0x00>>3)<<11) | ((0x00>>2)<<5) | (0xAA>>3),  //1 – blue                    (#0000AA) 	000001 	1
 ((0x00>>3)<<11) | ((0xAA>>2)<<5) | (0x00>>3),  //2 – green                   (#00AA00) 	000010 	2
 ((0x00>>3)<<11) | ((0xAA>>2)<<5) | (0xAA>>3),  //3 – cyan                    (#00AAAA) 	000011 	3
 ((0xAA>>3)<<11) | ((0x00>>2)<<5) | (0x00>>3),  //4 – red                     (#AA0000) 	000100 	4
 ((0xAA>>3)<<11) | ((0x00>>2)<<5) | (0xAA>>3),  //5 – magenta                 (#AA00AA) 	000101 	5
 ((0xAA>>3)<<11) | ((0x55>>2)<<5) | (0x00>>3),  //6 – brown                   (#AA5500) 	010100 	20
 ((0xAA>>3)<<11) | ((0xAA>>2)<<5) | (0xAA>>3),  //7 – white / light gray      (#AAAAAA) 	000111 	7
 ((0x55>>3)<<11) | ((0x55>>2)<<5) | (0x55>>3),  //8 – dark gray /bright black (#555555) 	111000 	56
 ((0x55>>3)<<11) | ((0x55>>2)<<5) | (0xFF>>3),  //9 – bright blue             (#5555FF) 	111001 	57
 ((0x55>>3)<<11) | ((0xFF>>2)<<5) | (0x55>>3),  //10 – bright green           (#55FF55) 	111010 	58
 ((0x55>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3),  //11 – bright cyan            (#55FFFF) 	111011 	59
 ((0xFF>>3)<<11) | ((0x55>>2)<<5) | (0x55>>3),  //12 – bright red             (#FF5555) 	111100 	60
 ((0xFF>>3)<<11) | ((0x55>>2)<<5) | (0xFF>>3),  //13 – bright magenta         (#FF55FF) 	111101 	61
 ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0x55>>3),  //14 – bright yellow          (#FFFF55) 	111110 	62
 ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3)   //15 – bright white           (#FFFFFF) 	111111 	63
};

/*****   Test Code Guide ***********************************************
  1: ADCTermometer(void)
      LAB3 part A
*/
#define TEST_CODE 1

bool timerbool = false;
int counter =0;

void main(void) {
  PortF_LED_Init();     // initialize onboard port F LEDs
                        // initialize onboard buttons
  
  //not needed for lab3 part a
  Switch_Init();        // initializes PA5 and PA6 to interface with
                        // offboard buttoms
  //LED_Init();           // initializes PA2, PA3, PA4 to interface with
                        // offboard LED
  //timer init is happening inside of pll_init now // Timer_Init(); // enables timer 0 
  PLL_Init(80);           // sets cpu clock to 80MHz
  LCD_Init();
  Touch_Init();
  //ADC_Andrew_Init();    // starts the ADC to take cpu temperatures and starts first sample
  welcomeFlash();       // display a friendly start-up flash
  Interrupt_Init();
  __enable_interrupt();
  
  switch (TEST_CODE) {
  case 1:
    
    ADCThermometer();
    break;
  } 
}

void ADCThermometer(void) {
  
  //LCD_SetCursor(0,0);
  LCD_ColorFill(Color4_Andrew[1]);
  while(1) {
    //F_DATA = (timerbool ? BLUE : 0);
    // Sit here and do nothing
    // while the handlers do all the work
    
  }
}




void Timer0_Handler(void) {
  timerbool = !timerbool;
  
  int temp = TakeTemperature();
  SetLED_Temp(temp);
  // handle the LCD
  LCD_SetCursor(0,0);
  //LCD_PrintInteger(temp);
  //LCD_PrintInteger(++counter);
  //LCD_Printf("Current Counter: %d\nTemp: %d\nRAW: %d\n", ++counter, temp, ADC0_SSFIFO3_ADC);
  unsigned long x = Touch_ReadX();
  unsigned long y = Touch_ReadY();
  LCD_Printf("Current Counter: %d\nx: %d\ny: %d\n", ++counter, x, y);
  //printf("Time: %d Temp: %d RAW: %d Temp1: %d\n", ++counter, temp2, ADC0_SSFIFO3_ADC, temp1);
  //printf("Time: %d Temp: %d RAW:%d\n", counter, temp, ADC0_SSFIFO3_ADC);
  Timer0_FLAG |= 0x01;  // clear timeout flag
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
  //int temp = (int) (147.5 - (247.5 * ADC0_SSFIFO3_ADC) / 4096.0);
  // this conversion may not be right. I just calculated it with a room temperature
  // tiva
  //int temp = (int)(ADC0_SSFIFO3_ADC / 15.24);
  float VREFP = 3.3;
  float VREFN = 0;
  int temp = (int) (147.5 - ((75 * (VREFP - VREFN) * ADC0_SSFIFO3_ADC)) / 4096.0);
 
  // reset raw interrupt to 0
  ADC0_ISC_ADC |= (1<<3);
  // reset digital comparator interrupt status to 0
  ADC0_DCISC_ADC |= (1<<3);
  // reset the sample sequencer
  ADC0_SSCTL3_ADC &= ~(1<<3);
  // start a new sequence
  ADC0_PSSI_ADC |= (1<<3);
  
  return temp;
}

void SetLED_Temp(int temp) {
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
  }
}

// a small welcome flash, to acknowledge start of the program
void welcomeFlash() {
  // FSM_LED_Off();

  F_DATA = YELLOW;
  waitn(1000);
  F_DATA = GREEN;
  waitn(1000);
  F_DATA = BLUE;
  waitn(1000);
  F_DATA = RED;
  waitn(1000);
  F_DATA = 0x0;
  waitn(1000);
}

// stalls the processor for about n clock cycles
void waitn(int n) {
  while(n--){};
}