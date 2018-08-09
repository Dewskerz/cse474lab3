/*
  Copyright 2018 
  Andrew Cunningham,  andyham@uw.edu, 1610973
  Abhyudaya Gupta

  CSE 474 SU 2018
  Lab 3

  using Texas Instruments tm4c123gh6pm

*/

#define TEST_CODE 3

#include "header.h"
#include "SSD2119.h"
#include <stdio.h>
#include <math.h>

#define PI 3.14159265

unsigned short Color4_Andrew[16] = {
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

// start: depreciated helper for calibrating
void calibrate(unsigned long xraw,  unsigned long yraw);
int xmin = 9999999;
int xmax = 0;
int ymin = 9999999;
int ymax = 0;
// end: depreciated helper for calibrating

/*****   Test Code Guide ***********************************************
  1: ADCTermometer(void)
      LAB3 part A
  2: LCDCube(void)
      LAB3 Part D
      Draws a 3d rotating cube
*/

bool timerbool = false;
int counter = 0;
int currclockspeed = 80;
unsigned short xp;
unsigned short yp;

void main(void) {
  switch (TEST_CODE) {
  case 1:
    ADCThermometer();
    break;
  case 2:
    LCDCube();
    break;
  case 3:
    FSM_TrafficLight();
    break;
  } 
}

void ADCThermometer(void) {
  Enable_All_GPIO();              // just turn on all of the GPIO Ports all at once
  PortF_LED_Init();              // initialize onboard port F LEDs and onboard button
  PLL_Init(currclockspeed);      // sets cpu clock to 80MHz
  Timer_Init(currclockspeed);  //ADC_Andrew_Init();
  ADC_Andrew_Init();
  LCD_Init();
  Interrupt_Init();
  __enable_interrupt();
  // starts the ADC to take cpu temperatures and starts first sample
  while(1) {
    //update once ever second
    if (timerbool) {
      // take the temperature and set the LED
      int temp = TakeTemperature();
      SetLED_Temp(temp);
      // display some useful information on the LCD
      // note, touch features are disabled right now
      LCD_SetCursor(0,0);
      LCD_Printf("temp:    \nclockspeed:    \n");
      LCD_SetCursor(0,0);
      LCD_Printf("temp: %d\nclockspeed: %d\n", temp, currclockspeed);
      timerbool = !timerbool;
    }
  }
}

// draw a 3D rotating cube that is started and stopped by button
void LCDCube(void) {
  Enable_All_GPIO();              // just turn on all of the GPIO Ports all at once
  
  PLL_Init(currclockspeed);      // sets cpu clock to 80MHz
  Timer_Init(currclockspeed / 4);
  LCD_Init();
  Touch_Init();
  Interrupt_Init();
  __enable_interrupt();
  
  bool cuberotate = true;
  //x range (1492, 3019) or 1527
  //y range (1391, 2330) or 939
  while(1) {
    if (timerbool) {
      LCD_SetCursor(0, 0);
      LCD_DrawFilledCircle(xp, yp, 2, Color4_Andrew[0]);
      LCD_DrawFilledRect(5,210,150,30,Color4_Andrew[2]);
      LCD_DrawFilledRect(165,210,150,30,Color4_Andrew[4]); 
      
      
      unsigned long xraw = Touch_ReadX();
      unsigned long yraw = Touch_ReadY();
      xp = 320 - (((xraw - 1492)*320) / 1527);
      yp = 240 - (((yraw - 1400) * 240) / 1200);
      LCD_Printf("Pressed                   \n");
      LCD_SetCursor(0, 0);
      LCD_Printf("Pressed(%d,%d)\n", xp, yp);
      if (currclockspeed < 10) {
         LCD_Goto(13,0);
         LCD_Printf(" \n");
      }
      
      if (xp < 160 && yp > 200) cuberotate = true;
      if (xp > 160 && yp > 200) cuberotate = false;
      if (cuberotate == true) {
        UpdateCube(PI/12);
      }
      
      
      LCD_DrawFilledCircle(xp, yp, 2, Color4_Andrew[4]);
      timerbool = !timerbool;
    }
  }
}
void FSM_TrafficLight(void) {
  Enable_All_GPIO();              // just turn on all of the GPIO Ports all at once
  
  PLL_Init(currclockspeed);      // sets cpu clock to 80MHz
  Timer_Init(currclockspeed);
  LCD_Init();
  Touch_Init();
  PortC_LED_Init();
  Interrupt_Init();
  __enable_interrupt();
  
  while(1) {
    //C_DATA |= C_RED;
    C_DATA |= 0x70;
    C_DATA &= ~(1<<4);
    C_DATA |= 0x70;
    C_DATA &= ~(1<<5);
    C_DATA |= 0x70;
    C_DATA &= ~(1<<6);
  }
}


/*
All Timer0_Handler does is set the timerbool flag
and update the counter on the lcd
*/
void Timer0_Handler(void) {
  timerbool = !timerbool;
  if ((++counter) > 999) counter = 0;
  LCD_SetCursor(302, 0);
  LCD_Printf("%d", counter);
  Timer0_FLAG |= 0x01;  // clear timeout flag
}


// hander for the onboard switches
void GPIOPortF_Handler(void) {
  switch(F_DATA & 0x11) {
        case 0x10: //switch 1 is pressed
          // PF4 - SW1 - 80MHz
          currclockspeed = 80;
          PLL_Init(currclockspeed);
          Timer_Init(currclockspeed);
          break;
        case 0x01: //switch 2 is pressed
          // PF0 - SW2 - 4MHz
          currclockspeed = 4;
          PLL_Init(currclockspeed);
          Timer_Init(currclockspeed);
          break;
  }
  PORTF_FLAG |= 0x11;
}

int TakeTemperature(void) {
  float VREFP = 3.3;
  float VREFN = 0;
  int temp = (int) (147.5 - ((75 * (VREFP - VREFN) * ADC0_SSFIFO3_ADC)) / 4096.0);
  printf("temp: %d\n",temp);
 
  // reset raw interrupt to 0
  ADC0_ISC_ADC |= (1<<3);
  // reset digital comparator interrupt status to 0
  ADC0_DCISC_ADC |= (1<<3);
  // reset the sample sequencer
  ADC0_SSCTL3_ADC &= ~(1<<1);
  // start a new sequence
  ADC0_PSSI_ADC |= (1<<3);
  
  return temp;
}

void SetLED_Temp(int temp) {
  if (temp >= 0 && temp < 17) {
    F_DATA = F_RED;
  } else if (temp >= 17 && temp < 19) {
    F_DATA = F_BLUE;
  } else if (temp >= 19 && temp < 21) {
    F_DATA = F_VIOLET;
  } else if (temp >= 21 && temp < 23) {
    F_DATA = F_GREEN;
  } else if (temp >= 23 && temp < 25) {
    F_DATA = F_YELLOW;
  } else if (temp >= 25 && temp < 27) {
    F_DATA = F_LIGHTBLUE;
  } else if (temp >= 27 && temp < 40) {
    F_DATA = F_WHITE;
  }
}

void calibrate(unsigned long xraw, unsigned long yraw) {
  if (xraw > xmax) {
      xmax = xraw;
  }
  if (xraw < xmin) {
    xmin = xraw;
  }
  if (yraw > ymax) {
    ymax = yraw;
  }
  if (yraw < ymin) {
    ymin = yraw;
  }
  LCD_Printf("X_R: (%d,%d)\n", xmin, xmax);
  LCD_Printf("Y_R: (%d,%d)\n", ymin, ymax);
}

//bool first = true;
void UpdateCube(float theta) {
  static float x[] = {0,-40,-16,24,-31,10,37,-5};
  static float y[] = {36,26,12,22,-14,-4,-19,-30};
  
  DrawCube(x,y,0);
 
  float costheta = cos(theta);
  float sintheta = sin(theta);
  
  for (int i = 0; i < 8; i++) {
    float x1 = x[i]*costheta - y[i]*sintheta;
    float y1 = y[i]*costheta + x[i]*sintheta;
    x[i] = x1;
    y[i] = y1;
  }

  DrawCube(x,y,15);
}

void DrawCube(float x[], float y[], unsigned short color) {
  short offsetx = 160;
  short offsety = 120;
  LCD_DrawLine(offsetx+x[0],offsety+y[0],offsetx+x[1],offsety+y[1], Color4_Andrew[color]);
  LCD_DrawLine(offsetx+x[1],offsety+y[1],offsetx+x[2],offsety+y[2], Color4_Andrew[color]);
  LCD_DrawLine(offsetx+x[2],offsety+y[2],offsetx+x[3],offsety+y[3], Color4_Andrew[color]);
  LCD_DrawLine(offsetx+x[3],offsety+y[3],offsetx+x[0],offsety+y[0], Color4_Andrew[color]);
  
  LCD_DrawLine(offsetx+x[4],offsety+y[4],offsetx+x[5],offsety+y[5], Color4_Andrew[color]);
  LCD_DrawLine(offsetx+x[5],offsety+y[5],offsetx+x[6],offsety+y[6], Color4_Andrew[color]);
  LCD_DrawLine(offsetx+x[6],offsety+y[6],offsetx+x[7],offsety+y[7], Color4_Andrew[color]);
  LCD_DrawLine(offsetx+x[7],offsety+y[7],offsetx+x[4],offsety+y[4], Color4_Andrew[color]);
  
  LCD_DrawLine(offsetx+x[1],offsety+y[1],offsetx+x[4],offsety+y[4], Color4_Andrew[color]);
  LCD_DrawLine(offsetx+x[2],offsety+y[2],offsetx+x[7],offsety+y[7], Color4_Andrew[color]);
  LCD_DrawLine(offsetx+x[3],offsety+y[3],offsetx+x[6],offsety+y[6], Color4_Andrew[color]);
  LCD_DrawLine(offsetx+x[0],offsety+y[0],offsetx+x[5],offsety+y[5], Color4_Andrew[color]);
  
}