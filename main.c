/*
  Copyright 2018 
  Andrew Cunningham,  andyham@uw.edu, 1610973
  Abhyudaya Gupta

  CSE 474 SU 2018
  Lab 3

  using Texas Instruments tm4c123gh6pm

*/

#define TEST_CODE 1

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


/*****   Test Code Guide ***********************************************
  1: ADCTermometer(void)
      LAB3 part A and B
  2: DMA_PORTF_LED()
      DMA, makes port f led blink
  3: LCDCube(void)
      LAB3 Part D
      Draws a 3d rotating cube
  4: FSM_TrafficLight
*/

bool timerbool = false;
int counter = 0;
int currclockspeed = 80;
long xp;
long yp;


void main(void) {
  switch (TEST_CODE) {
  case 1:
    ADCThermometer();
    break;
  case 2:
    LCDCube();
    break;
  case 3:
    DMAPortFLED();
    break;
  case 4:
    FSM_TrafficLight();
    break;
  } 
}

void ADCThermometer(void) {
  Enable_All_GPIO();              // just turn on all of the GPIO Ports all at once
  Timer_Init(currclockspeed);
  PLL_Init(currclockspeed);      // sets cpu clock to 80MHz
  ADC_Andrew_Init();
  UART0_INIT(BRD_80, DIVFRAC_80);
  PortF_LED_Init();
  PortC_LED_Init();  // this doesn't NEED to be here.. it just turns off the LEDs if they are plugged in
  Interrupt_Init();
  __enable_interrupt();
  // starts the ADC to take cpu temperatures and starts first sample
  while(1) {
    //update once ever second
    if (timerbool) {
      // take the temperature and set the LED
      int temp = TakeTemperature();
      printf("temp%d: %d\n",counter,temp);
      SetLED_Temp(temp);
      // see if there's anything from the UART
      UART0_WriteString("Enter: \"f\" or \"s\" to change speed, \"ENTER\" otherwise:\n\r");
      char c = UART0_ReadChar();
      UART0_WriteChar(c);
      UART0_WriteString("\n\r");
      switch (c) {
      case 'f':
        currclockspeed = 80;
        Timer_Init(currclockspeed);
        PLL_Init(currclockspeed);
        UART0_INIT(BRD_80, DIVFRAC_80);
        break;
      case 's':
        currclockspeed = 4;
        Timer_Init(currclockspeed);
        PLL_Init(currclockspeed);
        UART0_INIT(BRD_4, DIVFRAC_4);
        break;
      case '\n':
        break;
      default:
        UART0_WriteString("No or invalid input\n\r");
      }
      
      // send the temperature out over UART
      char tempstring[3];
      sprintf(tempstring, "%d", temp);
      UART0_WriteString(tempstring);
      UART0_WriteString("\n\r");
      timerbool = !timerbool;
    }
  }
}

void DMAPortFLED(void) {
  // Part B goes here
  // TODO:  All of the initializations
  // look at ADCTermometer for inspiration
  // the chances are it's going to have a lot of the same functionality for
  // initializing port f, and led
  // i dont know what else goes into it
  // if DMA needs an initialization, put it either on line 177 or 199 of header.h
  // and initialize it inside of initializations.c
  // again, don't worry about breaking anything, GitHub is designed such that you literally cannot
  // irreversibly break everything
}

// draw a 3D rotating cube that is started and stopped by button
void LCDCube(void) {
  Enable_All_GPIO();              // just turn on all of the GPIO Ports all at once
  
  PLL_Init(currclockspeed);      // sets cpu clock to 80MHz
  Timer_Init(currclockspeed /8);
  LCD_Init();
  Touch_Init();
  Interrupt_Init();
  __enable_interrupt();
  
  bool cuberotate = true;
  //x range (1492, 3019) or 1527
  //y range (1391, 2330) or 939
  while(1) {
    if (timerbool) {
      LCD_SetCursor(302, 0);
      LCD_Printf("%d", counter);
      LCD_SetCursor(0, 0);
      LCD_DrawFilledRect(20,120,60,80,Color4_Andrew[2]);
      LCD_DrawFilledRect(240,120,60,80,Color4_Andrew[4]); 
      
      
      long xraw = Touch_ReadX();
      long yraw = Touch_ReadY();
      
      /*
      Several failed attempts were made to get the coordinates accurate
      Touch_ReadX();
      Touch_ReadY();
      long xandy = Touch_GetCoords();
      xp = (xandy>>16);
      yp = (xandy & 0xFFFF);

      xp = ((320* xraw)/1700)+320;
      yp = ((240* yraw)/1200)-280;
      
      xp = 320 - (((xraw - 1492)*320) / 1200);
      yp = 240 - (((yraw - 1400) * 240) / 1200);
      */
      
      LCD_Printf("Pressed                   \n");
      LCD_SetCursor(0, 0);
      LCD_Printf("Pressed(%d,%d)\n", xraw, yraw);
      
      if (xraw > 1690 && xraw < 2000 && yraw > 1820 && yraw < 2000) {cuberotate = false;}
      if (xraw > 2080 && xraw < 2390 && yraw > 2080 && yraw < 2280) {cuberotate = true;}
      if (cuberotate == true) {
        UpdateCube(PI/24);
      }
      
      timerbool = !timerbool;
    }
  }
}
void FSM_TrafficLight(void) {
  Enable_All_GPIO();              // just turn on all of the GPIO Ports all at once
  
  PLL_Init(currclockspeed);      // sets cpu clock to 80MHz
  Timer_Init(currclockspeed);    // update every second
  LCD_Init();
  Touch_Init();
  PortC_LED_Init();
  Interrupt_Init();
  __enable_interrupt();
  
  F_DATA = 0;
  
  int countto = 4;
  
  bool isred = true;
  bool ison = true;
  bool isyellow = false;
  bool onoffpressed = false;
  bool passpressed = false;
  int switchcounter = 0;
  int lightcounter = 0;
  
  while(1) {
    if (timerbool) {
      LCD_SetCursor(302, 0);
      LCD_Printf("%d", counter);
      
      LCD_DrawFilledRect(20,120,60,80,Color4_Andrew[14]);
      if (!ison) {
        LCD_DrawFilledRect(240,120,60,80,Color4_Andrew[10]); 
      } else {
        LCD_DrawFilledRect(240,120,60,80,Color4_Andrew[12]); 
      }
      
      long xraw = Touch_ReadX();
      long yraw = Touch_ReadY();
      if (xraw > 1620 && xraw < 20200 && yraw > 1600 && yraw < 1980) {
        onoffpressed = true;
        
      } else if (xraw > 2090 && xraw < 2420 && yraw > 1980 && yraw < 2280) {
        passpressed = true;
      } else {
        onoffpressed = false;
        passpressed = false;
      }
      LCD_SetCursor(0, 0);
      LCD_Printf("Pressed                   \n");
      LCD_SetCursor(0, 0);
      LCD_Printf("Pressed(%d,%d)\n", xraw, yraw);
      
      
      
      if (onoffpressed) { // on/off switch depressed
        if (switchcounter == 1) {
          ison = !ison; // turn FSM either on or off
          switchcounter = 0;
        } else {
          switchcounter = 1; // hold for 1 more second
        } // end onoff check
      } else if (passpressed) { // Passenger depressed
        if (switchcounter == 1) {
          if(!isred) {
            isyellow = true; // make yellow next
            isred = true; // make sure red after yellow
          }
          lightcounter = 0;  // yellow for 5 seconds
          switchcounter = 0;
        } else {
          switchcounter = 1; // hold for 1 more second
        } // end pass check
      } else { // either onoff or pass is pressed
        switchcounter = 0;
      } // end switch checks
      
      if (ison) { // only change lights of FSM is on
        if (isyellow) { // gotta blink yellow
          PortC_LED_Setter(2);
          if (lightcounter == countto) {
            isyellow = false;
            lightcounter = 0;
          } else {
            lightcounter ++;
          }
        } else { // end handle yellow
          if (isred) { // light is red
            PortC_LED_Setter(1);
            if (lightcounter == countto) { // switch to green
              isred = false;
              lightcounter = 0;
            } else {
              lightcounter ++;
            }
          } else { // light is green
            PortC_LED_Setter(3);
            if (lightcounter == countto) { // switch to red
              isred = true;
              lightcounter = 0;
            } else {
              lightcounter ++;
            }
          }
        } // end switch between red and green
      } else { // this else never happens since the if is always trie
               // it's a depreciated relic of part 1
          C_DATA &= C_OFF;
          isred = true;
          isyellow = false;
          lightcounter = 0;
      }// end ison light operation
      
      timerbool = !timerbool;
    }
  }
}

void PortC_LED_Setter(int code) {
  C_DATA &= C_OFF;
  switch (code) {
  case 1:
    C_DATA |= C_RED;
    break;
  case 2:
    C_DATA |= C_YELLOW;
    break;
  case 3:
    C_DATA |= C_GREEN;
    break;
  }
    
    
}


/*
All Timer0_Handler does is set the timerbool flag
and update the counter on the lcd
*/
void Timer0_Handler(void) {
  timerbool = !timerbool;
  if ((++counter) > 999) counter = 0;
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
          UART0_INIT(BRD_80, DIVFRAC_80);
          break;
        case 0x01: //switch 2 is pressed
          // PF0 - SW2 - 4MHz
          currclockspeed = 4;
          PLL_Init(currclockspeed);
          Timer_Init(currclockspeed);
          UART0_INIT(BRD_4, DIVFRAC_4);
          break;
  }
  PORTF_FLAG |= 0x11;
}

int TakeTemperature(void) {
  float VREFP = 3.3;
  float VREFN = 0;
  int temp = (int) (147.5 - ((75 * (VREFP - VREFN) * ADC0_SSFIFO3_ADC)) / 4096.0);
 
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
  LCD_DrawLine((int)(offsetx+x[0]),(int)(offsety+y[0]),(int)(offsetx+x[1]),(int)(offsety+y[1]), Color4_Andrew[color]);
  LCD_DrawLine((int)(offsetx+x[1]),(int)(offsety+y[1]),(int)(offsetx+x[2]),(int)(offsety+y[2]), Color4_Andrew[color]);
  LCD_DrawLine((int)(offsetx+x[2]),(int)(offsety+y[2]),(int)(offsetx+x[3]),(int)(offsety+y[3]), Color4_Andrew[color]);
  LCD_DrawLine((int)(offsetx+x[3]),(int)(offsety+y[3]),(int)(offsetx+x[0]),(int)(offsety+y[0]), Color4_Andrew[color]);
  
  LCD_DrawLine((int)(offsetx+x[4]),(int)(offsety+y[4]),(int)(offsetx+x[5]),(int)(offsety+y[5]), Color4_Andrew[color]);
  LCD_DrawLine((int)(offsetx+x[5]),(int)(offsety+y[5]),(int)(offsetx+x[6]),(int)(offsety+y[6]), Color4_Andrew[color]);
  LCD_DrawLine((int)(offsetx+x[6]),(int)(offsety+y[6]),(int)(offsetx+x[7]),(int)(offsety+y[7]), Color4_Andrew[color]);
  LCD_DrawLine((int)(offsetx+x[7]),(int)(offsety+y[7]),(int)(offsetx+x[4]),(int)(offsety+y[4]), Color4_Andrew[color]);
  
  LCD_DrawLine((int)(offsetx+x[1]),(int)(offsety+y[1]),(int)(offsetx+x[4]),(int)(offsety+y[4]), Color4_Andrew[color]);
  LCD_DrawLine((int)(offsetx+x[2]),(int)(offsety+y[2]),(int)(offsetx+x[7]),(int)(offsety+y[7]), Color4_Andrew[color]);
  LCD_DrawLine((int)(offsetx+x[3]),(int)(offsety+y[3]),(int)(offsetx+x[6]),(int)(offsety+y[6]), Color4_Andrew[color]);
  LCD_DrawLine((int)(offsetx+x[0]),(int)(offsety+y[0]),(int)(offsetx+x[5]),(int)(offsety+y[5]), Color4_Andrew[color]);
  
}