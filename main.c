/*
  Copyright 2018 
  Andrew Cunningham, andyham@uw.edu, 1610973
  Abhyudaya Gupta, ag98@uw.edu , 1664461        

  CSE 474 SU 2018
  Lab 3

  using Texas Instruments tm4c123gh6pm

*/

#define TEST_CODE 4

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
      pools the thermometer every second and
      adjusts the Port F LED
      prints to IAR I/O console
      prints to UART 0
      onboard buttons configured to change the PLL and Timer when pressed
      see functionality in GPIOPortF_Handler()
      The user, over UART, is constantly prompted to change the clockspeed
      the input is then used to switch the clockspeed between 4MHz and 80MHz
      or do nothing
  2: DMA_PORTF_LED()
      DMA, makes port f led blink
      finished the functionality from the code supplied by course instructors
  3: LCDCube(void)
      LAB3 Part D
      Draws a 3d rotating cube
      that can be started or stopped via virtual buttons on the LCD
  4: FSM_TrafficLight
      implements lab 2 FSM using virtual passenger or start/stop buttons
      and the lights are on GPIO port c
*/

bool timerbool = false;  // all the timer hander does is toggle this
int counter = 0;         // and increment this
int currclockspeed = 80;  // the current clockspeed, starts at 80MHz
long xp;                 // coordinates currently touched by the LCD
long yp;                // coordinates currently touched by the LCD


void main(void) {
  // please see above for functionality
  switch (TEST_CODE) {
  case 1:
    ADCThermometer();
    break;
  case 2:
    DMAPortFLED();
    break;
  case 3:
    LCDCube();
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
  ADC_Andrew_Init();            // enable thermoneter
  UART0_INIT(BRD_80, DIVFRAC_80);
  PortF_LED_Init();   // enable the Port F LED
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
      // this is done every second
      // it's poor design, but accomplishes the purpose of the lab
      UART0_WriteString("Enter: \"f\" or \"s\" to change speed, \"ENTER\" otherwise:\n\r");
      char c = UART0_ReadChar();
      UART0_WriteChar(c);
      UART0_WriteString("\n\r");
      // check to see if we got anything from UART!
      switch (c) :
      case 'f':
        // set the clockspeed to 80MHZ
        currclockspeed = 80;
        Timer_Init(currclockspeed);
        PLL_Init(currclockspeed);
        UART0_INIT(BRD_80, DIVFRAC_80);
        break;
      case 's':
        // set the clockspeed to 4MHZ
        currclockspeed = 4;
        Timer_Init(currclockspeed);
        PLL_Init(currclockspeed);
        UART0_INIT(BRD_4, DIVFRAC_4);
        break;
      case '\n':
        break;
      default:
        // they typed something..
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
  // call the supplied DMA functionality
  DMAtestmain();
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
    // only enter once every second
    if (timerbool) {
      LCD_SetCursor(302, 0);
      LCD_Printf("%d", counter);  // display a handy counter
      LCD_SetCursor(0, 0);
      
      // draw start and stop
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
      
      // the coordinates seem  to drift whenever the LCD is unplugged
      // and plugged in.  These should be mostly right though
      if (xraw > 1690 && xraw < 2000 && yraw > 1820 && yraw < 2000) {cuberotate = false;}
      if (xraw > 2080 && xraw < 2390 && yraw > 2080 && yraw < 2280) {cuberotate = true;}
      if (cuberotate == true) {
        // only rotate the cube if it's turned on
        // and rotate it by pi/24 radians
        UpdateCube(PI/24);
      }
      
      // switch timerbool back so that this only triggers once a second
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
  
  // turn off Port F LED, we dont need them
  F_DATA = 0;
  
  
  int countto = 4; // count to this before changing colors. change this to adjust the frequency
  
  bool isred = true;         // used to switch back and forth between red and green
  bool ison = true;          // the light will turn off if false
  bool isyellow = false;     // used to tell if the light is yellow
  bool onoffpressed = false;  // is the user pressing the on off button?
  bool passpressed = false;   // is the user pressing the passenger button?
  int switchcounter = 0;      // used to track how long the light has been presed
  int lightcounter = 0;      // used to track how long the light has been its color
  
  while(1) {
    if (timerbool) {
      LCD_SetCursor(302, 0);
      LCD_Printf("%d", counter);
      
      // draw passenger button
      LCD_DrawFilledRect(20,120,60,80,Color4_Andrew[14]);
      if (!ison) {
        // make start/stop button green
        LCD_DrawFilledRect(240,120,60,80,Color4_Andrew[10]); 
      } else {
        // make start/stop button red
        LCD_DrawFilledRect(240,120,60,80,Color4_Andrew[12]); 
      }
      
      // get the coordinates
      long xraw = Touch_ReadX();
      long yraw = Touch_ReadY();
      
      // determine if a button has been pressed
      if (xraw > 1620 && xraw < 20200 && yraw > 1600 && yraw < 1980) {
        onoffpressed = true;
        
      } else if (xraw > 2090 && xraw < 2420 && yraw > 1980 && yraw < 2280) {
        passpressed = true;
      } else {
        // none are pressed, make sure that the bools reflect that
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
        if (isyellow) { // light is currently yellow
          PortC_LED_Setter(2);  // make sure that yellow is on
          if (lightcounter == countto) {  // time to switch away from yellow
            isyellow = false;
            lightcounter = 0;
          } else {
            lightcounter ++;  // not quite there yet, stay yellow
          }
        } else { // end handle yellow
          if (isred) { // light is red
            PortC_LED_Setter(1);  // make sure that red is on
            if (lightcounter == countto) { // switch to green
              isred = false;
              lightcounter = 0;
            } else {
              lightcounter ++;  // stay red
            }
          } else { // light is green
            PortC_LED_Setter(3);  // make sure light is green
            if (lightcounter == countto) { // switch to red
              isred = true;
              lightcounter = 0;
            } else {
              lightcounter ++;  // remain green
            }
          }
        } // end switch between red and green
      } else {
          PortC_LED_Setter(0xBADA55);  // we want to make sure the setter turns off
          isred = true;  // make sure is red when we start
          isyellow = false;
          lightcounter = 0;
      }// end ison light operation
      
      timerbool = !timerbool;
    }
  }
}

void PortC_LED_Setter(int code) {
  C_DATA &= ~(0xe0);  // start by turning everything off
  switch (code) {
  case 1:  // red
    C_DATA |= C_RED;
    break;
  case 2:  // yellow
    C_DATA |= C_YELLOW;
    break;
  case 3:  // green
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