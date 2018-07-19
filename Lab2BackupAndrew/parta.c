/*
  Copyright 2018 Andrew Cunningham
  andyham@uw.edu
  1610973

  CSE 474 SU 2018
  Lab 2a

  using Texas Instruments tm4c123gh6pm
  
  satisfies the requirements for part a
  Blinking blinks the Port F LED
  FSMRunPartA has a check for the timer to reset

  please see header for all function descriptions
*/

#include "header.h"

// lab 2 part a.1
void Blinking(void) {
  bool timerswitch = false;
  int colorcode = 1;
  while(1) {
    if (Timer0Check()) {
      timerswitch = !timerswitch;
      colorcode = (colorcode == 6 ? 1 : colorcode + 1);
    }
    if ((colorcode % 2) == 0) F_DATA = 0x0;
    else {
      switch (colorcode) {
      case 1:
        F_DATA = RED;
        break;
      case 3:
        F_DATA = BLUE;
        break;
      case 5:
        F_DATA = GREEN;
        break;
      }
    }
  }
}

// runs the FSM for lab 2 section A
void FSMRunPartA(void) {
  // all buttons have a 2 second response time
  // FSMOn starts oscillation between RED and GREEN every 5 seconds
  // FSMPass interrupts GREEN and moves to YELLOW for 5 seconds before red for 5
  
  bool ison = true;
  bool isred = true;
  bool isyellow = false;
  int switchcounter = 0;
  int lightcounter = 0;
  
  F_DATA = (ison ? WHITE : 0x0);
  
  while (1) {
    if (Timer0Check()) { // timerCheck
      if (FSMOn()) { // on/off switch depressed
        if (switchcounter == 1) {
          ison = !ison; // turn FSM either on or off
          F_DATA = (ison ? WHITE : 0x0);
          switchcounter = 0;
        } else {
          switchcounter = 1; // hold for 1 more second
        } // end onoff check
      } else if (FSMPass()) { // Passenger depressed
        if (switchcounter == 1) {
          isyellow = true; // make yellow next
          isred = true; // make sure red after yellow
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
          FSM_LED_Off();
          YELLOW_ON();
          if (lightcounter == 4) {
            isyellow = false;
            lightcounter = 0;
          } else {
            lightcounter ++;
          }
        } else { // end handle yellow
          if (isred) { // light is red
            FSM_LED_Off();
            RED_ON();
            if (lightcounter == 4) { // switch to green
              isred = false;
              lightcounter = 0;
            } else {
              lightcounter ++;
            }
          } else { // light is green
            FSM_LED_Off();
            GREEN_ON();
            if (lightcounter == 4) { // switch to red
              isred = true;
              lightcounter = 0;
            } else {
              lightcounter ++;
            }
          }
        } // end switch between red and green
      } else {
          FSM_LED_Off();
          isred = true;
          isyellow = false;
          lightcounter = 0;
      }// end ison light operation
    }// end timerCheck
  } // end while
}

bool Timer0Check() {
  if((Timer0_TIMEOUT & 0x00000001) == 1) {
    Timer0_FLAG |= (1<<0);
    return true;
  }
  return false;
}