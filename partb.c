/*
  Copyright 2018 Andrew Cunningham
  andyham@uw.edu
  1610973

  CSE 474 SU 2018
  Lab 2b

  using Texas Instruments tm4c123gh6pm
  
  satisfies the requirements for part b
  Blinking blinks the Port F LED
  FSMRunPartB changed the functionality to work with the timer interrupt
  UpdatePortFAndTimerPartB is called by the Port F switch interrupt handler
  

  please see header for all function descriptions

  pa2 - green
  pa3 - yellow
  pa4 - red

  pa5 - fsm on
  pa5 - pass

*/

#include "header.h"

bool part2timerswitch = true;
bool bluelighttoggle = false;

void BlinkingPartB(void) {
  int colorcode = 1;
  while(1) {
    if (1) {
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
    part2timerswitch = false;
    while(! part2timerswitch);
  }
}

// runs the FSM for lab 2 section B
void FSMRunPartB(void) {
  // all buttons have a 2 second response time
  // FSMOn starts oscillation between RED and GREEN every 5 seconds
  // FSMPass interrupts GREEN and moves to YELLOW for 5 seconds before red for 5
  
  bool ison = true;
  bool isred = true;
  bool isyellow = false;
  int switchcounter = 0;
  int lightcounter = 0;
  
  while (1) {
    if (1) { // I decide to keep this all really simple
             // since the loop now only executes when the timer switches
            // this if statement can always be true
            // not editing the code was an explicit design decision
      if (FSMOn()) { // on/off switch depressed
        if (switchcounter == 1) {
          ison = !ison; // turn FSM either on or off
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
      } else { // this else never happens since the if is always trie
               // it's a depreciated relic of part 1
          FSM_LED_Off();
          isred = true;
          isyellow = false;
          lightcounter = 0;
      }// end ison light operation
    }// end timerCheck
    
    // below is the additon for part 2
    // we will sit here and wait for the second to finish
    part2timerswitch = false;
    while( ! part2timerswitch);
    F_DATA = (bluelighttoggle ? BLUE : 0x0);
  } // end while
}

void UpdatePart2TimerSwitch(bool state) {
  part2timerswitch = true;
  bluelighttoggle = !bluelighttoggle;
}

void UpdatePortFAndTimerPartB(void) {
  switch(F_DATA & 0x11) {
        case 0x10: //switch 1 is pressed
          Timer0_CTL &= ~0x00000001;
          F_DATA = RED;
          break;
        case 0x01: //switch 2 is pressed
          Timer0_CTL |= 0x00000001;
          break;
  }
}