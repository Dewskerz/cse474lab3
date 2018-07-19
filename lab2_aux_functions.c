#include "header.h"

// a few auxiliary functions used in lab 2 that are not needed in lab 3
// removed from main.c to reduce clutter

//run/stop switch
unsigned long FSMOn(void) {
  //currently wired to switch
  return PA5;
}
// passenger switch
unsigned long FSMPass(void) {
  //currently wired to button
  return PA6;
}

// turns off all 3 LEDS
void FSM_LED_Off(void) {
  RED_OFF();
  YELLOW_OFF();
  GREEN_OFF();
  //F_DATA = WHITE;
}
