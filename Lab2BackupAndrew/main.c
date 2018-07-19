/*
  Copyright 2018 Andrew Cunningham
  andyham@uw.edu
  1610973

  CSE 474 SU 2018
  Lab 2a

  using Texas Instruments tm4c123gh6pm
  
  introduces Timer and Interrupt to the FSM traffic controller
  and a few other blinking functions

  uses two different c files
  parta.c and partb.c
  that satisfy both parts of the lab

  please see header for all function descriptions
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
#define TEST_CODE 4

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
  welcomeFlash();       // display a friendly start-up flash
  
  switch (TEST_CODE) {
  case 1:
    Blinking();
    break;
  case 2:
     FSMRunPartA();
     break;
  case 3:
    // blinking LED test from part B
    Interrupt_Init();
    __enable_interrupt();
    BlinkingPartB();
  case 4:
    // FSM test par
    Interrupt_Init();
    __enable_interrupt();
    FSMRunPartB();
  } 
}

// initlize the onboard portF LED
// initialize the two onboard switches
void PortF_LED_Init() {
  CONTROL_REGISTER = 0x20; //power on port F
  F_LOCK = 0x4C4F434B; //unlock value defined on datasheet
  F_CR = 0xFF;  //enables us to write to PUR
  F_DATA_DIRECTION = 0x0E; //0b0110 switches in, LED out
  F_PUR = 0x11; //0b100
  F_D_A = 0x1F; //0b11111 set all the ports to digital
}

// initialize the offboard switches in PA5 and PA6
void Switch_Init(void) {
  volatile unsigned long delay;

  CONTROL_REGISTER |= 0x0000001;          // activate the clock for port a
  delay = CONTROL_REGISTER;               // allow time for the clock to start
                                        // no need to unlock GPIO PORTA
  PORTA_ANALOG &= ~0x60;          // disable analog on PA5 and PA6
  PORTA_CONTROL &= 0x00F00000;      // PCTL GPIO on PA5 and PA6
  PORTA_DIRECTION &= ~0x60;            // set PA5, PA6 Direction to input
  PORTA_FUNCTION &= ~0x60;          // PA5 PA6 regular port function
  PORTA_DIGITAL |= 0x60;             // PA5 PA6 port set to digital
}

// initialize the offboard LED in PA2, PA3, PA4
void LED_Init(void) {
  volatile unsigned long delay;

  CONTROL_REGISTER |= 0x01;          // activate clock for Port A
  delay = CONTROL_REGISTER;           // allows time for clock to start
                                   // no need to unlock PA2
  PORTA_CONTROL &= 0x00000F00; // regular GPIO
  PORTA_ANALOG &= ~0x1C;     // disable analog function for PA2
  PORTA_DIRECTION |= 0x1C;        // PA2 set direction to output
  PORTA_FUNCTION &= ~0x1C;     // PA2 regular port funcion
  PORTA_DIGITAL |= 0x1C;        // PA2 enable digital port
}

// initialize timer0
void Timer_Init(void) {
  RCGCTIMER |= 0x01; // enable the timer
  Timer0_CTL &= ~0x00000001; // lock the timer
  Timer0_CFG = 0x00000000; // set the 32 bit configuration
  Timer0_TnMR = 0x00000002; // configure TnMR field to A
  Timer0_TnILR = 0x00F42400; // start interval 16,000000
  Timer0_INTERRUPT |= 0x11;  // turn on the interrupt
  Timer0_FLAG |= 0x00000001;  // clear timeout flag
  Timer0_CTL |= 0x00000001;  // unlock the timer timer
}

void Interrupt_Init(void) {
  INTERRUPT_ENABLE |= (1<<19);  // enable timer interrupt
  PORTF_EDGE_SENSITIVE &= ~0x11;  // makes bit 0 and 4 edge sensitive
  PORTF_TRIGGER_CONTROL &= ~0x11;  // trigger is controlled by iev
  PORTF_TRIGGER_SIDE = ~0x11;  // falling trigger level
  INTERRUPT_ENABLE |= (1<<30);  // enable switch interrupt
  INTERRUPT_PRIORITY = (3<<28);  // set switch interrupt priority 3
  PORTF_FLAG |= 0x11;  // clear any prior interrupts
  PORTF_MASK |= 0x11;  // unmask interrupts
}

// stalls the process for about n clock cycles
void waitn(int n) {
  while(n--){};
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

void Timer0_Handler(void) {
  timerledonoffswitch = !timerledonoffswitch;
  UpdatePart2TimerSwitch(timerledonoffswitch);
  Timer0_FLAG |= 0x00000001;  // clear timeout flag
}

void GPIOPortF_Handler(void) {
  UpdatePortFAndTimerPartB();
  PORTF_FLAG |= 0x11;
}

// helper functions
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

// turns off all 3 LEDS
void FSM_LED_Off(void) {
  RED_OFF();
  YELLOW_OFF();
  GREEN_OFF();
  //F_DATA = WHITE;
}
