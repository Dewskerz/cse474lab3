/**
  Copyright 2018 Andrew Cunningham
  1610973
  andyham@uw.edu
  CSE 474

  Lab 2

  Defines many constants needed for initialization and declares many function
  headers
*/

#include <stdint.h>
//#include <tm4c123gh6pm.h>
#include <stdbool.h>
#include "intrinsics.h"

#define RCC2TEST (*((volatile uint32_t *)0x400FE070))

#define RCC_BASE            0x400FE000   
#define CONTROL_REGISTER  (*((volatile uint32_t *)0x400FE108))  //system control registers
#define ptr(x) (*((volatile uint32_t *)x))

// ******************************
// definitions used for PLL
// we just use RCC2
#define RCC2    ptr(0x400FE070)
#define RCC     ptr(0x400FE060)

// ******************************
// definitions used for ADC
#define SYSCTL_RCGCADC_R        (*((volatile uint32_t *)0x400FE638))
#define ADC_OUTPUT              (*((volatile uint32_t *)0x400380A8))

#define ADC_BASE       0x400FE000
#define ADC0_MAP       0x40038000
#define ADC_CONTROL    ptr(0x400FE638)
#define ADCACTSS_SS3  ptr(0x40038000)
#define ADC_MUX        ptr(0x40038014)
#define ADC_SSCTL3        ptr(0x400380A4)
#define ADC_IM         ptr(0x40038008)
#define ADC_ISC        ptr(0x4003800C)
#define ADC_D_ISC      ptr(0x40038034)
#define ADC_SEQ_INIT   ptr(0x40038028)



// ******************************
// definitions used for Port A offboard LED and offboard switches
#define PORTA_ANALOG      (*((volatile uint32_t *)0x40004528))
#define PORTA_CONTROL       (*((volatile uint32_t *)0x4000452C))
#define PORTA_DIRECTION        (*((volatile uint32_t *)0x40004400))
#define PORTA_FUNCTION      (*((volatile uint32_t *)0x40004420))
#define PORTA_DIGITAL        (*((volatile uint32_t *)0x4000451C))
#define PORTA_DATA       (*((volatile uint32_t *)0x400043FC))

// ***************************
// Definitions for Port F LED
#define F_DATA_DIRECTION  (*((volatile uint32_t *)0x40025400))  //port F data direction
#define F_D_A             (*((volatile uint32_t *)0x4002551C))  //port F analog or digital
#define F_PUR             (*((volatile uint32_t *)0x40025510))  //port F pull up resistors
#define F_DATA            (*((volatile uint32_t *)0x400253FC))  //port F data
#define F_LOCK            (*((volatile uint32_t *)0x40025520))  //port F lock
#define F_CR              (*((volatile uint32_t *)0x40025524))  //port F CR

#define PA5 (*((volatile unsigned long *)0x40004080))
#define PA6 (*((volatile unsigned long *)0x40004100))

// ******************************
// Definitions for offboard switches through PA5 and PA6


// ******************************************
//  Definitions for Timer
#define RCGCTIMER        (*((volatile uint32_t *)0x400FE604)) // SYSCTL_RCGCTIMER_R
#define Timer0_CTL       (*((volatile uint32_t *)0x4003000C))
#define Timer0_CFG       (*((volatile uint32_t *)0x40030000))
#define Timer0_TnMR      (*((volatile uint32_t *)0x40030004))
#define Timer0_TnILR     (*((volatile uint32_t *)0x40030028))
#define Timer0_FLAG      (*((volatile uint32_t *)0x40030024))  // TIMER0_ICR_R
#define Timer0_INTERRUPT (*((volatile uint32_t *)0x40030018))  // TIMER0_IMR_R
#define Timer0_TIMEOUT   (*((volatile uint32_t *)0x4003001C))

// *****************************************
// Definitions for interrupt
#define PORTF_EDGE_SENSITIVE     (*((volatile uint32_t *)0x40025404))
#define PORTF_TRIGGER_CONTROL    (*((volatile uint32_t *)0x40025408))
#define PORTF_TRIGGER_SIDE       (*((volatile uint32_t *)0x4002540C))
#define PORTF_FLAG               (*((volatile uint32_t *)0x4002541C))
#define PORTF_MASK               (*((volatile uint32_t *)0x40025410))

#define INTERRUPT_ENABLE         (*((volatile uint32_t *)0xE000E100))
#define INTERRUPT_PRIORITY       (*((volatile uint32_t *)0xE000E41C))

//for port F onboard LED
#define RED 0x02  //0b00010
#define BLUE 0x04  //0b00100
#define GREEN 0x08  //0b01000
#define YELLOW 0x0A  //0b01010
#define WHITE 0x1E  //0b11110

/*******************************/
// test functions
/** Blinking
  uses TimerCheck()
  blinks Port F LED on/off evert second
  cycles through Red, Blue, and Green
*/
void Blinking(void);         // Lab 2 part a. 1
void BlinkingPartB(void);   // Lab 2 parb.4

/** FSMRunPartA
  uses TimerCheck();
  all buttons have a 2 second response time
  PA5 starts oscillation between RED and GREEN every 5 seconds
  PA6 interrupts moves to YELLOW for 5 seconds before red for 5
*/
void FSMRunPartA(void);      // Lab 2 part a. 2
void FSMRunPartB(void);      // Lab 2 part b. 4
                              // a copy of part A, but tweeked to use the 
                             // timer interrupt
/********************************/

void ADCThermometer(void);

/*************/
//powers on and off external LED in PA2, used in figure 7
void LED_On(void);
void LED_Off(void);
/***************/
// powers on and off various external LED
// PA2: GREEN, PA3: YELLOW, PA4: RED
void RED_ON(void);
void YELLOW_ON(void);
void GREEN_ON(void);
void RED_OFF(void);
void YELLOW_OFF(void);
void GREEN_OFF(void);
void FSM_LED_Off(void);
/*********************************/
// used to check input off offboard switches for FSM
// FSM_On: PA5, FSM_Pass: PA6
unsigned long FSMOn(void);
unsigned long FSMPass(void);
/*******************************/




/***********************/
//Initialization
void PortF_LED_Init();     // initialize onboard port F LEDs
                           // initialize onboard buttons
                           // does not use tm4c123gh6pm.h
void Switch_Init(void);   // initializes PA5 and PA6 to interface with
                          // offboard buttoms
void LED_Init(void);      // initializes PA2, PA3, PA4 to interface with
                         // offboard LED
void Timer_Init(void);      // sets up the timer to count down from 16,000000
void Interrupt_Init(void);  // sets up functionality for
                            // the two onboard buttons to act as interrupts
                            // and the timer interrupt
void ADC_Andrew_Init(void);
void PLL_Init(void);
void Temp_Read_Start(void);
/************************/

void welcomeFlash(); // cycles through PortF colors

// timer functions
// returns 1 if timer has counted down to 0, 0 otherwise
// when timer is at 0, rest to 16,000,000
bool Timer0Check(void);
void UpdatePart2TimerSwitch(bool state);
void UpdatePortFAndTimerPartB(void);

// wait functions
void waitn(int); // waits for n loops
