/**
  Copyright 2018 
  Andrew Cunningham,  andyham@uw.edu, 1610973
  Abhyudaya Gupta

  CSE 474 SU 2018
  Lab 3

  Defines many constants needed for initialization and declares many functions

*/

#ifndef _474_HEADER_H_
#define _474_HEADER_H_
//#include <tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#include "intrinsics.h"

/// **************************************************************************
/// ************* Macro Definitions

#define SYSCTL_RCGCGPIO      (*((volatile uint32_t *)0x400FE608))

#define RCC2TEST (*((volatile uint32_t *)0x400FE070))

#define RCC_BASE            0x400FE000   
#define CONTROL_REGISTER  (*((volatile uint32_t *)0x400FE108))  //system control registers
#define ptr(x) (*((volatile uint32_t *)x))

// ******************************
// definitions used for PLL
// we just use RCC2
#define RCC2        ptr(0x400FE070)
#define RCC         ptr(0x400FE060)
#define SYSCTL_RIS  ptr(0x400FE050)

// ******************************
// definitions used for ADC
#define SYSCTL_RCGCADC_ADC        (*((volatile uint32_t *)0x400FE638))

#define ADC0_ACTSS_ADC            (*((volatile uint32_t *)0x40038000))
#define ADC0_EMUX_ADC             (*((volatile uint32_t *)0x40038014))
#define ADC0_IM_ADC               (*((volatile uint32_t *)0x40038008))
#define ADC0_ISC_ADC              (*((volatile uint32_t *)0x4003800C))
#define ADC0_DCISC_ADC            (*((volatile uint32_t *)0x40038034))
#define ADC0_PSSI_ADC             (*((volatile uint32_t *)0x40038028))
#define ADC0_SSCTL3_ADC           (*((volatile uint32_t *)0x400380A4))
#define ADC0_SSFIFO3_ADC          (*((volatile uint32_t *)0x400380A8))

#define GPIO_PORTE_AFSEL_ADC      (*((volatile uint32_t *)0x40024420))
#define GPIO_PORTE_DEN_ADC        (*((volatile uint32_t *)0x4002451C))
#define GPIO_PORTE_AMSEL_ADC      (*((volatile uint32_t *)0x40024528))


// old
#define SYSCTL_RCGCADC_R        (*((volatile uint32_t *)0x400FE638))
#define ADC_OUTPUT              (*((volatile uint32_t *)0x400380A8))

 #define ADC_BASE       ptr(0x400FE000)
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
#define GPIO_PORTA_AMSEL_R      (*((volatile uint32_t *)0x40004528))
#define GPIO_PORTA_AFSEL_R      (*((volatile uint32_t *)0x40004420)) 
#define GPIO_PORTA_PCTL_R       (*((volatile uint32_t *)0x4000452C))
#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4000451C))

// ***************************
// Definitions for Port C LED
#define GPIO_PORTC_AMSEL_R      (*((volatile uint32_t *)0x40006528))
#define GPIO_PORTC_DEN_R        (*((volatile uint32_t *)0x4000651C))
#define GPIO_PORTC_DIR_R        (*((volatile uint32_t *)0x40006400))
#define GPIO_PORTC_PCTL_R       (*((volatile uint32_t *)0x4000652C))
#define GPIO_PORTC_AFSEL_R      (*((volatile uint32_t *)0x40006420))
#define GPIO_PORTC_DATA_R       (*((volatile uint32_t *)0x400063FC))
#define C_DATA       (*((volatile uint32_t *)0x400063FC))
#define C_OFF  0xe0;
#define C_RED  0x20;
#define C_YELLOW 0x40;
#define C_GREEN  0x80;

// ***************************
// Definitions for Port F LED
// and onboard switches
#define F_DATA_DIRECTION  (*((volatile uint32_t *)0x40025400))  //port F data direction
#define F_D_A             (*((volatile uint32_t *)0x4002551C))  //port F analog or digital
#define F_PUR             (*((volatile uint32_t *)0x40025510))  //port F pull up resistors
#define F_DATA            (*((volatile uint32_t *)0x400253FC))  //port F data
#define F_LOCK            (*((volatile uint32_t *)0x40025520))  //port F lock
#define F_CR              (*((volatile uint32_t *)0x40025524))  //port F CR

#define PA5 (*((volatile unsigned long *)0x40004080))
#define PA6 (*((volatile unsigned long *)0x40004100))


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

//****************************************************
// Definitions for UART
#define SYSCTL_RCGCUART_R       (*((volatile uint32_t *)0x400FE618))  // runmode clock gating control
#define UART0_CTL_R             (*((volatile uint32_t *)0x4000C030))  // control
#define UART0_IBRD_R            (*((volatile uint32_t *)0x4000C024))  // int baud rate divisor
#define UART0_FBRD_R            (*((volatile uint32_t *)0x4000C028))  // fractional baud rate divider
#define UART0_CC_R              (*((volatile uint32_t *)0x4000CFC8))  // line control
#define UART0_LCRH_R            (*((volatile uint32_t *)0x4000C02C))  // clock configuration
#define UART0_FR_R              (*((volatile uint32_t *)0x4000C018))  // flag
#define UART0_DR_R              (*((volatile uint32_t *)0x4000C000))  // data

/*
80 MHZ
  without LCD
    BDR_80 = 80,000,000 / (16 * 9600) = 520
    DIVFRAC_80 = integer(833333333333 * 64 + 0.5) = 53
*/
#define BRD_80              520
#define DIVFRAC_80          53
#define BRD_4               26
#define DIVFRAC_4           3

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
#define F_RED       0x02  // 0b00010
#define F_BLUE      0x04  // 0b00100
#define F_VIOLET    0x06  // 0b00110
#define F_GREEN     0x08  // 0b01000
#define F_YELLOW    0x0A  // 0b01010
#define F_LIGHTBLUE 0x0D  // 0b01100
#define F_WHITE     0x1E  //0b11110

/// **************************************************************************
/// ************* Function Declarations
void ADCThermometer(void);
void LCDCube(void);
void UpdateCube(float theta);
void DrawCube(float x[], float y[], unsigned short color);
void FSM_TrafficLight(void);
void PortC_LED_Setter(int code);
void UART0_INIT(int ibrd, int fbrd);

/******************/
//Initializations
void Enable_All_GPIO(void); // turns on all the GPIO Ports
void PortF_LED_Init();     // initialize onboard port F LEDs
                           // initialize onboard buttons
                           // does not use tm4c123gh6pm.h
void Switch_Init(void);   // initializes PA5 and PA6 to interface with
                          // offboard buttoms
void LED_Init(void);      // initializes PA2, PA3, PA4 to interface with
                         // offboard LED
void Timer_Init(int clockspeedmhz);      // sets up the timer to count down from 16,000000
void Interrupt_Init(void);  // sets up functionality for
                            // the two onboard buttons to act as interrupts
                            // and the timer interrupt
void ADC_Andrew_Init(void); // funky name because there's an ADC_Init somewhere else
void PLL_Init(int speed);   // set the CPU clock to speed MHz
void PortC_LED_Init(void);  // initialize ports PA5, 6, and 7 to work with offboard LEDS
void UART0_INIT(int ibrd, int fbrd); // configure the UART to work at a baud rate determined by the parameters
char UART0_ReadChar();  // read a single character from the UART
void UART0_WriteChar(char c);  // write a single character
void UART0_WriteString(char* s); // write a string
int DMAtestmain(void);  // calls the functionality supplied by the lab handouts.  It was all consolidated here
void DMAPortFLED(void); // wrapper for DMA test
/************************/

void welcomeFlash(); // cycles through PortF colors

int TakeTemperature(void);
void SetLED_Temp(int temp);

// timer functions
// returns 1 if timer has counted down to 0, 0 otherwise
// when timer is at 0, rest to 16,000,000
bool Timer0Check(void);
void UpdatePart2TimerSwitch(bool state);
void UpdatePortFAndTimerPartB(void);

// wait functions
void waitn(int); // waits for n loops

#endif
