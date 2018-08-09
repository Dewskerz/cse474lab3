
// This header file is the driver for GPTM Timer A0

#include <stdint.h>

#ifndef TIMER0_H
#define TIMER0_H

// Registers configuring timer0
#define RCGCTIMER         (*((volatile uint32_t *) 0x400FE604))
#define EN0               (*((volatile uint32_t *) 0xE000E100))
#define T0_CTL            (*((volatile uint32_t *) 0x4003000C))
#define T0_CFG            (*((volatile uint32_t *) 0x40030000))
#define T0_TAMR           (*((volatile uint32_t *) 0x40030004))
#define T0_TAILR          (*((volatile uint32_t *) 0x40030028))
#define T0_RIS            (*((volatile uint32_t *) 0x4003001C))
#define T0_ICR            (*((volatile uint32_t *) 0x40030024))
#define T0_MIMR           (*((volatile uint32_t *) 0x40030018))

// Inputs to T0_TAILR for different speeds at 16MHz
#define SEC1              0x00F42400  // 1 second
#define SEC2              0x01E84800  // 2 seconds
#define SEC5              0x04C4B400  // 5 seconds

#define SEC1_4            0x003D0900
#define SEC1_80           0x04C4B400

#define TAEN_OFF          0x00
#define TAEN_ON           0x01
#define TAOTE             0x20        // enable Timer A ADC trigger

// Set up timer0 with given speed; allow interrupt if interrupt == 1
void timer0_setup(uint32_t speed, int interrupt);
// Enable timer0 if on == 1, otherwise disable timer0
// Enable timer A ADC trigger
void timer0_ctrl(int on);
// Return 1 when time is out; else return 0
int timer0_out();
// Clear the timer and begin next period
void timer0_clear();

#endif
