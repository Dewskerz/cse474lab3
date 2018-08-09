// This header file is the driver for GPIO Port F, which controls
// two on-board pushbuttons and three LEDs. This file is an adapted
// version of lab1/src/lab1a.h

#ifndef PORT_F_H
#define PORT_F_H

// Inputs to GPIO_F_DATA for configuring LEDs and switches.
#define RED     0x02        // enable red LED
#define BLUE    0x04        // enable blue LED
#define GREEN   0x08        // enable green LED
#define VIOLET  0x06        // enable violet color
#define YELLOW  0x0A
#define LBLUE   0x0C
#define WHITE   0x0E
#define RGB     0x0E        // enable all three LEDs
#define SWITCH  0x11        // enable switch1 and switch2
#define CLEAR   0x00        // disable all

// Registers configuring GPIO Port F.
#define RCGCGPIO       *((volatile unsigned long *) 0x400FE608)
#define GPIO_F_DIR     *((volatile unsigned long *) 0x40025400)
#define GPIO_F_DEN     *((volatile unsigned long *) 0x4002551C)
#define GPIO_F_DATA    *((volatile unsigned long *) 0x400253FC)

// Input to RCGCGPIO for enabling Port F.
#define RCGCGPIO_F_ON  0x20

// Registers unlocking GPIO PF0.
#define GPIO_LOCK      *((volatile unsigned long *) 0x40025520)
#define GPIO_CR        *((volatile unsigned long *) 0x40025524)
#define GPIO_UR        *((volatile unsigned long *) 0x40025510)

// Input to GPIO_LOCK for unlocking PF0.
#define GPIO_CR_UNLOCK 0x4C4F434B

// Registers configuring interrupt.
#define GPIO_F_IS      *((volatile unsigned long *) 0x40025404)
#define GPIO_F_IBE     *((volatile unsigned long *) 0x40025408)
#define GPIO_F_IEV     *((volatile unsigned long *) 0x4002540C)
#define GPIO_F_IM      *((volatile unsigned long *) 0x40025410)
#define GPIO_F_ICR     *((volatile unsigned long *) 0x4002541C)

#endif