
#ifndef PORT_E_H
#define PORT_E_H

// Registers configuring GPIO Port E.
#define RCGCGPIO       *((volatile unsigned long *) 0x400FE608)
#define GPIO_E_DIR     *((volatile unsigned long *) 0x40024400)
#define GPIO_E_DEN     *((volatile unsigned long *) 0x4002451C)
#define GPIO_E_AFSEL   *((volatile unsigned long *) 0x40024420)
#define GPIO_E_AMSEL   *((volatile unsigned long *) 0x40024528)


// Input to RCGCGPIO for enabling Port E.
#define RCGCGPIO_E_ON  0x10

#define PE3            0x08

#endif