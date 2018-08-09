#ifndef UART0_H
#define UART0_H

// Find the Baud-Rate Divisor:
// BRD = clock_freq / (16 * 9600)
// BRD_4 = 26.041666666666666666666   DIVFRAC = 3
// BRD_80 = 520.8333333333333333333   DIVFRAC = 53

#define RCGCUART            *((volatile unsigned long *) 0x400FE618)
#define UART0_CTL           *((volatile unsigned long *) 0x400C0030)
#define UART0_IBRD          *((volatile unsigned long *) 0x400C0024)
#define UART0_FBRD          *((volatile unsigned long *) 0x400C0028)
#define UART0_LCRH          *((volatile unsigned long *) 0x400C002C)
#define UART0_CC            *((volatile unsigned long *) 0x400C0FC8)
#define UART0_IM            *((volatile unsigned long *) 0x400C0038)
#define UART0_FR            *((volatile unsigned long *) 0x400C0018)
#define UART0_DR            *((volatile unsigned long *) 0x400C0000)

// GPIO Port A for UART0
#define RCGCGPIO            *((volatile unsigned long *) 0x400FE608)
#define GPIO_A_AFSEL        *((volatile unsigned long *) 0x40004420)
#define GPIO_A_PCTL         *((volatile unsigned long *) 0x4000452C)
#define GPIO_A_DEN          *((volatile unsigned long *) 0x4000451C)

#define EN0                 *((volatile unsigned long *) 0xE000E100)

// inputs (4MHz or 80MHz) to uart0_setup()
#define BRD_4               26
#define DIVFRAC_4           3
#define BRD_80              520
#define DIVFRAC_80          53

void uart0_setup(int ibrd, int fbrd) {
    RCGCUART |= 0x01;
    RCGCGPIO |= 0x01; // enable GPIO Port A for UART0

    // set up GPIO Port A
    GPIO_A_AFSEL = 0x03;  // enable alternative function for PA0 and PA1
    GPIO_A_PCTL = (1 << 0) | (1 << 4);  // assign UART0 to PA0 and PA1
    GPIO_A_DEN = 0x03; // enable digital on PA0 and PA1

    // set up UART0
    UART0_CTL &= ~(1 << 0); // disable UART0 while configuring
    UART0_IBRD = ibrd; // set the integer part of the BRD
    UART0_FBRD = fbrd;  // set the fractional part of the BRD
    UART0_LCRH = (0x3 << 5); // set 8-bit wordlength (default: no parity, 1 stop bit)
    UART0_CC = 0x0; // select UART clock source as system clock

    // set up interrupt
    // UART0_IM = (1 << 4); // set UART0 receive interrupt mask
    // EN0 |= (1 << 5); // enable UART0 interrupt at NVIC

    UART0_CTL = (1 << 0) | (1 << 8) | (1 << 9); // enable UART0, enable receive and transmit section
}

char read_char() {
    while (UART0_FR & (1 << 4)); // wait if receiver is empty
    return (char) UART0_DR;
}

void print_char(char c) {
    while (UART0_FR & (1 << 5)); // wait if transmitter is full
    UART0_DR = c;
}

void print_string(char *str) {
    while (*str) {
        print_char(*str);
        str++;
    }
}

#endif
