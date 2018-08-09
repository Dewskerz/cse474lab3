#ifndef SYS_CLOCK_H
#define SYS_CLOCK_H

#define RCC         *((volatile unsigned long *) 0x400FE060)
#define RCC2        *((volatile unsigned long *) 0x400FE070)
#define RIS         *((volatile unsigned long *) 0x400FE050)

// RCC pins
#define XTAL        6

// RCC2 pins
#define OSCSRC2     4
#define BYPASS2     11
#define PWRDN2      13
#define SYSDIV2LSB  22
#define DIV400      30
#define USERCC2     31

void sysclock_setup(int sysdiv) {
  RCC = (RCC &~(0x7C << 4)) + (0x54 << 4); // clear XTAL field and configure 16MHz crystal
  RCC2 |= (1U << 31); // RCC2 override the RCC
  RCC2 |= (1 << 11); // set bypass2 while initializing

  RCC2 &= ~(0x7 << 4); // use the main oscillator
  RCC2 |= (0x4 << 28); // use 400MHz PLL
  RCC2 &= ~(0x02 << 12); // activate PLL
  RCC2 = (RCC2 &~(0x1FC << 20)) + (sysdiv << 22);
  while(RIS & (4 << 4) == 0){} // wait for PLLRIS bit
  RCC2 &= ~(8 << 8); // clear bypass2 to use PLL
}

#endif