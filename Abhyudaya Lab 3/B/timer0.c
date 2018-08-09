
// This file is the implementation of timer0.h

#include "timer0.h"

void timer0_setup(uint32_t speed, int interrupt) {
  RCGCTIMER |= 0x001;
  T0_CTL = TAEN_OFF;    // disable timer while configuring
  T0_CFG = 0x00;        // select 32-bit timer configuration
  T0_TAMR = 0x02;       // select countdown, periodic mode
  T0_TAILR = speed;     // set speed
  if (interrupt) {
    T0_MIMR = 0x01;     // enable interrupt mask
    EN0 |= (1 << 19);   // enable timer0 at NVIC
    T0_ICR |= 0x11;     // clear timer
  }
}

void timer0_ctrl(int on) {
  if (on) {
    T0_CTL = (TAEN_ON | TAOTE);
  }
}

int timer0_out() {
  return T0_RIS & 0x01;
}

void timer0_clear() {
  T0_ICR |= 0x11;
}
