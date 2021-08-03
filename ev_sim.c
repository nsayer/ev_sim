/*

 EV Sim Remote
 Copyright 2019 Nicholas W. Sayer
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warran of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// Define this for v2.1 hardware
#define USE_AC

// CPU frequency - 16 MHz
#define F_CPU (16000000UL)

// Serial baud
#define BAUD 9600

#include <util/setbaud.h>

// Note that some versions of the AVR LIBC forgot to
// define the individual PUExn bit numbers.
#ifndef PUEA0
#define PUEA0 0
#define PUEA1 1
#define PUEA2 2
#define PUEA3 3
#define PUEA4 4
#define PUEA5 5
#define PUEA6 6
#define PUEA7 7
#define PUEB0 0
#define PUEB1 1
#define PUEB2 2
#define PUEB3 3
#endif

// R0 is the 2.7k resistor, R1 is the 1.3k and R2 is the 330 ohm one.
#define R0_PIN (0)
#ifdef USE_AC
#define R1_PIN (3)
#define R2_PIN (4)
#else
#define R1_PIN (2)
#define R2_PIN (3)
#endif

// How long (in ticks) do we sample?
#define SAMPLE_TICKS (1000UL)

// Where is the separation between the pilot being "high" and "low"?
#define ANALOG_STATE_TRANSITION_LEVEL 556

volatile uint8_t tx_buf[128];
volatile uint8_t tx_head, tx_tail;

volatile uint64_t ticks_cnt;

volatile uint8_t state;

ISR(TIMER0_COMPA_vect) {
  ticks_cnt++;
}

static inline uint64_t inline __attribute__ ((always_inline)) ticks() {
  uint64_t out;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    out = ticks_cnt;
  }
  return out;
}

// TX register empty interrupt handler
ISR(USART0_UDRE_vect) {
  if (tx_head == tx_tail) {
    // the transmit queue is empty.
    UCSR0B &= ~_BV(UDRIE0); // disable the TX interrupt
    return;
  }
  UDR0 = tx_buf[tx_tail];
  if (++tx_tail == sizeof(tx_buf)) tx_tail = 0; // point to the next char
}

static void tx_char(const char c) {
  int buf_in_use;
  do {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      buf_in_use = tx_head - tx_tail;
    }
    if (buf_in_use < 0) buf_in_use += sizeof(tx_buf);
    wdt_reset(); // we might be waiting a while.
  } while (buf_in_use >= sizeof(tx_buf) - 2) ; // wait for room in the transmit buffer

  tx_buf[tx_head] = c;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // this needs to be atomic, because an intermediate state is tx_head
    // pointing *beyond* the end of the buffer.
    if (++tx_head == sizeof(tx_buf)) tx_head = 0; // point to the next free spot in the tx buffer
  }
  UCSR0B |= _BV(UDRIE0); // enable the TX interrupt
}

static void tx_pstr(const char *buf) {
  for(int i = 0; i < strlen_P(buf); i++)
    tx_char(pgm_read_byte(&(buf[i])));
}

static void tx_str(const char *buf) {
  for(int i = 0; i < strlen(buf); i++)
    tx_char(buf[i]);
}

// RX interrupt
ISR(USART0_RX_vect) {
  uint8_t c = UDR0;
  c &= 0x7f; // remove parity
  switch(c) {
    case 'A':
    case 'a':
      state = 'A';
      PORTA = 0; // Disconnected
      break;
    case 'B':
    case 'b':
      state = 'B';
      PORTA = _BV(R0_PIN); // Vehicle present
      break;
    case 'C':
    case 'c':
      state = 'C';
      PORTA = _BV(R0_PIN) | _BV(R1_PIN); // Power requested
      break;
    case 'D':
    case 'd':
      state = 'D';
      PORTA = _BV(R0_PIN) | _BV(R1_PIN) | _BV(R2_PIN); // Ventilation requested
      break;
  }
}

volatile uint32_t hi_count, lo_count, state_changes;

#ifdef USE_AC  
ISR(ANA_COMP0_vect) {
  //ACSR0A |= _BV(ACI0); // clear the interrupt

  //uint8_t state = (ACSR0A & _BV(ACO0)) != 0;

  // we have hysteresis, so we _know_ this means we had a _real_ change.
  state_changes++;
}
#endif

void __ATTR_NORETURN__ main() {

  wdt_enable(WDTO_1S);

  // We use USART0, Timer 0 and the ADC.
  PRR = _BV(PRTWI) | _BV(PRUSART1) | _BV(PRSPI) | _BV(PRTIM2) | _BV(PRTIM1);

  PORTA = 0; // turn all outputs off
  DDRB = 0; // RX pin is input
  DDRA = _BV(R0_PIN) | _BV(R1_PIN) | _BV(R2_PIN) | _BV(7); // mode pins and tx output
#ifdef USE_AC
  PUEA = _BV(PUEA5) | _BV(PUEA6); // pull-ups for the unused pins on port A.
#else
  PUEA = _BV(PUEA4) | _BV(PUEA5) | _BV(PUEA6); // pull-ups for the unused pins on port A.
#endif

  ADMUXA = _BV(MUX0); // input 1, single-ended
  ADMUXB = 0; // Vcc AREF, gain 1
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1); // clock /64, enable
  ADCSRB = 0;
#ifdef USE_AC
  DIDR0 = _BV(ADC1D) | _BV(ADC2D); // AIN00 / ADC1 & AIN01 is analog
#else
  DIDR0 = _BV(ADC1D); // AIN00 / ADC1 is analog
#endif

#ifdef USE_AC
  ACSR0A = _BV(ACIE0); // positive input is AIN00, enable interrupts
  ACSR0B = _BV(HSEL0) | _BV(HLEV0); // Negative input is AIN01, high hysteresis
#else
  ACSR0A = _BV(ACD0); // disable AC
#endif

  UCSR0A = 0;
  UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0); // Interrupt on receive
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8N1
  UCSR0D = 0;
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;
  #if USE_2X
  UCSR0A |= _BV(U2X0);
  #else
  UCSR0A &= ~_BV(U2X0);
  #endif
  REMAP = _BV(U0MAP); // we want TX and RX on PA7 and PB2.

  // Set up timer 0 as a millisecond tick source
  TCCR0A = _BV(WGM01); // CTC mode
  TCCR0B = _BV(CS01) | _BV(CS00); // divide by 64.
  TIMSK0 = _BV(OCIE0A); // interrupt on compare A
  OCR0A = 249; // 16 MHz divided by (64 * 250) = 1000
  
  tx_head = tx_tail = 0; // point to the start of the buffer.

  state = 'A';

  sei(); // release the hounds!

  while(1) {

    uint16_t high_analog = 0;
    uint16_t low_analog = 0xffff;

#ifdef USE_AC
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      state_changes = 0;
    }
#endif
#ifndef USE_AC
    uint32_t last_state = 99;
#endif
    uint32_t hi_save = 0, lo_save = 0, changes_save = -1;
    for(uint64_t now = ticks(); ticks() - now < SAMPLE_TICKS; ) {
      wdt_reset(); // pet the dog

      // read the ADC.
      ADCSRA |= _BV(ADIF); // clear the complete flag
      ADCSRA |= _BV(ADSC); // start conversion
      while(!(ADCSRA & _BV(ADIF))) ; // wait for it
      uint16_t analog = ADC;
      ADCSRA |= _BV(ADIF); // clear the complete flag for good measure

      if (analog > high_analog) high_analog = analog;
      if (analog < low_analog) low_analog = analog;

      int state = analog > ANALOG_STATE_TRANSITION_LEVEL;
      if (state)
	hi_save++;
      else
        lo_save++;
#ifndef USE_AC
      if (state != last_state) {
	changes_save++;
        last_state = state;
      }
#endif

    }

#ifdef USE_AC
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      changes_save = state_changes;
    }
#endif
    char pbuf[20];
    tx_pstr(PSTR("{ state: \""));
    snprintf_P(pbuf, sizeof(pbuf), PSTR("%c"), state);
    tx_str(pbuf);
    tx_pstr(PSTR("\", state_changes: "));
    snprintf_P(pbuf, sizeof(pbuf), PSTR("%d"), changes_save);
    tx_str(pbuf);
    tx_pstr(PSTR(", low_count: "));
    snprintf_P(pbuf, sizeof(pbuf), PSTR("%d"), lo_save);
    tx_str(pbuf);
    tx_pstr(PSTR(", high_count: "));
    snprintf_P(pbuf, sizeof(pbuf), PSTR("%d"), hi_save);
    tx_str(pbuf);
    tx_pstr(PSTR(", low_adc: "));
    snprintf_P(pbuf, sizeof(pbuf), PSTR("%d"), low_analog);
    tx_str(pbuf);
    tx_pstr(PSTR(", high_adc: "));
    snprintf_P(pbuf, sizeof(pbuf), PSTR("%d"), high_analog);
    tx_str(pbuf);
    tx_pstr(PSTR(" }\r\n"));
    
  }
  __builtin_unreachable();
}
