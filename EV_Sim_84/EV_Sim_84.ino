/*

 J1772 EV Pilot Analysis sketch for Arduino
 Copyright 2013 Nicholas W. Sayer
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <Arduino.h>
#include <util/atomic.h>

#define VERSION "(84) 2.0"

// Hardware versions prior to 4.0 use DIGITAL_SAMPLING. 4.0 and beyond use
// analog.
//
// You are allowed to define both. If you do, then digital sampling is used
// to obtain the duty cycle, while analog sampling is used for the high/low
// voltage measurement. It turns out, however, that both are interleaved,
// so the speed advantage of digital sampling is lost.
//
// A workaround for that would be to replace analogRead() with manual ADC control,
// and performing the digital sampling while waiting for the ADC to complete.
//
// It turns out, however, that analog sampling is good enough for our purposes.
//#define DIGITAL_SAMPLING
#define ANALOG_SAMPLING

// Define this for hardware 4.1 This requires ANALOG_SAMPLING
#define USE_AC

#if !defined(ANALOG_SAMPLING) && defined(USE_AC)
#error USE_AC requires ANALOG_SAMPLING
#endif

#if !defined(DIGITAL_SAMPLING) && !defined(ANALOG_SAMPLING)
#error Well, you have to let me do it SOMEHOW...
#endif

#ifdef DIGITAL_SAMPLING
#define PILOT_DIGITAL_SAMPLING_PIN  0
#else
#ifndef USE_AC
// Since we're not going to use digital sampling,
// we have to decide what constitutes a high and
// what constitutes a low for the purposes of the
// square wave sampling. This happens to be the 0 volt
// level.
#define ANALOG_STATE_TRANSITION_LEVEL 556
#endif
#endif

#ifdef ANALOG_SAMPLING
#ifdef USE_AC
#define PILOT_ANALOG_SAMPLING_PIN 1
#else
#define PILOT_ANALOG_SAMPLING_PIN 2
#endif
#endif

// The pins connected up to the LCD.
#define LCD_D4 7
#define LCD_D5 6
#define LCD_D6 5
#define LCD_D7 4
#define LCD_RS 3
#ifdef USE_AC
#define LCD_E 0
#else
#define LCD_E 1
#endif

#define BUTTON_PIN 8

#define SAMPLE_PERIOD 500

#ifdef ANALOG_SAMPLING
// The button selects from different available display modes.
// At the moment, there are two: J1772 ampacity and min/max voltage
#define MODE_COUNT 2
#else
// There's no analog display mode without analog sampling
#define MODE_COUNT 1
#endif

// The different types of button events.
#define BUTTON_NONE 0
#define BUTTON_SHORT 1
#define BUTTON_LONG 2

// When the button state changes, further state changes are ignored for this many ms.
#define DEBOUNCE_INTERVAL 50
// The boundary between a BUTTON_SHORT and a BUTTON_LONG in ms.
#define LONG_PRESS_INTERVAL 250

// The scale and offset for the linear formula that relates A/D reading to pilot voltage.
// A circuit simulation shows the readback voltage as 4.543 at 12 volts in and 891 mV at -12 volts.
// Since a single A/D unit represents 4.88 mV, we can work out a formula to convert A/D readings
// into millivolts of the actual pilot signal
#ifdef ANALOG_SAMPLING
#define PILOT_READ_SCALE (6570L)
#define PILOT_READ_OFFSET (2500)
#endif

#ifdef USE_AC
// We sometimes get way-too-short intervals. If the AC returns an interval shorter than this, 
// then it's just noise and we should ignore it.
// The timer is running at 16 MHz, so this is a microsecond, or a tenth of a percent duty cycle.
// The shortest legal duty cycle is 5%.
#define NOISE_REDUCTION (16)
#endif

#include <LiquidCrystal.h>

// Keep strings in PROGMEM. Copy them into this temp buffer just-in-time.
char p_buffer[96];
#define P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)

LiquidCrystal display(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// special return result for the 'digital communications' pilot.
#define DIGITAL_COMM_REQD 999999
// duty is tenths-of-a-percent (that is, fraction out of 1000).
static inline unsigned long dutyToMA(unsigned long duty) {
  // Cribbed from the spec - grant a +/-2% slop
  if (duty < 30) {
    // < 3% is an error
    return 0;
  }
  else if (duty <= 70) {
    // 3-7% - digital
    return DIGITAL_COMM_REQD;
  }
  else if (duty < 80) {
    // 7-8% is an error
    return 0;
  }
  else if (duty <= 100) {
    // 8-10% is 6A
    return 6000;
  }
  else if (duty <= 850) { // 10-85% uses the "low" function
    return duty * 60;
  } 
  else if (duty <= 960) { // 85-96% uses the "high" function
    return (duty - 640) * 250;
  }
  else if (duty <= 970) {
    // 96-97% is 80A
    return 80000;
  }
  else { // > 97% is an error
    return 0;
  }
}

static inline char read_button() {
  static unsigned char last_state = HIGH; // HIGH is unpressed, LOW is pressed
  static unsigned long debounce_time = 0; // ignore events until this time
  static unsigned long button_press_start = 0;
  if (debounce_time != 0) {
    if (millis() > debounce_time) {
      debounce_time = 0; // debounce period is over
    } else {
      return BUTTON_NONE;
    }
  }
  unsigned char state = digitalRead(BUTTON_PIN);
  if (state == last_state) return BUTTON_NONE;
  last_state = state;
  debounce_time = millis() + DEBOUNCE_INTERVAL;
  switch(state) {
    case LOW:
      button_press_start = millis();
      return BUTTON_NONE;
      break;
    case HIGH:
      unsigned char out = (millis() - button_press_start > LONG_PRESS_INTERVAL)?BUTTON_LONG:BUTTON_SHORT;
      button_press_start = 0;
      return out;
      break;
  }
}

#ifdef USE_AC
volatile unsigned int state_changes, hi_period, lo_period, last_capture;
ISR(ANA_COMP_vect) {
  unsigned int capture = ICR1;
  unsigned int delta = capture - last_capture;
  if (delta < NOISE_REDUCTION) return; // this interval is too short. Ignore it.
  last_capture = capture;
  
  int state = (ACSR & _BV(ACO)) != 0;

  if (state) {
    lo_period = delta; // if it's high, we just captured the low period
    TCCR1B &= ~_BV(ICES1); // now look for the falling edge
  } else {
    hi_period = delta;
    TCCR1B |= _BV(ICES1);
  }

  state_changes++;
}
#endif

#ifdef ANALOG_SAMPLING
// Multiply this value by all ADC readings to properly scale them
uint16_t vcc_mv;

void calibrate_adc() {
  ADMUX = _BV(MUX5) | _BV(MUX0); // select the internal 1.1v ref and Vcc ref
  delay(2); // wait 2 ms

  ADCSRA |= _BV(ADSC); // start conversion
  while(ADCSRA & _BV(ADSC)) ; // wait for it
  uint16_t adc = ADC;

  vcc_mv = (uint16_t) ((1100L * 1023L) / adc);
  
  ADMUX = _BV(MUX0); // put the mux back the way we found it
}

static inline long scale_mv(unsigned int value) {
  int mv = (int)((((long)value) * vcc_mv) / 1024L);
  mv -= PILOT_READ_OFFSET;
  return (mv * PILOT_READ_SCALE) / 1000L;
}
#endif

void setup() {
#ifdef DIGITAL_SAMPLING
  pinMode(PILOT_DIGITAL_SAMPLING_PIN, INPUT_PULLUP);
#endif
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#ifdef ANALOG_SAMPLING
  pinMode(PILOT_ANALOG_SAMPLING_PIN, INPUT);
  analogReference(DEFAULT);
#ifdef USE_AC
  DIDR0 = _BV(ADC1D) | _BV(ADC2D); // Turn off AIN0 & AIN1
#else
  DIDR0 = _BV(ADC1D); // Turn off the digital input on the analog pin
#endif
#endif

  // crank up the ADC clock.
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);

#ifdef USE_AC
  ACSR = _BV(ACIE) | _BV(ACIC); // enable analog comparator interrupts, use AC for input capture
  TCCR1A = 0;
  TCCR1B = _BV(CS11); // prescale by 1
#endif
    
  display.begin(16, 2);
  
  display.clear();
  
  display.print(P("EV Sim "));
  display.print(P(VERSION));
  
  delay(2000);
  display.clear();
}

void loop() {
  static unsigned char mode = 0;
  unsigned int last_state = 99; // neither HIGH nor LOW
  unsigned long hi_save = 0, lo_save = 0, changes_save = -1; // ignore the first change from "invalid"
#ifdef ANALOG_SAMPLING
  unsigned int high_analog=0, low_analog=0xffff;
  static unsigned char cal_delay = 0;
  if (cal_delay++ == 0) calibrate_adc();
  if (cal_delay >= 30) cal_delay = 0; // calibrate every 30 sample periods.
#endif

#ifdef USE_AC
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    state_changes = 0;
  }
#endif

  for(unsigned long start_poll = millis(); millis() - start_poll < SAMPLE_PERIOD; ) {
    switch(read_button()) {
      case BUTTON_NONE:
        break;
      case BUTTON_SHORT:
      case BUTTON_LONG:
        if (++mode >= MODE_COUNT) mode = 0;
        break;
    }

#ifdef ANALOG_SAMPLING    
    unsigned int analog = analogRead(PILOT_ANALOG_SAMPLING_PIN);
    if (analog > high_analog) high_analog = analog;
    if (analog < low_analog) low_analog = analog;
#endif

#ifndef USE_AC
    unsigned int state;
#ifdef DIGITAL_SAMPLING    
    state = digitalRead(PILOT_DIGITAL_SAMPLING_PIN);
#else
    state = (analog < ANALOG_STATE_TRANSITION_LEVEL)?LOW:HIGH;
#endif

    if (state == LOW)    
      lo_save++;
    else
      hi_save++;
      
      if (state != last_state) {
        changes_save++;
        last_state = state;
      }
#endif      
  }

#ifdef USE_AC
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    changes_save = state_changes;
    hi_save = hi_period;
    lo_save = lo_period;
  }
#endif

  char buf[32];
  unsigned long amps = 0;
  if (changes_save == 0) {
    sprintf(buf, P("   0 Hz     %s   "), 
#ifdef USE_AC
    ((ACSR & _BV(ACO)) == 0) // Actually ask the comparator. We can't count on the duty cycle system
#else
    (lo_save > hi_save)
#endif
    ?"-":"+");
  } else {
    unsigned int duty = (hi_save * 1000) / (hi_save + lo_save);
    duty %= 1000; // turn 100% into 0% just for display purposes. A 100% duty cycle doesn't really make sense.
  
    unsigned long frequency = ((changes_save / 2) * 1000) / SAMPLE_PERIOD;

    amps = dutyToMA(duty);

    sprintf(buf, P("%4ld Hz   %2d.%01d %%"), frequency, duty / 10, duty % 10);
  }
  display.setCursor(0, 0);
  display.print(buf);

  switch(mode) {
    case 0:
      if (amps == DIGITAL_COMM_REQD) {
        sprintf(buf, P("Digital         "));
      } else {
        sprintf(buf, P("%2ld.%02ld A         "), amps / 1000, (amps % 1000) / 10);
      }
      break;
#ifdef ANALOG_SAMPLING
    case 1:
      long low_mv = scale_mv(low_analog);
      long high_mv = scale_mv(high_analog);
      sprintf(buf,P(" %+03ld.%01ld  %+03ld.%01ld"), low_mv/1000, abs(low_mv % 1000) / 100, high_mv/1000, abs(high_mv % 1000) / 100);
      break;
#endif
  }
  
  display.setCursor(0, 1);
  display.print(buf);
}
