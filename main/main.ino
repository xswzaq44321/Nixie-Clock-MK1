#include <Wire.h>
#include "shift_register.h"

#define SET_INTERVAL(TIME, CALLBACK, ...)\
do {\
  static unsigned long t = millis();\
  if(millis() - t >= TIME){\
    CALLBACK(__VA_ARGS__);\
    t = millis();\
  }\
} while(0)

byte numberToDisplay[IC_COUNT];

enum MODE {
  NORMAL,
  LIGHT,
  ALARM,
  TIMER,
  SETTIME,
  CNT
};

int mode = NORMAL;
byte system_flags = 0b00000010;
#define FLAG_UPDATE (1 << 0)
#define FLAG_LED    (1 << 1)
#define FLAG_DATE   (1 << 2)
#define FLAG_12HR   (1 << 3)



/**************************************************************************/
/* PWM INTERRUPT */
void timer_setup()
{
  // close interrupt
  cli();
  
  // disable compare output mode for OCnA, OCnB, OCnC
  TCCR1A = 0;
  TCCR1B = 0;
//  // CTC mode, clear timer on compare
//  TCCR1B |= (1 << WGM12);
  // set prescaler to 1
  TCCR1B |= (1 << CS10);
  // timer overflow second formula: 65536 / (16,000,000 / prescaler)
  // formula: duty_cycle * 65536
  OCR1A = 1 * 65535;
  // reset timer counter
  TCNT1 = 0;
  // set interrupt mask for TIMER1_COMPA_vect and TIMER1_OVF_vect
  TIMSK1 = (1 << OCIE1A) | (1 << TOIE1);
  
  //open interrupt
  sei();
}

ISR(TIMER1_COMPA_vect){
  emptyDisplay();
}

ISR(TIMER1_OVF_vect){
  pushData(numberToDisplay);
}
/**************************************************************************/



/**************************************************************************/
/* BUTTON INTERRUPT */
void button_setup() {
  /* set PB2,4,5,6 as input */
  DDRB &= ~(1 << PB2 | 1 << PB4 | 1 << PB5 | 1 << PB6);
  DDRE |= 1 << PE6;     // set PE6 as output
  PORTE &= ~(1 << PE6); // output low

  PCICR |= 1 << PCIE0;  // enable pin change interrupt 0
  PCIFR |= 1 << PCIF0;  // clear interrupt flag bit
  PCMSK0 |= 0b01110100; // enable PCINT2,4,5,6
}

ISR(PCINT0_vect){
  #define DEBOUNCE_DELAY 10

  static const int PCINT0_Pins[] = {PB4, PB5, PB6, PB2};
  static const int buttonCount = sizeof(PCINT0_Pins) / sizeof(*PCINT0_Pins);  
  static byte buttonPressed = 0;

  static unsigned long TIME_BTN_MODE = 0;
  static unsigned long TIME_BTN_LIGHT = 0;
  static unsigned long TIME_BTN_START = 0;
  static unsigned long TIME_BTN_RESET = 0;

  static const byte MASK_BTN_MODE  = 1 << PB4;
  static const byte MASK_BTN_LIGHT = 1 << PB5;
  static const byte MASK_BTN_START = 1 << PB6;
  static const byte MASK_BTN_RESET = 1 << PB2;

  /* PINB stores which pin has input */
  byte stateChange = buttonPressed ^ PINB;

  /* button MODE */
  if ((stateChange & MASK_BTN_MODE) && millis() - TIME_BTN_MODE >= DEBOUNCE_DELAY) {
    buttonPressed ^= MASK_BTN_MODE;
    if (buttonPressed & MASK_BTN_MODE) {
      mode++;
    }
    else {
      // NOP
    }
    TIME_BTN_MODE = millis();
  }

  /* button LIGHT */
  if ((stateChange & MASK_BTN_LIGHT) && millis() - TIME_BTN_LIGHT >= DEBOUNCE_DELAY) {
    buttonPressed ^= MASK_BTN_LIGHT;
    if (buttonPressed & MASK_BTN_LIGHT) {
      btn_LIGHT_press();
    }
    else {
      // NOP
    }
    TIME_BTN_LIGHT = millis();
  }

  /* button START */
  if ((stateChange & MASK_BTN_START) && millis() - TIME_BTN_START >= DEBOUNCE_DELAY) {
    buttonPressed ^= MASK_BTN_START;
    if (buttonPressed & MASK_BTN_START) {
      btn_START_press();
    }
    else {
      btn_START_release();
    }
    TIME_BTN_START = millis();
  }

  /* button RESET */
  if ((stateChange & MASK_BTN_RESET) && millis() - TIME_BTN_RESET >= DEBOUNCE_DELAY) {
    buttonPressed ^= MASK_BTN_RESET;
    if (buttonPressed & MASK_BTN_RESET) {
      btn_RESET_press();
    }
    else {
      // NOP
    }
    TIME_BTN_RESET = millis();
  }

}
/**************************************************************************/



/**************************************************************************/
/* RTC */
#include "RTClib.h"

RTC_DS3231 rtc;
DateTime currentTime;

/*
 * Update [numberToDisplay] to current time (hr, min, sec) every second
 * or FLAG_UPDATE is on.
 * Will NOT push data to nixie tube.
 * Set PM to true to update to 12-hour clock.
 */
inline void updateHMS(bool PM = false) {
  DateTime tmp_t = rtc.now();
    
  if (system_flags & FLAG_UPDATE || tmp_t.second() != currentTime.second()) {
    currentTime = tmp_t;

    /* turn 24-hour clock to 12-hour clock */
    int tmp_hour = currentTime.hour();
    if (PM && tmp_hour > 12)
    tmp_hour -= 12;
    
    #ifdef IC_COUNT_8
    numberToDisplay[7] = tmp_hour / 10;
    numberToDisplay[6] = tmp_hour % 10;
    numberToDisplay[5] = NO_LIGHT;
    numberToDisplay[4] = currentTime.minute() / 10;
    numberToDisplay[3] = currentTime.minute() % 10;
    numberToDisplay[2] = NO_LIGHT;
    numberToDisplay[1] = currentTime.second() / 10;
    numberToDisplay[0] = currentTime.second() % 10;
    #else
    numberToDisplay[5] = tmp_hour / 10;
    numberToDisplay[4] = tmp_hour % 10;
    numberToDisplay[3] = currentTime.minute() / 10;
    numberToDisplay[2] = currentTime.minute() % 10;
    numberToDisplay[1] = currentTime.second() / 10;
    numberToDisplay[0] = currentTime.second() % 10;
    #endif
  }
}

/*
 * Update [numberToDisplay] to current date if the date changes
 * or FLAG_UPDATE is on.
 * Will NOT push data to nixie tube.
 */
inline void updateDate() {
  DateTime tmp_t = rtc.now();
  if (system_flags & FLAG_UPDATE || tmp_t.day() != currentTime.day()) {
    currentTime = tmp_t;
    
    #ifdef IC_COUNT_8
    int tmp_year = currentTime.year();
    for (int i = 4; i <= 7; ++i) {
      numberToDisplay[i] = tmp_year % 10;
      tmp_year /= 10;
    }
    numberToDisplay[3] = currentTime.month() / 10;
    numberToDisplay[2] = currentTime.month() % 10;
    numberToDisplay[1] = currentTime.day() / 10;
    numberToDisplay[0] = currentTime.day() % 10;
    #else
    int tmp_year = currentTime.year();
    for (int i = 4; i <= 5; ++i) {
      numberToDisplay[i] = tmp_year % 10;
      tmp_year /= 10;
    }
    numberToDisplay[3] = currentTime.month() / 10;
    numberToDisplay[2] = currentTime.month() % 10;
    numberToDisplay[1] = currentTime.day() / 10;
    numberToDisplay[0] = currentTime.day() % 10;
    #endif
  }
}

inline void rtc_setup() {
  rtc.begin();
  currentTime = rtc.now();

  /* need to call adjust() if RTC is used for the first time */
  rtc.adjust(DateTime(2021, 4, 23, 21, 45, 0));
}
/**************************************************************************/



/**************************************************************************/
/* LED */

// To disable pragma messages on compile
// include this Before including FastLED.h
#define FASTLED_INTERNAL
#include <FastLED.h>

#define LED_PIN 7
CRGB leds[IC_COUNT];
CHSV ledColor = CHSV(208.0 / 360 * 255, 89.0 / 100 * 255, 60);

inline void led_setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, IC_COUNT);
  fill_solid(leds, IC_COUNT, ledColor);
  FastLED.show();
}
/**************************************************************************/




/**************************************************************************/
/* BUTTON ACTIONS */

inline void btn_LIGHT_press() {
  switch (mode) {
  case NORMAL:
    system_flags ^= FLAG_LED;
    if (system_flags & FLAG_LED) {
      fill_solid(leds, IC_COUNT, ledColor);
    }
    else {
      fill_solid(leds, IC_COUNT, CRGB(0,0,0));
    }
    FastLED.show();
    break;
  }
}

inline void btn_START_press() {
  switch (mode) {
  case NORMAL:
    system_flags |= FLAG_DATE;
    system_flags |= FLAG_UPDATE;
    break;
  }
}

inline void btn_START_release() {
  switch (mode) {
  case NORMAL:
    system_flags &= ~FLAG_DATE;
    system_flags |= FLAG_UPDATE;
    break;
  }
}

inline void btn_RESET_press() {
  switch (mode) {
  case NORMAL:
    system_flags ^= FLAG_12HR;
    system_flags |= FLAG_UPDATE;
    break;
  }
}
/**************************************************************************/



inline void updateClock() {
  if (system_flags & FLAG_DATE) {
    updateDate();
  }
  else {
    updateHMS(system_flags & FLAG_12HR);
  }
}

void setup() {
  timer_setup();
  
  register_setup();
  rtc_setup();
  button_setup();
  led_setup();
}

void loop() {
  /* update clock instantly (when ISR is called) */
  if (system_flags & FLAG_UPDATE) {  
    updateClock();
    system_flags &= ~FLAG_UPDATE;
    return;
  }

  /* update clock every 100ms */
  SET_INTERVAL(100, updateClock);
}
