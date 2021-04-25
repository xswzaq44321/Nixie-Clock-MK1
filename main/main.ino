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

void rotateCW(bool);

byte numberToDisplay[IC_COUNT];

#define FLAG_UPDATE (1 << 0)
#define FLAG_CLOCK  (1 << 1)
#define FLAG_LED    (1 << 2)
#define FLAG_DATE   (1 << 3)
#define FLAG_12HR   (1 << 4)
#define FLAG_ALM    (1 << 5)
#define FLAG_BUZZ   (1 << 6)
byte system_flags = 0b00000110;

/**************************************************************************/
/* PWM INTERRUPT */

float dutyCycle = 1;

void timer_setup()
{ 
  cli(); // close interrupt
  
  /* disable compare output mode for OCnA, OCnB, OCnC */
  TCCR1A = 0;
  TCCR1B = 0;
 
  TCCR1B |= (1 << CS10); // set prescaler to 1
  
  // timer overflow second formula: 65536 / (16,000,000 / prescaler)
  // formula: duty_cycle * 65536 
  OCR1A = dutyCycle * 65535;
  
  TCNT1 = 0; // reset timer counter
  
  // set interrupt mask for TIMER1_COMPA_vect and TIMER1_OVF_vect
  TIMSK1 = (1 << OCIE1A) | (1 << TOIE1);
  
  sei(); //open interrupt
}

ISR(TIMER1_COMPA_vect){
  emptyDisplay();
}

ISR(TIMER1_OVF_vect){
  pushData(numberToDisplay);
}
/**************************************************************************/



/**************************************************************************/
/* MODE */

enum SYSTEM_MODE {
  NORMAL,
  LIGHT,
  ALARM,
  TIMER,
  SETTIME,
  SYS_MODE_CNT
};

int mode = NORMAL;
/**************************************************************************/



/**************************************************************************/
/* SUB MODE */

enum SET_LIGHT_MODE {
  SET_NIXIE,
  SET_LED_H,
  SET_LED_S,
  SET_LED_V,
  SET_LIGHT_CNT
};

enum SET_ALM_MODE {
  SET_ALM_H,
  SET_ALM_M,
  SET_ALM_CNT
};

enum SET_TIME_MODE {
  SET_TIME_SEC,
  SET_TIME_HR,
  SET_TIME_MIN,
  SET_TIME_Y,
  SET_TIME_MON,
  SET_TIME_D,
  SET_TIME_CNT
};

int subMode = 0;
/**************************************************************************/



/**************************************************************************/
/* BUTTON INTERRUPT */

void button_setup() {
  /* set PB2,3,4,5,6 as input */
  DDRB &= ~(1 << PB2 | 1 << PB4 | 1 << PB5 | 1 << PB6 | 1 << PB1 | 1 << PB3);
  DDRE |= 1 << PE6;     // set PE6 as output
  PORTE &= ~(1 << PE6); // output low

  PCICR |= 1 << PCIE0;  // enable pin change interrupt 0
  PCIFR |= 1 << PCIF0;  // clear interrupt flag bit
  PCMSK0 |= 0b01111100; // enable PCINT2,3,4,5,6
}

int aLastState = 0;

ISR(PCINT0_vect){

  /* for rotary encoder */
  static const byte MASK_RT_A = (1 << PB3);
  static const byte MASK_RT_B = (1 << PB1);

  static byte buttonPressed = 0;

  int aState = !!(PINB & MASK_RT_A);

  if (aState != aLastState) {
    int bState = !!(PINB & MASK_RT_B);
    (bState != aState) ? rotateCW(true) : rotateCW(false);
  }
  
  aLastState = aState;



//  static byte buttonPressed = 0;
//
//  /* PINB stores which pin has input */
//  byte stateChange = buttonPressed ^ PINB;
//
//  int aState = !!(PINB & MASK_RT_A);
//
//  if (stateChange & MASK_RT_A) {
//    (((PINB >> PB3) ^ (PINB >> PB1)) & 1)?
//      rotateCW() : rotateCCW();
//  }
  


  /* for button */
  #define DEBOUNCE_DELAY 10
  
  static unsigned long TIME_BTN_MODE = 0;
  static unsigned long TIME_BTN_LIGHT = 0;
  static unsigned long TIME_BTN_START = 0;
  static unsigned long TIME_BTN_RESET = 0;

  static const byte MASK_BTN_LIGHT = 1 << PB4;
  static const byte MASK_BTN_MODE  = 1 << PB5;
  static const byte MASK_BTN_START = 1 << PB6;
  static const byte MASK_BTN_RESET = 1 << PB2;

  /* PINB stores which pin has input */
  byte stateChange = buttonPressed ^ PINB;

  /* button MODE */
  if ((stateChange & MASK_BTN_MODE) && millis() - TIME_BTN_MODE >= DEBOUNCE_DELAY) {
    buttonPressed ^= MASK_BTN_MODE;
    if (buttonPressed & MASK_BTN_MODE) {
      btn_MODE_press();
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

inline void rtc_setup() {
  rtc.begin();
  currentTime = rtc.now();

  /* need to call adjust() if RTC is used for the first time */
  rtc.adjust(DateTime(2021, 4, 23, 21, 45, 0));
}

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
/**************************************************************************/



/**************************************************************************/
/* LED */

// To disable pragma messages on compile
// include this Before including FastLED.h
#define FASTLED_INTERNAL
#include <FastLED.h>

#define LED_PIN 7
CRGB leds[IC_COUNT];

/*
 * formula for normal HSV mapping to CHSV
 * CH = H / 360 * 255
 * CS = S / 100 * 255
 * CV = V / 100 * 255
 */
CHSV ledColor(147, 227, 60);

inline void led_setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, IC_COUNT);
  fill_solid(leds, IC_COUNT, ledColor);
  FastLED.show();
}
/**************************************************************************/



/**************************************************************************/
/* BUTTON ACTIONS */

inline void btn_MODE_press() {
  ++mode %= SYS_MODE_CNT;
  subMode = 0;
  
  switch (mode) {
  case LIGHT:
    system_flags &= ~FLAG_CLOCK;
    memset(numberToDisplay, 0, IC_COUNT);
    break;

  case ALARM:
    if (system_flags & FLAG_LED)
      fill_solid(leds, IC_COUNT, ledColor);
    else
      fill_solid(leds, IC_COUNT, CRGB(0,0,0));
    FastLED.show();
    break;
  }
}

inline void btn_LIGHT_press() {
  switch (mode) {
  case NORMAL:
    system_flags ^= FLAG_LED;
    if (system_flags & FLAG_LED)
      fill_solid(leds, IC_COUNT, ledColor);
    else
      fill_solid(leds, IC_COUNT, CRGB(0,0,0));
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

inline void btn_RESET_press() {
  switch (mode) {
  case NORMAL:
    system_flags ^= FLAG_12HR;
    system_flags |= FLAG_UPDATE;
    break;

  case LIGHT:
    ++subMode %= SET_LIGHT_CNT;
    
    if (subMode == SET_NIXIE)
      system_flags &= ~FLAG_CLOCK;
    else
      system_flags |= FLAG_CLOCK;    
    break;
  }

  leds[0] = ledColor;
  FastLED.show();
}


inline void btn_START_release() {
  switch (mode) {
  case NORMAL:
    system_flags &= ~FLAG_DATE;
    system_flags |= FLAG_UPDATE;
    break;
  }
}
/**************************************************************************/



/**************************************************************************/
/* ROTARTY ENCODER ACTIONS */

inline void rotateCW(bool clockwise = true) {

  int sign = (clockwise)? 1 : -1;
  
  switch (mode) {

  /*** LIGHT MODE ***/
  case LIGHT:
    switch (subMode) {
   
    case SET_NIXIE:
      /*
       * Multiply dutyCycle(nixie tube's brightness) by 1.05 or 0.95.
       * Use multiply instead of add will look more instinctively for human.
       */
      dutyCycle *= (20 + sign) * 0.05;
      if (dutyCycle > 0.95) dutyCycle = 1;
      if (dutyCycle <= 0.05 ) dutyCycle = 0.05;
      OCR1A = dutyCycle * 65535;
      return;

    case SET_LED_H:
      ledColor.h += sign;
      break;

    case SET_LED_S:
      ledColor.s += 3 * sign;
      break;

    case SET_LED_V:
      ledColor.v += sign;
      break;
    }

    /*
     * FastLED.show() will be executed in circleLight(),
     * since circleLight() is repeatedly called in LIGHT mode.
     * Although executing show() here should only be a waste of time,
     * doing it here will froze the system for unknown reason.
     */
    fill_solid(leds, IC_COUNT, ledColor);
    break;
  } 
  
}
/**************************************************************************/



inline void circleLight() {
  if (mode != LIGHT) return;
  
  static byte circular = 0;
  circular += 10;
    
  switch (subMode) {
  case SET_NIXIE:
    SET_INTERVAL(500, [] {
      static byte cnt = 1;
      memset(numberToDisplay, cnt, IC_COUNT);
      ++cnt %= 10;
    });
    return;
  
  case SET_LED_H:
    leds[0] = CHSV(circular, ledColor.s, ledColor.v);
    break;

  case SET_LED_S:
    leds[0] = CHSV(ledColor.h, circular, ledColor.v);
    break;

  case SET_LED_V:
    leds[0] = CHSV(ledColor.h, ledColor.s, circular);
    break;
  }

  FastLED.show();
}

inline void updateClock() {
  if (!(system_flags & FLAG_CLOCK))
    return;
    
  if (system_flags & FLAG_DATE) {
    updateDate();
  }
  else {
    updateHMS(system_flags & FLAG_12HR);
  }
}

void setup() {
  Serial.begin(9600);
   
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
  SET_INTERVAL(100, [] {
    updateClock();
    if (mode == LIGHT)
      circleLight();
  });
}
