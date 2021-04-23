#include <Wire.h>
#include "RTClib.h"
#include "ShiftRegister.h"

#define SET_INTERVAL(TIME, CALLBACK, ...)\
do {\
  static unsigned long t = millis();\
  if(millis() - t >= TIME){\
    CALLBACK(__VA_ARGS__);\
    t = millis();\
  }\
} while(0)

enum MODE {
  NORMAL,
  LIGHT,
  ALARM,
  TIMER,
  SETTIME,
  CNT
};

int mode = NORMAL;
byte numberToDisplay[IC_COUNT];
RTC_DS3231 rtc;
DateTime currentTime;

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
  fastShiftOutArr(numberToDisplay);
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
      // TODO
    }
    else {
      // TODO
    }
    TIME_BTN_LIGHT = millis();
  }

  /* button START */
  if ((stateChange & MASK_BTN_START) && millis() - TIME_BTN_START >= DEBOUNCE_DELAY) {
    buttonPressed ^= MASK_BTN_START;
    if (buttonPressed & MASK_BTN_START) {
      // TODO
    }
    else {
      // TODO
    }
    TIME_BTN_START = millis();
  }

  /* button RESET */
  if ((stateChange & MASK_BTN_RESET) && millis() - TIME_BTN_RESET >= DEBOUNCE_DELAY) {
    buttonPressed ^= MASK_BTN_RESET;
    if (buttonPressed & MASK_BTN_RESET) {
      // TODO
    }
    else {
      // TODO
    }
    TIME_BTN_RESET = millis();
  }

}
/**************************************************************************/



/*
 * update [numberToDisplay] every second
 * will NOT push data to nixie tube
 */
inline void updateTime() {
  DateTime tmp_t = rtc.now();
  if (tmp_t.second() != currentTime.second()) {
    currentTime = tmp_t;
    
    #ifdef IC_COUNT_8
    numberToDisplay[7] = currentTime.hour() / 10;
    numberToDisplay[6] = currentTime.hour() % 10;
    numberToDisplay[5] = NO_LIGHT;
    numberToDisplay[4] = currentTime.minute() / 10;
    numberToDisplay[3] = currentTime.minute() % 10;
    numberToDisplay[2] = NO_LIGHT;
    numberToDisplay[1] = currentTime.second() / 10;
    numberToDisplay[0] = currentTime.second() % 10;
    #else
    numberToDisplay[5] = currentTime.hour() / 10;
    numberToDisplay[4] = currentTime.hour() % 10;
    numberToDisplay[3] = currentTime.minute() / 10;
    numberToDisplay[2] = currentTime.minute() % 10;
    numberToDisplay[1] = currentTime.second() / 10;
    numberToDisplay[0] = currentTime.second() % 10;
    #endif
  }
}

inline void rtc_setup() {
  rtc.begin();
  currentTime = rtc.now();

  /* need to call adjust() if RTC is used for the first time */
  // rtc.adjust(DateTime(2021, 1, 1, 3, 30, 25));
}

void setup() {
  Serial.begin(9600);
  
  timer_setup();
  
  register_setup();
  rtc_setup();
  button_setup();
}

void loop() {
  SET_INTERVAL(100, updateTime);
}
