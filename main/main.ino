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

byte numberToDisplay[IC_COUNT];
RTC_DS3231 rtc;
DateTime currentTime;

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
  rtc.adjust(DateTime(2021, 1, 1, 3, 30, 25));
}

void setup() {
  timer_setup();
  
  register_setup();
  rtc_setup();
}

void loop() {
  SET_INTERVAL(100, updateTime);
}
