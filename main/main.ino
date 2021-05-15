#include <Wire.h>
#include "shift_register.h"
#include "buzzer.h"

void rotateCW(bool);
void addTimeOnDisplay(int);
void adjustTime(int);
byte daysOfMonth(byte mon, int y);

byte rollNumber[IC_COUNT] = {0};

/**************************************************************************/
/* SYSTEM FLAGS */

#define FLAG_LED  (1 << 0)
#define FLAG_12HR (1 << 1)
#define FLAG_ALM  (1 << 2)
#define FLAG_TMR  (1 << 3)
byte settings = 0b0001;

#define MD_UPDATE (1 << 0)
#define MD_CLOCK  (1 << 1)
#define MD_DATE   (1 << 2)
#define MD_BUZZ   (1 << 3)
#define MD_SYNC   (1 << 4)
#define MD_ROLL   (1 << 5)
byte system_mode = 0b00000011;
/**************************************************************************/



/**************************************************************************/
/* EEPROM */
#include <EEPROMWearLevel.h>

#define EEPROM_LAYOUT_VERSION 0

#define ADDR_ALARM_H  0
#define ADDR_ALARM_M  1
#define ADDR_LED_H    2
#define ADDR_LED_S    3
#define ADDR_LED_V    4
#define ADDR_SETTING  5
#define ADDR_OCR1A_H  6
#define ADDR_OCR1A_L  7

#define AMOUNT_OF_INDEXES 8
/**************************************************************************/



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
  
  /*
   * timer overflow second formula: 65536 / (16,000,000 / prescaler)
   * formula: duty_cycle * 65536
   */
  OCR1A = dutyCycle * 65535;

  TCNT1 = 0; // reset timer counter

  // set interrupt mask for TIMER1_COMPA_vect and TIMER1_OVF_vect
  TIMSK1 = (1 << OCIE1A) | (1 << TOIE1);

  sei(); //open interrupt
}

ISR(TIMER1_COMPA_vect) {
  if (dutyCycle == 1 || system_mode & MD_ROLL)
    return;
  
  emptyDisplay();
}

ISR(TIMER1_OVF_vect) {
  #define ROLL_DELAY 10
  #define ROLL_STOP  90
  static_assert(ROLL_STOP / ROLL_DELAY >= 9,
                  "ROLL_STOP must be at least 9 times of ROLL_DELAY");

  static byte roll_cnt = 0;

  if (system_mode & MD_ROLL) {
    if (roll_cnt <= ROLL_STOP) {
      if (roll_cnt++ % ROLL_DELAY != 0)
        return;
      for (int i = 0; i < IC_COUNT; ++i)
        rollNumber[i] = (rollNumber[i] + 3) % 10;
      pushData(rollNumber);
    }
    else {
      roll_cnt = 0;
      system_mode &= ~MD_ROLL;
    }
    return;
  }

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

#define LED_ALARM A0
#define LED_TIMER A1
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
  SET_ALM_ON,
  SET_ALM_M,
  SET_ALM_H,
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

ISR(PCINT0_vect) {

  /* rotary encoder */
  static bool aLastState = false;
  bool aState = !!(PINB & (1 << PB3));
  if (aState != aLastState) {
    bool bState = !!(PINB & (1 << PB1));
    rotateCW(bState != aState);
  }
  aLastState = aState;

  /* buttons */
  #define DEBOUNCE_DELAY 10

  static const byte MASK_BTN_LIGHT = 1 << PB4;
  static const byte MASK_BTN_MODE  = 1 << PB5;
  static const byte MASK_BTN_START = 1 << PB6;
  static const byte MASK_BTN_RESET = 1 << PB2;

  /* PINB stores which pin has input */
  static byte buttonPressed = 0;
  byte stateChange = buttonPressed ^ PINB;

  /* button MODE */
  if (stateChange & MASK_BTN_MODE) {
    SET_INTERVAL(DEBOUNCE_DELAY, [] {
      buttonPressed ^= MASK_BTN_MODE;
      if (buttonPressed & MASK_BTN_MODE) {
        btn_MODE_press();
      }
      else {
        // NOP
    }
    });
  }

  /* button LIGHT */
  if (stateChange & MASK_BTN_LIGHT) {
    SET_INTERVAL(DEBOUNCE_DELAY, [] {
      buttonPressed ^= MASK_BTN_LIGHT;
      if (buttonPressed & MASK_BTN_LIGHT) {
        btn_LIGHT_press();
      }
      else {
        // NOP
      }
    });
  }

  /* button START */
  if (stateChange & MASK_BTN_START) {
    SET_INTERVAL(DEBOUNCE_DELAY, [] {
      buttonPressed ^= MASK_BTN_START;
      if (buttonPressed & MASK_BTN_START) {
        btn_START_press();
      }
      else {
        btn_START_release();
      }
    });
  }

  /* button RESET */
  if (stateChange & MASK_BTN_RESET) {
    SET_INTERVAL(DEBOUNCE_DELAY, [] {
      buttonPressed ^= MASK_BTN_RESET;
      if (buttonPressed & MASK_BTN_RESET) {
        btn_RESET_press();
      }
      else {
        // NOP
      }
    });
  }
  
}
/**************************************************************************/



/**************************************************************************/
/* TIME */
#include "RTClib.h"

RTC_DS3231 rtc;
DateTime currentTime;
byte alarm_h = 22;
byte alarm_m = 30;

byte adjust_hr  = 0;
byte adjust_min = 0;
uint16_t  adjust_y   = 0;
byte adjust_mon = 0;
byte adjust_d   = 0;
byte adjust_s   = 0;

inline void rtc_setup() {
  rtc.begin();

  currentTime = rtc.now();
  rtc.clearAlarm(1);
}

byte inline daysOfMonth(byte mon, int y) {
  switch (mon) {
  case 1: case 3: case 5: case 7: case 8: case 10: case 12:
    return 31;
    
  case 4: case 6: case 9: case 11:
    return 30;
    
  case 2:
    if ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)
      return 29;
    else
      return 28;
      
  default:
    return -1;
  }
}

/*
   Update [numberToDisplay] to current time(hr, min, sec) + adjusted every second
   or MD_UPDATE is on.
   Will NOT push data to nixie tube.
   Set PM to true to update to 12-hour clock.
*/
inline void updateHMS() {
  DateTime tmp_t = rtc.now();

  if (system_mode & MD_UPDATE)
    goto UPDATE;

  if (tmp_t.second() != currentTime.second()) {

    if (system_mode & MD_SYNC) {
      system_mode &= ~MD_SYNC;
      currentTime = DateTime(
                      currentTime.year() + adjust_y,
                      (currentTime.month() + adjust_mon - 1) % 12 + 1,
                      (currentTime.day() + adjust_d - 1) % 31 + 1,
                      (currentTime.hour() + adjust_hr) % 24,
                      (currentTime.minute() + adjust_min) % 60,
                      adjust_s
                    );
      rtc.adjust(currentTime);
      adjust_s = adjust_y = adjust_mon = adjust_d = adjust_hr = adjust_min = 0; 
    }
    else {
      UPDATE:
      currentTime = tmp_t;
    } 

    byte tmp_hr = currentTime.hour() + adjust_hr;
    tmp_hr %= (settings & FLAG_12HR)? 12 : 24;
    
    byte tmp_min = (currentTime.minute() + adjust_min) % 60;

    SET_HR(tmp_hr);
    SET_MIN(tmp_min);
    SET_SEC(currentTime.second());
  }
}

/*
   Update [numberToDisplay] to current date if the date changes
   or MD_UPDATE is on.
   Will NOT push data to nixie tube.
*/
inline void updateDate() {
  DateTime tmp_t = rtc.now();
  if (system_mode & MD_UPDATE || tmp_t.day() != currentTime.day()) {
    currentTime = tmp_t;

    uint16_t tmp_y = currentTime.year() + adjust_y;
    byte tmp_mon = (currentTime.month() + adjust_mon - 1) % 12 + 1;
    byte tmp_d = (currentTime.day() + adjust_d  - 1) % 31 + 1;

    SET_YEAR(tmp_y);
    SET_MONTH(tmp_mon);
    SET_DAY(tmp_d);
  }
}

/*
 * addTimeOnDisplay only adjust the time in numberToDisplay.
 * It does NOT adjust time in RTC.
 */
void addTimeOnDisplay(int t) {   
  switch (subMode) {
    
  case SET_ALM_M:
    alarm_m = (alarm_m + 60 + t) % 60;
    SET_MIN(alarm_m);
    break;
    
  case SET_ALM_H:
    alarm_h = (alarm_h + 24 + t) % 24;
    SET_HR(alarm_h);
    break;   
  }
}


/*
 * adjustTime only changes time adjustment(adjust_hr, adjust_min, etc.)
 * It does NOT adjust time in RTC, nor does it changes currentTime.
 * Time adjustment will be added with RTC.now() to get a new time.
 * Set MD_SYNC flag to write the new time into RTC and currentTime.
 */
void adjustTime(int t) {
  system_mode |= MD_UPDATE;
  
  switch (subMode) {
    
  case SET_TIME_HR:
    adjust_hr = (adjust_hr + 24 + t) % 24;
    break;
    
  case SET_TIME_MIN:
    adjust_min = (adjust_min + 60 + t) % 60;
    break;

  case SET_TIME_Y:
    adjust_y += t;
    break;

  case SET_TIME_MON:
    adjust_mon = (adjust_mon + 12 + t) % 12;
    break;

  case SET_TIME_D:
    adjust_d = (adjust_d + 31 + t) % 31;
    break;
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
   formula for normal HSV mapping to CHSV
   CH = H / 360 * 255
   CS = S / 100 * 255
   CV = V / 100 * 255
*/
CHSV ledColor(147, 227, 60);

inline void led_setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, IC_COUNT);

  pinMode(LED_ALARM, OUTPUT);
  pinMode(LED_TIMER, OUTPUT);
}
/**************************************************************************/



/**************************************************************************/
/* MODE INDICATOR */

inline void md_LIGHT_indicate() {
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

#define PERIOD    700 // period of nixie tube flickering
#define OFF_TIME  200 // time of nixie tube being off in a period
static_assert(PERIOD > OFF_TIME,
  "PERIOD should be greater then OFF_TIME");

inline void md_ALARM_indicate() {
  
  if (subMode == SET_ALM_ON)
    return;

  static unsigned long waitStart = millis() + PERIOD;

  /* nixie tube off */
  SET_INTERVAL(PERIOD, [] {
    switch (subMode) {
    case SET_ALM_H:   CLR_HR();   break;
    case SET_ALM_M:   CLR_MIN();  break;
    }
    waitStart = millis();
  });

  /* nixie tube on */
  if (millis() - waitStart >= OFF_TIME) {   
    switch (subMode) {
    case SET_ALM_H: SET_HR(alarm_h);  break;
    case SET_ALM_M: SET_MIN(alarm_m); break;
    }
  }
}

inline void md_SETTIME_indicate() {
  static unsigned long waitStart = millis() + PERIOD;

  /* nixie tube off */
  SET_INTERVAL(PERIOD, [] {
    system_mode &= ~MD_CLOCK;    
    switch (subMode) {
    case SET_TIME_SEC:  CLR_SEC();    break;
    case SET_TIME_HR:   CLR_HR();     break;
    case SET_TIME_MIN:  CLR_MIN();    break;
    case SET_TIME_Y:    CLR_YEAR();   break;
    case SET_TIME_MON:  CLR_MONTH();  break;
    case SET_TIME_D:    CLR_DAY();    break;
    }
    waitStart = millis();
  });

  /* nixie tube on */
  if (millis() - waitStart >= OFF_TIME) {
    system_mode |= MD_CLOCK;
    system_mode |= MD_UPDATE;
  }
}
/**************************************************************************/



/**************************************************************************/
/* BUTTON ACTIONS */

inline void btn_MODE_press() {
  if (system_mode & MD_BUZZ) {
    system_mode &= ~MD_BUZZ;
    return;
  }
  
  ++mode %= SYS_MODE_CNT;
  subMode = 0;

  switch (mode) {
  case NORMAL:
    system_mode |= MD_CLOCK;
    system_mode &= ~MD_DATE;
    system_mode |= MD_UPDATE;

    if (adjust_hr || adjust_min || adjust_y || adjust_mon || adjust_d) {
      adjust_s = (currentTime.second() + 1) % 60;
      system_mode |= MD_SYNC;
    }
    break;

  case LIGHT:
    system_mode &= ~MD_CLOCK;
    memset(numberToDisplay, 0, IC_COUNT);
    break;

  case ALARM:    
    system_mode &= ~MD_CLOCK;
    system_mode |= MD_UPDATE;
    
    if (settings & FLAG_LED)
      fill_solid(leds, IC_COUNT, ledColor);
    else
      fill_solid(leds, IC_COUNT, CRGB(0, 0, 0));

    EEPROMwl.update(ADDR_LED_H, ledColor.h);
    EEPROMwl.update(ADDR_LED_S, ledColor.s);
    EEPROMwl.update(ADDR_LED_V, ledColor.v);

    EEPROMwl.update(ADDR_OCR1A_H, (byte)(0xFF & (OCR1A >> 8)));
    EEPROMwl.update(ADDR_OCR1A_L, (byte)(0xFF & OCR1A));

    SET_HR(alarm_h);
    SET_MIN(alarm_m);
    SET_SEC(0);
    
    break;

  case TIMER:
    system_mode &= ~MD_CLOCK;
    digitalWrite(LED_TIMER, HIGH);
    if (settings & FLAG_TMR) {
      
    }
    else {
      memset(numberToDisplay, 0, IC_COUNT);
    }
    break;

  case SETTIME:
    system_mode |= MD_CLOCK;
    digitalWrite(LED_TIMER, LOW);
    break;
  }
}

inline void btn_LIGHT_press() {
  if ((system_mode & MD_BUZZ) && mode != ALARM) {
    system_mode &= ~MD_BUZZ;
    return;
  }
  
  switch (mode) {
  case NORMAL:
    settings ^= FLAG_LED;
    EEPROMwl.update(ADDR_SETTING, settings);
    if (settings & FLAG_LED)
      fill_solid(leds, IC_COUNT, ledColor);
    else
      fill_solid(leds, IC_COUNT, CRGB(0, 0, 0));
    system_mode |= MD_UPDATE;
    break;

  case ALARM:
    if (subMode == SET_ALM_ON)
      system_mode ^= MD_BUZZ;
    else
      addTimeOnDisplay(-1);
    break;

  case SETTIME:
    adjustTime(-1);
    break;
  }
}

inline void btn_START_press() {
  if (system_mode & MD_BUZZ) {
    system_mode &= ~MD_BUZZ;
    return;
  }
  
  switch (mode) {
  case NORMAL:
    system_mode |= MD_DATE;
    system_mode |= MD_UPDATE;
    break;

  case ALARM:
    if (subMode == SET_ALM_ON) {
      settings ^= FLAG_ALM;
      EEPROMwl.update(ADDR_SETTING, settings);
      digitalWrite(LED_ALARM, settings & FLAG_ALM);
    }
    else
      addTimeOnDisplay(1);
    break;

  case SETTIME:
    if (subMode == SET_TIME_SEC)
      system_mode |= MD_SYNC;
    else
      adjustTime(1);
    break;
  }
}

inline void btn_RESET_press() {
  if (system_mode & MD_BUZZ) {
    system_mode &= ~MD_BUZZ;
    return;
  }
  
  switch (mode) {
  case NORMAL:
    settings ^= FLAG_12HR;
    EEPROMwl.update(ADDR_SETTING, settings);
    system_mode |= MD_UPDATE;
    break;

  case LIGHT:
    ++subMode %= SET_LIGHT_CNT;

    if (subMode == SET_NIXIE)
      system_mode &= ~MD_CLOCK;
    else
      system_mode |= MD_CLOCK;

    /*
      leds[0] is used to indicate what in HSV is being set
      by changing H, S, or V gradually.
      Set leds[0] back to ledColor when switch to next subMode.
    */
    leds[0] = ledColor;
    system_mode |= MD_UPDATE;
    break;

  case ALARM:
    ++subMode %= SET_ALM_CNT;
    
    SET_HR(alarm_h);
    SET_MIN(alarm_m);
    SET_SEC(0);
    break;

  case SETTIME:
    ++subMode %= SET_TIME_CNT;
    
    if (subMode == SET_TIME_SEC)
      system_mode &= ~MD_DATE;
    else if (subMode == SET_TIME_Y)
      system_mode |= MD_DATE;
    break;
  }
}


inline void btn_START_release() {
  switch (mode) {
  case NORMAL:
    system_mode &= ~MD_DATE;
    system_mode |= MD_UPDATE;
    break;
  }
}
/**************************************************************************/



/**************************************************************************/
/* ROTARTY ENCODER ACTIONS */

inline void rotateCW(bool clockwise = true) {

  int sign = (clockwise) ? 1 : -1;

  switch (mode) {

  /*** LIGHT MODE ***/
  case LIGHT:
    switch (subMode) {
    case SET_NIXIE:
      /*
         Multiply dutyCycle(nixie tube's brightness) by 1.05 or 0.95.
         Use multiply instead of add will look more instinctively for human.
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

    fill_solid(leds, IC_COUNT, ledColor);
    break;

  /*** ALARM MODE ***/
  case ALARM:
    addTimeOnDisplay(sign);
    break;

  /*** SETTIME MODE ***/
  case SETTIME:
    if (subMode != SET_TIME_SEC)
      adjustTime(sign);
    break;
  }

}
/**************************************************************************/



inline void updateTime() {  
  (system_mode & MD_DATE)? updateDate() : updateHMS();
}

inline void bootload() { 
  rtc.begin();
  rtc.adjust(DateTime(2021, 4, 20, 22, 10, 0));
  
  EEPROMwl.begin(EEPROM_LAYOUT_VERSION, AMOUNT_OF_INDEXES);

  EEPROMwl.write(ADDR_SETTING, settings);
  EEPROMwl.write(ADDR_ALARM_H, alarm_h);
  EEPROMwl.write(ADDR_ALARM_M, alarm_m);
  EEPROMwl.write(ADDR_LED_H, ledColor.h);
  EEPROMwl.write(ADDR_LED_S, ledColor.s);
  EEPROMwl.write(ADDR_LED_V, ledColor.v);
  EEPROMwl.write(ADDR_OCR1A_H, 0xFF);
  EEPROMwl.write(ADDR_OCR1A_L, 0xFF);
}

void setup() {
  /* 
   * Run bootload() for the fist time using this program
   * or want to reset the whole clock
   */
//  bootload();
  
  for (int i = 1; i < IC_COUNT; ++i)
    rollNumber[i] = (rollNumber[i - 1] + 7) % 10;
  
  EEPROMwl.begin(EEPROM_LAYOUT_VERSION, AMOUNT_OF_INDEXES);

  settings = EEPROMwl.read(ADDR_SETTING);
  
  timer_setup();
  int high = EEPROMwl.read(ADDR_OCR1A_H);
  OCR1A |= high << 8;
  OCR1A |= EEPROMwl.read(ADDR_OCR1A_L);
  dutyCycle = OCR1A / 65535.0;

  register_setup();
  
  rtc_setup();
  alarm_h = EEPROMwl.read(ADDR_ALARM_H);
  alarm_m = EEPROMwl.read(ADDR_ALARM_M);
  
  button_setup();
  
  led_setup();
  byte h = EEPROMwl.read(ADDR_LED_H);
  byte s = EEPROMwl.read(ADDR_LED_S);
  byte v = EEPROMwl.read(ADDR_LED_V);
  ledColor = CHSV(h, s, v);
  if (settings & FLAG_LED)
    fill_solid(leds, IC_COUNT, ledColor);
  else
    fill_solid(leds, IC_COUNT, CRGB(0, 0, 0));
  FastLED.show();

  digitalWrite(LED_ALARM, settings & FLAG_ALM);
  digitalWrite(LED_TIMER, settings & FLAG_TMR);
}

void loop() {
  if (rtc.now().minute() % 10 == 0 && rtc.now().second() == 0) {
    SET_INTERVAL(120000, [] {
      system_mode |= MD_ROLL;
    });
  }

  if (system_mode & MD_ROLL)
    return;

  if (system_mode & MD_BUZZ)
    playMelody();

  /* update clock instantly (when ISR is called) */
  if (system_mode & MD_UPDATE) {
    if (system_mode & MD_CLOCK)
      updateTime();
      
    FastLED.show();
    
    rtc.setAlarm1(DateTime(0, 0, 0, alarm_h, alarm_m, 0), DS3231_A1_Hour);
    EEPROMwl.update(ADDR_ALARM_H, alarm_h);
    EEPROMwl.update(ADDR_ALARM_M, alarm_m);
    system_mode &= ~MD_UPDATE;
    return;
  }

  /* update clock every 100ms */
  SET_INTERVAL(100, [] {
    /* alarm set off */
    if (rtc.alarmFired(1)) {
      rtc.clearAlarm(1);
      if (settings & FLAG_ALM) {
        restartMelody();
        system_mode |= MD_BUZZ;
      }
    }

    if (system_mode & MD_CLOCK)
      updateTime();

    if (mode == LIGHT)
      md_LIGHT_indicate();

    if (mode == ALARM)
      md_ALARM_indicate();

    if (mode == SETTIME)
      md_SETTIME_indicate();
  });
}
