/* 
 * Our PCB can only have 6 or 8 nixie tubes.
 * Define IC_COUNT_8 if there are 8 nixie tubes.
 */
// #define IC_COUNT_8

#ifdef IC_COUNT_8
#define IC_COUNT 8
#else
#define IC_COUNT 6
#endif

byte numberToDisplay[IC_COUNT];

#define ST_CP 5
#define SH_CP 6
#define DS    4

#define NO_LIGHT 15;

#ifdef __AVR_ATmega32U4__

#define MSDATAHIGH  PORTD |=  (1 << PD4)
#define MSDATALOW   PORTD &= ~(1 << PD4)
  
#define MSCLOCKHIGH PORTD |=  (1 << PD7)
#define MSCLOCKLOW  PORTD &= ~(1 << PD7)

#define MSLATCHHIGH PORTC |=  (1 << PC6)
#define MSLATCHLOW  PORTC &= ~(1 << PC6)

#else

#define PORT_DATA   digitalPinToPort(DS)
#define PORT_CLOCK  digitalPinToPort(SH_CP)
#define PORT_LATCH  digitalPinToPort(ST_CP)

#define MSDATAHIGH  *portOutputRegister(PORT_DATA)  |=  digitalPinToBitMask(DS)
#define MSDATALOW   *portOutputRegister(PORT_DATA)  &= ~digitalPinToBitMask(DS)

#define MSCLOCKHIGH *portOutputRegister(PORT_CLOCK) |=  digitalPinToBitMask(SH_CP)
#define MSCLOCKLOW  *portOutputRegister(PORT_CLOCK) &= ~digitalPinToBitMask(SH_CP)

#define MSLATCHHIGH *portOutputRegister(PORT_LATCH) |=  digitalPinToBitMask(ST_CP)
#define MSLATCHLOW  *portOutputRegister(PORT_LATCH) &= ~digitalPinToBitMask(ST_CP)

#endif

#define MSDATASET(X) (X) ? (MSDATAHIGH) : (MSDATALOW)
#define MSCLOCKFLICK do {MSCLOCKHIGH; MSCLOCKLOW;} while (0)

static inline void register_setup() {
  pinMode(ST_CP, OUTPUT);
  pinMode(SH_CP, OUTPUT);
  pinMode(DS, OUTPUT);
}

static inline void __attribute__((optimize("-O3"))) pushData(byte data[]) {
  MSLATCHLOW;
  for (int i = 0; i < IC_COUNT; ++i) {

    /*
     * set first 4 bits to 0 since data[i]
     * should always be less than 15
     */
    MSCLOCKLOW;
    MSDATALOW;
    for (int j = 0; j < 4; ++j)
      MSCLOCKFLICK;

    /* push the remain 4 bits to register */
    for (byte bitMask = 0b1000; bitMask; bitMask >>= 1) {
      MSDATASET(data[i] & bitMask);
      MSCLOCKFLICK;
    }
  }
  MSLATCHHIGH;
}

/*
 * Push 0x0F to all register
 * for nixie tube to turn off all light.
 */
static inline void __attribute__((optimize("-O3"))) emptyDisplay(){   
  MSLATCHLOW;
  for (int i = 0; i < IC_COUNT; ++i) {
    MSCLOCKLOW;
    
    MSDATALOW;
    for (int j = 0; j < 4; ++j)
      MSCLOCKFLICK;

    MSDATAHIGH;
    for (int j = 0; j < 4; ++j)
      MSCLOCKFLICK;
  }
  MSLATCHHIGH;
}

#define SET_HR(HOUR)\
do {\
  numberToDisplay[5] = NO_LIGHT;\
  numberToDisplay[IC_COUNT - 1] = (HOUR) / 10;\
  numberToDisplay[IC_COUNT - 2] = (HOUR) % 10;\
} while(0)

#ifdef IC_COUNT_8
#define SET_MIN(MINUTE)\
do {\
  numberToDisplay[4] = (MINUTE) / 10;\
  numberToDisplay[3] = (MINUTE) % 10;\
  numberToDisplay[2] = NO_LIGHT;\
} while(0)
#else
#define SET_MIN(MINUTE)\
do {\
  numberToDisplay[3] = (MINUTE) / 10;\
  numberToDisplay[2] = (MINUTE) % 10;\
} while(0)
#endif

#define SET_SEC(SECOND)\
do {\
  numberToDisplay[1] = (SECOND) / 10;\
  numberToDisplay[0] = (SECOND) % 10;\
} while(0)

#define SET_MONTH(MONTH)\
do {\
  numberToDisplay[3] = (MONTH) / 10;\
  numberToDisplay[2] = (MONTH) % 10;\
} while(0)

#define SET_DAY(DAY)\
do {\
  numberToDisplay[1] = (DAY) / 10;\
  numberToDisplay[0] = (DAY) % 10;\
} while(0)

#define SET_YEAR(YEAR)\
do {\
  int _year = (YEAR);\
  for (int i = 4; i < IC_COUNT; ++i) {\
    numberToDisplay[i] = _year % 10;\
    _year /= 10;\
  }\
} while(0)

#define CLR_HR()\
do {\
  numberToDisplay[IC_COUNT - 1] = NO_LIGHT;\
  numberToDisplay[IC_COUNT - 2] = NO_LIGHT;\
} while(0)

#ifdef IC_COUNT_8
#define CLR_MIN()\
do {\
  numberToDisplay[4] = NO_LIGHT;\
  numberToDisplay[3] = NO_LIGHT;\
} while(0)
#else
#define CLR_MIN()\
do {\
  numberToDisplay[3] = NO_LIGHT;\
  numberToDisplay[2] = NO_LIGHT;\
} while(0)
#endif

#define CLR_SEC()\
do {\
  numberToDisplay[1] = NO_LIGHT;\
  numberToDisplay[0] = NO_LIGHT;\
} while(0)

#define CLR_MONTH()\
do {\
  numberToDisplay[3] = NO_LIGHT;\
  numberToDisplay[2] = NO_LIGHT;\
} while(0)

#define CLR_DAY()\
do {\
  numberToDisplay[1] = NO_LIGHT;\
  numberToDisplay[0] = NO_LIGHT;\
} while(0)

#define CLR_YEAR()\
do {\
  for (int i = 4; i < IC_COUNT; ++i)\
    numberToDisplay[i] = NO_LIGHT;\
} while(0)
