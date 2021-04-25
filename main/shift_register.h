/* our PCB can only have 6 or 8 nixie tubes
 * define IC_COUNT_8 if there are 8 nixie tubes
 */
// #define IC_COUNT_8

#ifdef IC_COUNT_8
#define IC_COUNT 8
#else
#define IC_COUNT 6
#endif

#define ST_CP 5
#define SH_CP 6
#define DS    4

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

void pushData(byte data[]) {

  MSLATCHLOW;
  for (int i = 0; i < IC_COUNT; ++i) {

    /*
     * set first 4 bits to 0 since data[i]
     * should always be less than 15
     */
    MSCLOCKLOW;
    MSDATALOW;
    
    MSCLOCKFLICK;
    MSCLOCKFLICK;
    MSCLOCKFLICK;
    MSCLOCKFLICK;

    /* push the remain 4 bits to register */
    byte bitMask = 0b1000;
    byte data_cpy = data[i];
    MSDATASET(data_cpy & bitMask);
    MSCLOCKFLICK;
    
    bitMask >>= 1;
    MSDATASET(data_cpy & bitMask);
    MSCLOCKFLICK;
    
    bitMask >>= 1;
    MSDATASET(data_cpy & bitMask);
    MSCLOCKFLICK;
    
    bitMask >>= 1;
    MSDATASET(data_cpy & bitMask);
    MSCLOCKFLICK;
  }
  MSLATCHHIGH;
}

void emptyDisplay(){
  /*
   * push 0b00001111 to all register
   * for nixie tube to turn off all light
   */
   
  MSLATCHLOW;
  for (int i = 0; i < IC_COUNT; ++i) {
    MSCLOCKLOW;
    MSDATALOW;
    
    MSCLOCKFLICK;
    MSCLOCKFLICK;
    MSCLOCKFLICK;
    MSCLOCKFLICK;

    MSDATAHIGH;
    
    MSCLOCKFLICK;
    MSCLOCKFLICK;
    MSCLOCKFLICK;
    MSCLOCKFLICK;
  }
  MSLATCHHIGH;
}
