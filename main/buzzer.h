#define SET_INTERVAL(TIME, CALLBACK, ...)\
do {\
  static unsigned long t = 0;\
  if(millis() - t >= TIME){\
    CALLBACK(__VA_ARGS__);\
    t = millis();\
  }\
} while(0)

const int frequency[24] = {0, 131, 147, 156, 175, 196, 208,
                          262, 294, 311, 349, 392, 415, 466,
                          523, 587, 622, 698, 740, 784, 831, 932, 988,
                          1047};
#define BUZZER A2

#define REST  0
#define C3    (1 << 3)
#define D3    (2 << 3)
#define DS3   (3 << 3)
#define F3    (4 << 3)
#define G3    (5 << 3)
#define GS3   (6 << 3)

#define C4    (7 << 3)
#define D4    (8 << 3)
#define DS4   (9 << 3)
#define F4    (10 << 3)
#define G4    (11 << 3)
#define GS4   (12 << 3)
#define AS4   (13 << 3)

#define C5    (14 << 3)
#define D5    (15 << 3)
#define DS5   (16 << 3)
#define F5    (17 << 3)
#define FS5   (18 << 3)
#define G5    (19 << 3)
#define GS5   (20 << 3)
#define AS5   (21 << 3)
#define B5    (22 << 3)

#define C6    (23 << 3)

#define DOT   0b100

// wholenote = (60000 * 4) / tempo / beatUnit;
unsigned int wholenote = (60000 * 4) / 93 / 8;

union Note {
  byte note;
  struct {
    unsigned divider: 2;
    unsigned dot: 1;
    unsigned index: 5;
  };
};

const Note melody[] = {
  
  G5|0, F5|1, DS5|1, D5|1, AS4|1, G4|0, C5|1, D5|1, DS5|1, F5|1, D5|1, REST|1, G3|0,

  C6|1, G3|1, REST|1, G3|1, G5|1, G3|1, REST|1, G3|1, DS5|1, G3|1, D5|1, G3|1, C5|1, G3|1, REST|1, G3|1,
  C5|1, C4|1, D5|1, C4|1, DS5|1, C4|1, C5|1, C4|1, AS4|1, G3|1, C5|1, G3|1, G4|1, G3|1, REST|1, G3|1, 
  C6|1, G3|1, REST|1, G3|1, G5|1, G3|1, REST|1, G3|1, DS5|1, G3|1, D5|1, G3|1, C5|1, G3|1, C5|1, D5|1,
  DS5|1, C3|1, F5|1, C3|1, D5|1, C3|1, AS4|1, C3|1, C5|1, G3|1, G4|1, G3|1, C5|1, G3|1, REST|1, G3|1,

  C6|1, G3|1, REST|1, G3|1, G5|1, G3|1, REST|1, G3|1, DS5|1, G3|1, D5|1, G3|1, C5|1, G3|1, REST|1, G3|1,
  C5|1, C4|1, D5|1, C4|1, DS5|1, C4|1, C5|1, C4|1, AS4|1, G3|1, C5|1, G3|1, G4|1, G3|1, REST|1, G3|1,
  C6|1, G3|1, REST|1, G3|1, G5|1, G3|1, REST|1, G3|1, DS5|1, C4|1,F5|1, C4|1, G5|1, C4|1, C5|1, C3|1,
  D5|1, D3|1, F5|1, D3|1, D5|1, D3|1, AS4|1, D3|1, C5|1, G3|1, REST|1, G3|1, REST|1, G3|1, REST|1, G3|1,

  F3|1, F4|1, F3|1, F4|1, GS3|1, F4|1, C4|1, F4|1, DS4|1, DS3|1, D4|1, DS4|1, C4|1, DS4|1, G3|1, DS4|1,
  F3|1, F4|1, F3|1, F4|1, GS3|1, F4|1, C4|1, F4|1, DS4|1, DS3|1, F4|1, G3|1, G4|1, G3|1, REST|1, G3|1,
  GS5|1, C4|1, C3|1, G5|1, F5|1, C4|1, DS5|1, F5|1, G5|1, G3|1, C4|1, G3|1, G5|1, G3|1, REST|1, G3|1,
  F5|DOT|1, F5|2, F5|1, DS5|1, D5|1, D3|1, DS5|1, F5|1, DS5|1, G3|1, F5|1, G3|1, G5|1, G3|1, DS5|1, G3|1,

  F3|1, F4|1, F3|1, F4|1, GS3|1, F4|1, C4|1, F4|1, DS4|1, DS3|1, D4|1, DS4|1, C4|1, DS4|1, G3|1, DS4|1,
  F3|1, F4|1, F3|1, F4|1, GS3|1, F4|1, C4|1, F4|1, DS4|1, DS3|1, F4|1, G3|1, G4|1, G3|1, REST|1, G3|1,
  GS5|1, C4|1, C3|1, G5|1, F5|DOT|1, F5|2, G5|1, AS5|1, C6|1, G3|1, G5|1, G3|1, DS5|1, G3|1, C4|1, G3|1,
  D5|DOT|1, D5|2, D5|1, F5|1, D5|1, AS4|1, G4|1, AS4|1, C5|1, G3|1, REST|1, G3|1, G3|0, AS4|0,

  DS5|1, DS5|2, DS5|2, DS5|1, F5|1, G5|1, G5|2, G5|2, F5|1, DS5|1, D5|1, D5|2, D5|2, D5|1, DS5|1, D5|DOT|0, DS5|2, D5|2,
  C5|1, C5|2, C5|2, C5|1, D5|1, DS5|1, DS5|1, D5|1, C5|1, AS4|1, AS4|2, AS4|2, AS4|1, C5|1, D5|DOT|0, C5|2, AS4|2, 
  GS4|1, GS4|2, GS4|2, GS4|1, AS4|1, C5|1, C5|2, C5|2, AS4|1, GS4|1, G4|1, G4|2, AS4|2, DS5|1, F5|1, DS5|0, AS4|0,
  DS5|1, DS5|2, DS5|2, DS5|1, F5|1, FS5|1, FS5|1, GS5|1, FS5|1, F5|1, F5|2, F5|2, F5|1, FS5|1, F5|DOT|0, G5|2, F5|2, 

  DS5|1, DS5|2, DS5|2, DS5|1, F5|1, G5|1, G5|2, G5|2, F5|1, DS5|1, D5|1, D5|2, D5|2, D5|1, DS5|1, D5|DOT|0, DS5|2, D5|2,
  C5|1, C5|2, C5|2, C5|1, D5|1, DS5|1, DS5|1, D5|1, C5|1, AS4|1, AS4|2, AS4|2, AS4|1, C5|1, D5|DOT|0, C5|2, AS4|2, 
  GS4|1, GS4|2, GS4|2, GS4|1, AS4|1, C5|1, C5|2, C5|2, DS5|1, F5|1, G5|1, G5|2, G5|2, G5|1, AS5|1, G5|1, G5|1, F5|1, DS4|1,
  D5|1, F5|1, G5|1, D5|1, F5|1, G5|1, D5|1, F5|1, D5|1, F5|1, G5|1, C6|1, B5|0, REST|0
  
};

const unsigned int noteLen = sizeof(melody) / sizeof(*melody);
unsigned int thisNote = 0;

void playMelody() {  
  static unsigned int noteDuration = 0;

  SET_INTERVAL(noteDuration, [] {
    Note note = melody[thisNote];
    noteDuration = wholenote >> note.divider;

    /* dealing with dotted notes (add half of the duration) */
    noteDuration += (noteDuration >> 1) * note.dot;

    /* only plays 87.5% of the noteDuration */
    unsigned int buzzDuration;
    buzzDuration = (noteDuration >> 1) + (noteDuration >> 2) + (noteDuration >> 3);
    
    tone(BUZZER, frequency[note.index], buzzDuration);
    ++thisNote %= noteLen;
  });
}

static inline void restartMelody() {
  thisNote = 0;
}
