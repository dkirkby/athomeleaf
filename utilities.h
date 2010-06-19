#include "WProgram.h"
#include <math.h>
#include "pins.h"
#include "packet.h"

// ---------------------------------------------------------------------
// Configuration data read from EEPROM
// ---------------------------------------------------------------------
#define CONFIG_ADDR        0x14 // EEPROM offset where saved config data starts

void loadConfig(Config *config);
void saveConfig(const Config *config);

// ---------------------------------------------------------------------
// Shared globals
// ---------------------------------------------------------------------
extern byte byteValue;
extern unsigned int uintValue;
extern float floatValue;

// Printing support for float values

#define printFloat(x,m) { floatToPrint = (x); printMultiplier = (m); _printFloat(); }

extern float floatToPrint;
extern unsigned long multiplier;
extern void _printFloat(void);

// Support for optional 16x2 LCD
void LCDinit(byte backlightLevel = 5); // range is 0 (off) to 29 (max)
void LCDprint(const char *line1, const char *line2 = 0);
void LCDclear();
void LCDpos(byte row, byte col=0);

// Geiger-clicking support 

extern unsigned long clickThreshold;
extern void tick(void);

// Synchronous sampling

extern uint16_t buffer[],*bufptr,delayCycles;
extern byte counter;
extern unsigned long timestamp;
extern void dumpBuffer(byte dumpType,BufferDump *dump);
extern void unpackSamples(const uint8_t *src, uint16_t *dst);

#define DUMP_BUFFER_POWER_LO 0
#define DUMP_BUFFER_POWER_HI 1
#define DUMP_BUFFER_LIGHT_LO 2
#define DUMP_BUFFER_LIGHT_HI 3
#define DUMP_BUFFER_AC_PHASE 4

// ---------------------------------------------------------------------
// Do a burst of 256 x 5kHz ADC samples lasting exactly 51,200 us
// 250 samples span exactly 3 60Hz powerline cycles.
// ---------------------------------------------------------------------
#define acquireADCSamples(ADC_CHANNEL) {\
    bufptr = buffer; \
    timestamp = micros(); \
    noInterrupts(); \
    do { \
        /* toggle pin13 to allow scope timing measurements */ \
        digitalWrite(STROBE_PIN, HIGH); \
        *bufptr++ = TCNT0; \
        *bufptr++ = analogRead(ADC_CHANNEL); \
        digitalWrite(STROBE_PIN, LOW); \
        /* insert some idle delay (borrowed from delayMicroseconds() in wiring.c) */ \
        delayCycles = 328; /* 4 CPU cycles = 0.25us per iteration */ \
        __asm__ __volatile__ ( \
            "1: sbiw %0,1" "\n\t" /* 2 cycles */ \
            "brne 1b" : "=w" (delayCycles) : "0" (delayCycles) /* 2 cycles */ \
            ); \
    } while(++counter); /* wraps around at 256 */ \
    interrupts(); \
}

// Lighting analysis

extern unsigned short lightingMean,lighting120Hz;
extern void lightingAnalysis(float scaleFactor, BufferDump *dump);

// Power analysis

extern void powerAnalysis(float scaleFactor, BufferDump *dump);
extern void phaseAnalysis(BufferDump *dump);

// Signalling

extern void chirp(byte,byte);
extern void tone(unsigned int, unsigned int);
extern void cricket(),bird(byte repetions=3);
