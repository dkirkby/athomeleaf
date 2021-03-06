#ifndef UTILITIES_H
#define UTILITIES_H

#include "pins.h" // move into .cpp when STROBE_PIN has been eliminated below
#include "packet.h"

#include <stdint.h>

// ---------------------------------------------------------------------
// Configuration data read from EEPROM
// ---------------------------------------------------------------------
#define CONFIG_ADDR        0x14 // EEPROM offset where saved config data starts

void loadConfig(Config *config);
void saveConfig(const Config *config);

// Printing support

extern void pprint(uint8_t value);
extern void pprint(uint16_t value);
extern void pprint(uint32_t value);
extern void printFloat(float value, uint32_t multiplier);

// Synchronous sampling

extern uint16_t buffer[],*bufptr,delayCycles;
extern uint8_t counter;
extern uint32_t timestamp;
extern void dumpBuffer(uint8_t dumpType,BufferDump *dump);
extern void unpackSamples(const uint8_t *src, uint16_t *dst);

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

extern float lightLevel,light120Hz;
extern void lightingAnalysis(float scale, uint16_t delay, BufferDump *dump);

// Power analysis

extern uint8_t nClipped,wrapOffset,currentComplexity;
extern uint16_t voltagePhase;
extern float apparentPower,zeroXingDelay;
extern uint32_t moment0,moment1,tzero;

extern void powerAnalysis(uint16_t gain, uint16_t delay, BufferDump *dump);
extern void phaseAnalysis(BufferDump *dump);

#endif
