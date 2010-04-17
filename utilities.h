#include "WProgram.h"
#include <math.h>
#include "pins.h"
#include "config.h"
#include "mirf.h"
#include "packet.h"

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
void LCDinit(byte backlightLevel = 15); // range is 0 (off) to 29 (max)
void LCDprint(const char *line1, const char *line2 = 0);

// Geiger-clicking support 

extern unsigned long clickThreshold;
extern void tick(void);

// Synchronous sampling

extern unsigned int buffer[],*bufptr,delayCycles;
extern byte counter;

// Lighting analysis

extern unsigned int lightingMean,lighting120Hz;
extern void lightingAnalysis(void);

// Power analysis

extern float rmsPower;
extern void powerAnalysis(void);

// Wireless

extern byte nordicOK;
extern void initNordic(byte);

// Signalling

extern void chirp(byte,byte);
extern void tone(unsigned int, unsigned int);
extern void cricket(void);
