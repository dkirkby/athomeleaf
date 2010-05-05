#include "WProgram.h"
#include <math.h>
#include "pins.h"
#include "mirf.h"
#include "packet.h"
#include "config.h"

// ---------------------------------------------------------------------
// Configuration data read from EEPROM
// ---------------------------------------------------------------------
#define SERIAL_NUMBER_ADDR 0x10

extern void copySerialNumber(LookAtMe*);

// ---------------------------------------------------------------------
// Shared globals
// ---------------------------------------------------------------------
extern byte byteValue;
extern unsigned int uintValue;
extern float floatValue;
extern DataPacket packet,dumpPacket;

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

extern unsigned short buffer[],*bufptr,delayCycles;
extern byte counter;
extern void dumpBuffer(byte dumpType);

// Lighting analysis

extern unsigned short lightingMean,lighting120Hz;
extern void lightingAnalysis(float scaleFactor);

// Power analysis

extern float rmsPower;
extern void powerAnalysis(void);

// Wireless

#define RADIO_CHANNEL 0
#define NORDIC_ADDR_LEN 3
#define NORDIC_MAX_RETRIES 15

extern byte nordicOK;
extern byte idleAddress[],dataAddress[],configAddress[];
extern byte lamAddressLSB;

extern void initNordic(unsigned short id, byte isHub);
extern byte getNordic(byte *payload, byte payloadSize);
extern byte sendNordic(byte *address, byte *payload, byte payloadSize);

// Signalling

extern void chirp(byte,byte);
extern void tone(unsigned int, unsigned int);
extern void cricket(void);
