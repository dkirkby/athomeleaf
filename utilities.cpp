#include "WProgram.h" // arduino header

#include "utilities.h"
#include "wireless.h"

#include <math.h>
#include <avr/eeprom.h>

// ---------------------------------------------------------------------
// Shared globals
// ---------------------------------------------------------------------
uint8_t byteValue;
uint16_t uintValue;
float floatValue;

// =====================================================================
// Loads previously saved config data from EEPROM into the specified
// RAM address. Skips over the fixed header.
// =====================================================================
void loadConfig(Config *config) {
    eeprom_read_block((void*)&(config->networkID),(void*)CONFIG_ADDR,
        sizeof(Config)-sizeof(config->header));
}

// =====================================================================
// Saves config data in RAM to EEPROM. Each byte is compared with the
// existing EEPROM contents before performing an erase-and-write cycle,
// in order to minimize the EEPROM wear. Ignores the values of the
// fixed header and network ID.
// =====================================================================

#define CONFIG_INITIAL_SKIP (sizeof(config->header) - sizeof(config->networkID))

void saveConfig(const Config *config) {
    // Iterate over the bytes to save after skipping over the fixed header and network ID
    uint8_t newValue,offset = sizeof(config->networkID);
    while(offset < sizeof(Config) - sizeof(config->header)) {
        // Do we need to change this byte in EEPROM?
        newValue = *((uint8_t*)config + sizeof(config->header) + offset);
        if(eeprom_read_byte((const uint8_t*)(CONFIG_ADDR+offset)) != newValue) {
            eeprom_write_byte((uint8_t*)(CONFIG_ADDR+offset),newValue);
        }        
        offset++;
    }
}

// =====================================================================
// Print a floating point value as a fixed-precision decimal.
// Uses Serial.write() and Serial.print(). Does not emit a newline.
// =====================================================================

float floatToPrint;
uint32_t printMultiplier;

void _printFloat() {

    static uint32_t printULong;

    // convert +/-ABC.XYZ to ABCXYZ with rounding
    if(floatToPrint < 0) {
        printULong = -floatToPrint*printMultiplier + 0.5;
        if(printULong) {
            Serial.write('-');
        }
    }
    else {
        printULong = floatToPrint*printMultiplier + 0.5;
    }
    // emit the integral part ABC
    Serial.print(printULong/printMultiplier,DEC);
    // emit the fractional part .XYZ
    Serial.write('.');
    printULong %= printMultiplier;
    // left pad the fractional part XYZ with zeros if necessary
    while(printMultiplier > 10) {
        printMultiplier /= 10;
        if(printULong >= printMultiplier) break;
        Serial.write('0');
    }
    Serial.print(printULong,DEC);
}

// =====================================================================
// Initializes an optional 16x2 serial LCD. There is no way to know
// if one is connected since the device is write-only.
// =====================================================================
void LCDinit(uint8_t backlightLevel) {
    Serial.begin(9600);
    // turn cursor off
    Serial.write(0xfe);
    Serial.write(0x0c);
    delay(100); // needed to allow LCD serial decoder to keep up?
    // turn backlight off
    Serial.write(0x7c);
    Serial.write((uint8_t)(0x80 | (backlightLevel % 30)));
    delay(100); // needed to allow LCD serial decoder to keep up?
}

// =====================================================================
// Clears the optional LCD display and writes the specified messages.
// The caller is responsible for ensuring that the message strings
// are no more than 16 characters long.
// =====================================================================
void LCDprint(const char *line1, const char *line2) {
    // clear the display
    Serial.write(0xfe);
    Serial.write(0x01);
    // show the first line
    Serial.print(line1);
    if(0 != line2) {
        // move to the second line
        Serial.write(0xfe);
        Serial.write(0xc0);
        Serial.print(line2);
    }
}

// =====================================================================
// Clears the optional LCD display
// =====================================================================
void LCDclear() {
    Serial.write(0xfe);
    Serial.write(0x01);
    delay(100);    
}

// =====================================================================
// Positions the (invisible) cursor on the optional LCD.
// row = 0 is the top line, row = 1 is the bottom line.
// valid col values are 0 (leftmost) to 15.
// =====================================================================
void LCDpos(uint8_t row, uint8_t col) {
    Serial.write(0xfe);
    if(row == 0) {
        Serial.write((uint8_t)(0x80 | (col & 0x0f)));
    }
    else {
        Serial.write((uint8_t)(0xc0 | (col & 0x0f)));
    }
    //delay(100);
}

// =====================================================================
// Generates a pseudo-random sequence of 32-bit unsigned integers
// =====================================================================

uint32_t randomValue = 24071966UL;

void nextRandom() {
    randomValue = randomValue*2891336453UL + 1640531513UL;
    randomValue ^= randomValue >> 13;
    randomValue ^= randomValue << 17;
    randomValue ^= randomValue >> 5;
}

// =====================================================================
// Delays for about 1ms and generates an audible and visible "Geiger"
// click at pseudo-random intervals with an average rate controlled
// by the value of the clickThreshold global.
// =====================================================================

uint32_t clickThreshold = 0;

void tick() {

    static uint8_t doClick;

    // generate a pseudo-random unsigned long
    nextRandom();
    doClick = (randomValue < clickThreshold) ? 1 : 0;
    delayMicroseconds(250);
    digitalWrite(PIEZO_PIN,doClick);
    delayMicroseconds(500);
    digitalWrite(PIEZO_PIN,LOW);
    delayMicroseconds(250);
}

// ---------------------------------------------------------------------
// Globals used for 60/120 Hz waveform captures (lighting + AC power)
// ---------------------------------------------------------------------
#define BUFFER_SIZE 512
uint16_t buffer[BUFFER_SIZE],*bufptr,delayCycles;
uint8_t counter = 0;

#define WAVEDATA(K) buffer[(((K)+6)<<1)|1]

// Used to latch the TIMER0 microsecond counter at a fixed time before
// the first sample is latched.
uint32_t timestamp;

// ---------------------------------------------------------------------
// Dump the buffer contents via the wireless interface. Values are
// assumed to be 10-bit and packed accordingly. The first 15 bytes
// of the dump buffer provided will be included in the first packet
// sent (and overwritten as subsequent packets are sent). The dump
// buffer should already have its networkID field set (and this will
// not be overwritten). In total, 22 packets will be sent.
// ---------------------------------------------------------------------
void dumpBuffer(uint8_t dumpType, BufferDump *dump) {
    // the first packet can be identified by its zero sequence number
    dump->sequenceNumber = 0;
    // skip over the first 15 bytes of packed data then...
    // ...send the dump type, and timestamp
    dump->packed[15] = dumpType;
    dump->packed[16] = 0xff & (timestamp >> 24);
    dump->packed[17] = 0xff & (timestamp >> 16);
    dump->packed[18] = 0xff & (timestamp >>  8);
    dump->packed[19] = 0xff & (timestamp);
    // ...and the first 8 packed samples
    packSamples(&buffer[0],&dump->packed[20]);
    packSamples(&buffer[4],&dump->packed[25]);
    // try to send the first packet now
    if(0x0f < sendNordic(dumpAddress, (uint8_t*)dump, sizeof(BufferDump))) {
        // don't keep going if our first packet didn't get through
        return;
    }
    // the remaining 21 packets have the same structure
    uintValue = 8;
    for(dump->sequenceNumber = 1; dump->sequenceNumber < 22; dump->sequenceNumber++) {
        packSamples(&buffer[uintValue],&dump->packed[0]);
        uintValue+= 4;
        packSamples(&buffer[uintValue],&dump->packed[5]);
        uintValue+= 4;
        packSamples(&buffer[uintValue],&dump->packed[10]);
        uintValue+= 4;
        packSamples(&buffer[uintValue],&dump->packed[15]);
        uintValue+= 4;
        packSamples(&buffer[uintValue],&dump->packed[20]);
        uintValue+= 4;
        packSamples(&buffer[uintValue],&dump->packed[25]);
        uintValue+= 4;
        // try to send this packet now
        if(0x0f < sendNordic(dumpAddress, (uint8_t*)dump, sizeof(BufferDump))) return;
    }
}

// ---------------------------------------------------------------------
// Lighting analysis parameters
// ---------------------------------------------------------------------
#define NLIGHTSAMP 250
#define NLIGHTSAMPBY2 125
#define NLIGHTSAMPBY4 63
#define MINUNCLIPPED 15
#define ADCMIN 0
#define ADCMAX 1005
// The index period corresponding to 120 Hz = 2pi/(NLIGHTSAMP/6)
#define DPHI120 0.15079644737231007

// ---------------------------------------------------------------------
// AC power analysis parameters
// ---------------------------------------------------------------------
#define NPOWERSAMP 250
#define NPOWERSAMPBY2 125
#define NPOWERSAMPBY4 63
// The index period corresponding to 60 Hz = 2pi/(NPOWERSAMP/3)
#define DPHI60 0.075398223686155036

// ---------------------------------------------------------------------
// Lighting and power analysis shared globals
// ---------------------------------------------------------------------
static uint8_t cycle,nzero;
static float sink,cosk,cosSum,sinSum;

// =====================================================================
// Analyzes a lighting waveform to determine its mean and 120Hz
// amplitude, using a linear least-squares technique that is robust
// against clipping and can reconstruct waveforms whose baseline exceeds
// the ADC input range as long as some 120Hz is visible within the range.
// This function makes the following math calls: 62*sin, 62*cos, 3*sqrt.
// The scale factor is used to scale the results from ADC counts
// to the unsigned short globals lightingMean and lighting120Hz.
// =====================================================================

uint16_t lightingMean,lighting120Hz;

void lightingAnalysis(float scaleFactor, BufferDump *dump) {
    
    static float beta0,beta1,beta2,alpha00,alpha01,alpha02,alpha11,alpha12,alpha22;
    
    nzero = 0;
    alpha00 = NLIGHTSAMP;
    alpha11 = alpha22 = NLIGHTSAMPBY2;
    alpha01 = alpha02 = alpha12 = 0;
    beta0 = beta1 = beta2 = 0;
    for(byteValue = 0; byteValue < NLIGHTSAMPBY4; byteValue++) {
        if(byteValue == 0) {
            sink = 0;
            cosk = 1;
        }
        else {
            floatValue = DPHI120*byteValue;
            sink = sin(floatValue);
            cosk = cos(floatValue);
        }
        for(cycle = 0; cycle < 4; cycle++) {
            if((cycle%2) && (byteValue==0)) continue;
            switch(cycle) {
                case 0:
                uintValue = WAVEDATA(byteValue);
                break;
                case 1:
                uintValue = WAVEDATA(NLIGHTSAMPBY2-byteValue);
                break;
                case 2:
                uintValue = WAVEDATA(NLIGHTSAMPBY2+byteValue);
                break;
                case 3:
                uintValue = WAVEDATA(NLIGHTSAMP-byteValue);
                break;
            }
            if(uintValue > ADCMIN && uintValue < ADCMAX) {
                floatValue = uintValue;
                beta0 += floatValue;
                beta1 += floatValue*cosk;
                beta2 += floatValue*sink;
            }
            else {
                if(uintValue <= ADCMIN) nzero++;
                alpha00 -= 1;
                alpha01 -= cosk;
                alpha02 -= sink;
                alpha11 -= cosk*cosk;
                alpha12 -= sink*cosk;
                alpha22 -= sink*sink;
            }
            sink = -sink;
        }
    }
    
    if(0 != dump) {
        /* zero out the dump header */
        for(byteValue = 0; byteValue < 15; byteValue++) dump->packed[byteValue] = 0;
    }

    // Check for an almost singular matrix which signals an over/under-flow condition
    // in the sampling. At this point, alpha00 contains the number of unclipped samples
    // out of a possible NLIGHTSAMP maximum.
    if(alpha00 < MINUNCLIPPED) {
        //beta1 = 0;
        lighting120Hz = 0xffff;
        if(nzero > NLIGHTSAMPBY2) {
            // underflow
            //beta0 = 0;
            lightingMean = 0;
        }
        else {
            // overflow
            //beta0 = -1;
            lightingMean = 0xffff;
        }
    }
    else {
        // Calculate the Cholesky decomposition of the symmetric postitive definite matrix alpha(i,j).
        // Replace each alpha(i,j) with the lower matrix element L(j,i).
        alpha00 = sqrt(alpha00);
        alpha01 /= alpha00;
        alpha02 /= alpha00;
        alpha11 = sqrt(alpha11-alpha01*alpha01);
        alpha12 = (alpha12-alpha01*alpha02)/alpha11;
        alpha22 -= alpha02*alpha02+alpha12*alpha12; // this is actually L(2,2)^2
    
        // Solve the linear equations alpha . x = beta for the vector of coeficients x.
        // Replace each beta(i) with x(i)
        beta0 /= alpha00;
        beta1 = (beta1 - alpha01*beta0)/alpha11;
        beta2 = (beta2 - alpha02*beta0 - alpha12*beta1)/alpha22;
        beta1 = (beta1 - alpha12*beta2)/alpha11;
        beta0 = (beta0 - alpha01*beta1 - alpha02*beta2)/alpha00;
        
        // Store the 120 Hz peak amplitude in beta1
        beta1 = sqrt(beta1*beta1 + beta2*beta2);
        
        // save our analysis results (in floating point ADC counts)
        // in case this buffer gets dumped
        if(0 != buffer) {
            *(float*)(&dump->packed[0]) = beta0;
            *(float*)(&dump->packed[4]) = beta1;
        }

        // scale beta0 to lightingMean
        beta0 = scaleFactor*beta0 + 0.5;
        if(beta0 <= 0) {
            lightingMean = 0;
        }
        else if(beta0 >= 0xfffe) {
            lightingMean = 0xfffe;
        }
        else {
            lightingMean = (unsigned short)beta0;
        }
        
        // scale beta1 to lighting120Hz
        beta1 = scaleFactor*beta1 + 0.5;
        if(beta1 <= 0) {
            lighting120Hz = 0;
        }
        else if(beta1 >= 0xfffe) {
            lighting120Hz = 0xfffe;
        }
        else {
            lighting120Hz = (unsigned short)beta1;
        }
        
    }
}

// =====================================================================
// Analyzes an AC current waveform to determine its true RMS power
// relative to a 120V RMS 60Hz voltage. The algorithm assumes that
// there is no clipping and that the load is purely resistive.
// This function makes the following math calls: 62*sin, 62*cos, 1*sqrt.
// The scale factor is used to scale the results from ADC counts.
// =====================================================================

void powerAnalysis(float scaleFactor, BufferDump *dump) {
    cosSum = sinSum = 0;
    for(byteValue = 0; byteValue < NPOWERSAMPBY4; byteValue++) {
        if(byteValue == 0) {
            sink = 0;
            cosk = 1;
        }
        else {
            floatValue = DPHI60*byteValue;
            sink = sin(floatValue);
            cosk = cos(floatValue);
        }
        for(cycle = 0; cycle < 4; cycle++) {
            if((cycle%2) && (byteValue==0)) {
                cosk = -1;
                continue;
            }
            switch(cycle) {
                case 0:
                uintValue = WAVEDATA(byteValue);
                break;
                case 1:
                uintValue = WAVEDATA(NPOWERSAMPBY2-byteValue);
                cosk = -cosk;
                break;
                case 2:
                uintValue = WAVEDATA(NPOWERSAMPBY2+byteValue);
                sink = -sink;
                break;
                case 3:
                uintValue = WAVEDATA(NPOWERSAMP-byteValue);
                cosk = -cosk;
                break;
            }
            floatValue = uintValue;
            cosSum += floatValue*cosk;
            sinSum += floatValue*sink;
        }
    }
    // store the floating point 60 Hz RMS in ADC units
    floatValue = sqrt(cosSum*cosSum+sinSum*sinSum);
    if(0 != dump) {
        /* zero out the dump header */
        for(byteValue = 0; byteValue < 15; byteValue++) dump->packed[byteValue] = 0;
        /* send our floating point RMS in the dump header */
        *(float*)(&dump->packed[0]) = floatValue;
    }
    // convert to a 16-bit integer using the provided scale factor
    uintValue = (unsigned short)(scaleFactor*floatValue+0.5);
}

// =====================================================================
// Aanalyzes the zero-crossing fiducial signal to determine the
// AC power factor.
// =====================================================================

void phaseAnalysis(BufferDump *dump) {
    if(0 != dump) {
        /* zero out the dump header */
        for(byteValue = 0; byteValue < 15; byteValue++) dump->packed[byteValue] = 0;
    }    
}

// =====================================================================
// Generates a frequency chirp
// =====================================================================

void chirp(uint8_t cycles, uint8_t timebase) {

    uint8_t counter = cycles;
    uint16_t delay = cycles*timebase;
    
    //digitalWrite(RED_LED_PIN,HIGH);
    do {
        digitalWrite(PIEZO_PIN,HIGH);
        delayMicroseconds(delay);
        digitalWrite(PIEZO_PIN,LOW);
        delayMicroseconds(delay);
        delay -= timebase;
    } while(--counter);

    counter = cycles;
    //digitalWrite(RED_LED_PIN,LOW);
    do {
        delay += timebase;
        digitalWrite(PIEZO_PIN,HIGH);
        delayMicroseconds(delay);
        digitalWrite(PIEZO_PIN,LOW);
        delayMicroseconds(delay);        
    } while(--counter);
}

// =====================================================================
// Generates a square-wave tone
// =====================================================================

void tone(uint16_t halfPeriod,uint16_t cycles) {
    uint16_t counter = cycles;
    while(counter--) {
        digitalWrite(PIEZO_PIN,HIGH);
        delayMicroseconds(halfPeriod);
        digitalWrite(PIEZO_PIN,LOW);
        delayMicroseconds(halfPeriod);
    }
}

// =====================================================================
// Copies REG_BIT from REG to PORT_BIT of PORT, for example:
//
//    writePort(PORTB,PORTB2,value,2);
//
// Takes exactly 7 clock cycles (0.4375us at 16MHz) with the transition
// occuring during a 2-cycle CBI/SBI operation that starts at the 4th
// cycle. When called repeatedly with the same REG (but possibly
// changing other parameters) there is no extra overhead incurred
// between calls and a 7-cycle period is guaranteed.
// =====================================================================

#define writePort(PORT,PORT_BIT,REG,REG_BIT) \
    __asm__ __volatile__ (                \
        "sbrs %[reg],%[regBit]"     "\n\t"\
    	"rjmp 1f"                   "\n\t"\
    	"nop"                       "\n\t"\
        "sbi %[port],%[portBit]"    "\n\t"\
    	"rjmp 2f"                   "\n\t"\
    "1:  cbi %[port],%[portBit]"    "\n\t"\
    	"nop"                       "\n\t"\
        "nop"                       "\n\t"\
    "2: "                                 \
        :/* no outputs */:                \
        [port] "I" (_SFR_IO_ADDR(PORT)),  \
        [portBit] "I" (PORT_BIT),         \
        [reg] "r" (REG),                  \
        [regBit] "I" (REG_BIT)            \
    );

#include <avr/pgmspace.h>
#include <avr/io.h>

// =====================================================================
// Shifts a binary waveform stored in flash memory out of the specified
// pin at a rate of 23 clock cycles per bit, or 16,000/23 ~ 696 kbps at
// 16 MHz which corresponds to 16x oversampling of ~43,478 Hz (as close
// as we can get to the 44,100 Hz sampling of CD quality audio).
//
// Each byte of the input data is shifted from LSB to MSB.
//
// One kByte of stored waveform is shifted out in 1024 x 8 x 23 / 16,000
// = 11.776 ms at 16 MHz.
//
// Without any delay, this routine is capable of 20 cycles per bit
// or 800 kbps at 16 MHz. Compare with the 1,411.2 kbps PCM rate used
// in CD quality audio = 2 chan x 16 bits/sample/chan x 44,100 samples/sec).
// However, 800 kbps corresponds to 100 kB/s or 10 ms/kB of sample data.
// =====================================================================

#define DELAY16 asm volatile("nop\n nop\n nop\n nop\nnop\n nop\n nop\n nop\nnop\n nop\n nop\n nop\nnop\n nop\n nop\n nop"::)
#define DELAY3 asm volatile("nop\n nop\n nop"::)

//#define PIEZO_PORT PORTB
//#define PIEZO_BIT  PORTB1

#define PIEZO_PORT PORTD
#define PIEZO_BIT  PORTD2

void shiftOut(uint16_t nData, const uint8_t *data) {
    
    uint8_t bits;
    // The loop overhead is 13 cycles and writing each bit takes 7 cycles.
    // Add 3 cycles delay between bits for a total period of 23 cycles.
    while(nData--) {
        bits = pgm_read_byte_near(data++);
        writePort(PIEZO_PORT,PIEZO_BIT,bits,0);
        DELAY16;
        writePort(PIEZO_PORT,PIEZO_BIT,bits,1);
        DELAY16;
        writePort(PIEZO_PORT,PIEZO_BIT,bits,2);
        DELAY16;
        writePort(PIEZO_PORT,PIEZO_BIT,bits,3);
        DELAY16;
        writePort(PIEZO_PORT,PIEZO_BIT,bits,4);
        DELAY16;
        writePort(PIEZO_PORT,PIEZO_BIT,bits,5);
        DELAY16;
        writePort(PIEZO_PORT,PIEZO_BIT,bits,6);
        DELAY16;
        writePort(PIEZO_PORT,PIEZO_BIT,bits,7);
        DELAY3;
    }
    
    // Always leave the bit cleared
    PIEZO_PORT &= ~_BV(PIEZO_BIT);
}

// =====================================================================
// Generates a cricket-like sound
// =====================================================================

#define CRICKET_SAMPLES 1766

uint8_t cricketSample[CRICKET_SAMPLES] PROGMEM = {
    170,170,85,85,85,85,85,85,85,85,85,85,85,85,85,85,149,170,170,170,82,85,85,85,85,181,170,170,85,85,171,
    170,170,90,165,170,170,74,85,169,42,85,165,170,170,170,170,106,85,171,86,181,106,85,173,170,170,42,85,
    169,82,165,82,169,74,85,149,170,85,181,106,181,90,173,85,171,85,85,85,85,169,84,42,165,84,74,165,74,85,
    85,181,106,181,214,90,107,173,214,170,90,165,170,82,41,165,164,148,82,74,165,170,170,106,213,90,107,107,
    107,107,173,85,171,170,82,165,148,146,148,164,164,148,82,85,169,86,171,173,173,109,219,218,218,170,85,85,
    85,74,74,73,146,36,73,41,165,84,85,173,86,219,182,109,219,182,181,213,106,165,170,164,164,36,146,36,73,
    82,82,170,170,90,173,109,219,118,219,109,219,106,181,170,84,73,74,18,137,36,146,36,37,85,149,213,90,187,
    109,183,187,109,183,181,90,85,165,146,36,137,72,36,34,73,146,170,84,173,181,109,183,187,187,221,110,107,
    173,169,170,36,73,68,68,68,34,146,36,149,170,90,107,187,219,221,187,219,221,214,214,74,85,146,36,17,34,
    66,68,36,137,74,85,181,182,238,238,238,189,187,187,109,109,85,170,36,137,136,16,66,132,72,36,165,74,107,
    107,187,187,247,189,123,247,218,182,170,82,137,36,66,8,33,8,145,136,74,165,214,214,238,221,251,190,247,
    238,181,109,85,149,34,137,16,132,32,8,34,17,165,82,173,173,123,247,126,223,247,190,219,182,170,170,72,36,
    4,65,32,16,66,68,74,169,218,218,222,189,223,223,247,125,183,109,85,85,18,145,64,16,16,16,132,8,149,82,
    181,181,123,239,247,239,239,251,110,187,170,170,68,36,8,4,8,16,8,33,74,41,91,91,223,251,254,251,239,247,
    221,110,85,85,17,145,0,1,2,16,8,66,146,82,181,181,251,222,255,254,247,223,187,221,170,170,68,68,16,64,0,
    16,16,4,37,37,91,91,223,247,239,255,247,191,119,119,85,85,17,17,1,8,0,16,32,16,74,82,181,181,251,126,255,
    255,251,255,238,238,170,170,68,68,128,0,0,4,64,32,146,36,91,91,191,239,255,255,253,255,222,189,85,85,17,
    17,2,0,16,0,64,128,36,73,173,181,247,251,255,251,255,255,189,123,181,170,68,68,0,16,0,0,128,0,73,146,218,
    90,127,191,255,255,255,255,189,239,170,85,34,18,32,0,0,0,0,1,146,36,109,173,247,247,255,255,255,255,123,
    223,85,181,136,136,0,0,0,0,0,2,18,73,182,214,254,254,255,255,255,255,251,190,181,106,34,34,0,32,0,0,0,0,
    36,146,90,107,223,223,255,255,255,255,255,125,171,85,17,17,1,0,0,0,0,0,68,34,173,181,251,251,255,255,255,
    255,255,251,90,173,68,68,0,0,0,0,0,0,72,68,181,90,127,127,255,255,255,255,255,255,86,107,34,34,0,128,0,0,
    0,0,128,136,90,171,239,239,255,255,255,255,255,255,181,90,145,136,0,0,0,0,0,0,128,136,106,181,253,253,
    255,255,255,255,255,255,173,213,72,68,0,0,0,0,0,0,0,17,173,86,127,127,255,255,255,255,255,255,175,173,36,
    34,0,0,0,0,0,0,0,33,181,106,223,239,255,255,255,255,255,255,111,109,18,145,0,0,0,0,0,0,0,32,85,171,251,
    251,255,255,255,255,255,255,111,107,145,72,0,0,0,0,0,0,0,32,86,181,253,254,255,255,255,255,255,255,111,
    91,73,36,0,0,0,0,0,0,0,64,90,85,191,191,255,255,255,255,255,255,127,219,72,34,0,16,0,0,0,0,0,0,106,85,
    223,239,255,255,255,255,255,255,127,219,36,146,0,0,0,0,0,0,0,0,170,85,239,251,255,255,255,255,255,255,
    127,219,36,145,0,0,0,0,0,0,0,0,170,106,247,253,255,255,255,255,255,255,127,219,36,73,0,0,0,0,0,0,0,0,170,
    170,251,190,255,255,255,255,255,255,255,219,36,73,0,64,0,0,0,0,0,0,168,170,251,222,255,255,255,255,255,
    255,255,219,36,73,0,2,0,0,0,0,0,0,84,170,125,223,255,255,255,255,255,255,255,219,36,37,32,0,0,0,0,0,0,0,
    84,169,125,239,255,255,255,255,255,255,255,237,36,37,8,0,0,0,0,0,0,0,84,165,125,239,255,255,255,255,255,
    255,255,109,37,41,4,0,0,0,0,0,0,0,82,149,123,247,255,255,255,255,255,255,255,118,41,41,4,0,0,0,0,0,0,0,
    42,85,123,247,255,255,255,255,255,255,127,183,73,41,4,64,0,0,0,0,0,0,168,82,123,247,255,255,255,255,255,
    255,191,219,74,74,4,32,0,0,0,0,0,0,148,74,119,247,255,255,255,255,255,255,223,237,84,74,8,16,0,0,0,0,0,0,
    82,169,238,238,223,255,255,255,255,255,255,118,165,82,16,16,0,0,0,0,0,0,42,165,238,238,247,255,255,255,
    255,255,191,187,41,149,16,16,0,0,0,0,0,0,165,82,221,237,239,255,255,255,255,255,223,221,74,165,32,16,0,0,
    0,0,0,32,82,74,187,221,239,255,255,255,255,255,119,119,85,42,65,16,0,0,0,0,0,4,41,165,182,219,223,255,
    255,255,255,255,187,187,169,82,130,32,0,0,0,0,128,128,148,146,109,183,191,255,255,255,255,255,238,238,82,
    85,4,33,0,0,0,0,16,16,74,73,219,182,127,255,255,255,255,251,183,187,85,170,8,66,0,0,0,0,2,1,165,164,182,
    109,255,254,255,255,247,255,221,221,170,82,33,66,0,0,32,0,32,16,82,74,109,219,254,253,255,247,255,191,
    119,119,85,85,66,132,0,64,0,0,8,2,37,37,219,182,251,253,223,255,255,254,221,221,170,170,136,136,0,32,0,0,
    129,32,82,82,181,173,247,251,247,255,239,127,119,119,85,85,17,17,2,16,0,8,16,4,37,37,91,91,223,247,247,
    255,253,247,221,221,170,170,68,34,4,16,128,0,132,32,82,82,181,182,189,239,247,191,127,255,118,119,85,85,
    137,68,16,8,32,128,32,4,37,37,107,173,123,239,247,239,223,239,219,221,170,170,34,137,32,8,8,16,8,33,82,
    82,181,86,239,222,247,247,247,253,118,183,85,85,73,18,65,16,4,4,66,132,164,164,106,173,221,189,247,251,
    125,223,219,237,170,170,146,36,130,16,4,129,16,33,74,74,181,90,119,187,239,251,222,247,110,187,86,85,73,
    82,136,16,130,32,68,136,148,148,90,173,238,118,239,123,239,189,219,110,181,170,146,164,16,33,66,16,34,34,
    41,41,173,90,219,237,222,189,247,238,110,219,170,85,73,73,34,34,66,136,136,136,146,82,85,173,110,219,221,
    189,187,119,219,182,85,173,148,148,68,36,34,68,36,34,41,165,170,86,219,182,187,221,221,221,109,219,170,
    106,74,41,137,68,34,34,18,73,82,74,213,170,182,109,187,219,237,110,219,182,90,85,165,82,18,73,34,18,73,
    34,165,84,86,85,91,219,182,219,110,183,173,109,213,170,82,169,36,73,36,145,36,73,74,165,170,170,181,181,
    109,219,182,219,218,218,170,90,169,84,82,146,36,73,146,36,149,74,85,213,90,107,109,219,182,109,173,173,
    85,85,85,170,164,164,36,73,74,74,42,149,106,85,181,214,218,218,214,182,214,90,181,170,42,85,74,41,41,41,
    41,165,84,169,170,170,90,173,181,214,218,90,107,181,170,90,165,170,148,74,41,165,148,82,169,82,85,85,173,
    90,173,213,90,171,213,106,85,85,85,85,42,85,74,165,82,169,82,85,170,86,85,173,106,181,90,181,106,85,171,
    170,170,74,85,169,82,149,42,85,169,170,170,170,170,90,85,171,106,85,173,170,170,90,165,170,170,42,85,85,
    170,170,82,85,85,85,85,181,170,170,170,86,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85
};

void cricket(void) {
    shiftOut(CRICKET_SAMPLES,cricketSample);
    delay(20);
    shiftOut(CRICKET_SAMPLES,cricketSample);
    delay(20);
    shiftOut(CRICKET_SAMPLES,cricketSample);
}

#define BIRD1_SAMPLES 1612

uint8_t birdSample1[BIRD1_SAMPLES] PROGMEM = {
    85,85,85,85,85,85,85,85,85,85,85,173,170,170,170,170,170,170,170,170,86,85,85,85,169,170,170,170,170,
    170,170,84,85,85,85,165,170,170,170,170,74,85,85,85,85,85,85,85,171,170,170,170,90,85,85,181,170,170,106,
    85,85,85,213,170,170,170,170,170,74,85,85,85,169,170,74,85,165,170,82,85,165,170,170,84,85,85,85,85,85,
    171,170,90,85,171,90,213,170,86,181,170,85,173,170,90,85,85,85,85,85,85,149,170,74,85,170,84,169,82,169,
    82,165,74,85,170,170,82,85,85,181,170,170,85,173,90,181,90,173,86,171,86,171,86,173,170,85,85,85,85,85,
    85,170,82,165,74,165,82,42,165,82,41,85,42,85,170,74,85,85,85,213,170,90,181,90,173,214,90,107,181,214,
    90,173,213,170,85,171,170,170,90,170,170,84,169,84,42,165,148,82,74,169,148,82,41,85,170,82,85,85,85,181,
    170,86,171,181,86,107,109,173,181,214,90,173,213,170,85,213,170,170,84,85,170,148,74,41,165,148,148,146,
    82,82,74,169,84,170,84,85,165,86,85,171,86,107,173,181,182,214,214,214,214,90,107,173,214,170,90,85,85,
    149,170,82,42,165,148,146,146,146,148,148,148,82,82,169,84,170,42,181,170,106,213,106,173,181,181,181,
    173,109,109,109,109,173,181,90,181,170,170,170,42,149,74,41,165,164,36,37,73,74,82,82,82,74,165,82,85,90,
    85,213,170,213,90,91,91,219,214,182,181,109,109,173,173,85,171,85,85,85,149,74,165,146,82,146,148,36,73,
    82,146,148,164,148,82,169,74,85,85,173,90,173,181,182,182,181,109,219,218,182,181,181,181,90,173,90,85,
    85,169,82,169,164,164,36,37,73,146,36,37,73,73,41,149,42,85,85,85,173,86,107,107,109,91,219,182,109,219,
    182,182,181,182,90,173,170,170,170,82,169,148,148,164,36,73,146,36,73,146,148,148,84,74,85,170,170,85,
    173,86,91,219,218,182,109,219,182,109,219,214,214,90,173,90,85,85,170,84,74,74,82,146,36,137,36,73,146,
    36,165,164,84,42,85,85,173,106,173,181,182,181,109,219,182,109,219,182,181,181,213,106,85,85,85,170,148,
    82,82,146,36,73,34,73,146,68,74,82,74,41,85,165,90,85,171,181,182,181,109,219,110,219,182,219,214,182,
    181,214,170,85,85,85,42,149,146,146,36,73,146,72,146,68,82,146,148,148,74,149,170,90,213,106,109,109,219,
    182,109,219,109,219,182,109,107,107,173,90,85,85,165,82,73,73,73,146,68,146,72,18,73,146,148,148,82,41,
    85,85,173,90,109,173,109,219,182,237,182,221,182,109,107,91,107,181,170,170,170,84,41,41,73,73,34,73,34,
    73,18,73,74,146,74,41,85,85,213,170,214,214,214,182,237,182,237,182,237,182,173,109,173,181,106,85,85,
    170,82,74,73,82,34,73,18,73,146,68,74,146,82,74,85,170,106,85,107,173,181,109,219,118,219,110,219,110,91,
    219,90,107,181,170,170,42,149,82,82,146,36,73,34,73,34,73,146,148,148,82,170,82,213,170,90,107,109,91,
    187,109,219,109,219,109,219,218,214,90,173,106,165,170,82,74,73,73,146,36,145,36,145,36,73,146,82,74,165,
    170,170,170,213,90,219,218,182,109,183,237,182,109,219,182,181,214,170,85,85,85,41,149,146,148,36,73,34,
    73,36,73,146,36,165,148,74,85,85,85,173,181,182,182,109,219,182,219,182,219,182,181,173,181,90,213,74,85,
    165,82,82,82,146,36,146,36,146,36,74,146,148,148,74,149,106,85,181,90,91,91,219,110,219,118,219,182,109,
    219,214,90,171,86,85,85,169,84,82,82,146,36,73,36,73,34,73,73,82,74,169,82,85,85,171,213,218,214,182,109,
    219,109,219,110,219,218,214,218,170,85,85,85,169,84,82,82,146,36,137,36,73,34,73,73,82,74,169,82,85,85,
    171,213,218,218,182,109,219,110,219,118,91,219,214,214,106,213,170,170,84,42,37,37,73,146,36,73,36,73,
    146,36,37,165,84,170,170,170,106,181,214,218,182,181,221,182,109,183,109,107,91,91,171,85,213,170,84,169,
    148,148,148,36,73,36,73,34,73,146,146,146,74,169,170,82,171,90,173,173,109,109,219,182,219,182,109,219,
    218,218,106,173,170,90,169,74,165,148,148,36,73,146,36,73,146,36,41,73,165,84,170,170,170,86,173,181,182,
    173,109,219,118,219,182,109,107,91,107,173,86,213,84,85,41,149,146,146,148,36,73,146,36,73,82,146,82,41,
    85,165,106,85,173,86,91,91,219,182,109,219,214,182,109,109,173,181,90,213,84,85,149,74,37,165,36,37,73,
    146,36,41,73,74,74,73,149,74,85,85,173,106,173,213,214,182,182,109,109,219,218,214,214,90,173,90,85,85,
    85,165,82,41,37,37,37,73,74,146,148,164,164,148,82,169,42,85,213,170,90,171,181,181,182,181,109,107,219,
    218,214,218,106,173,86,213,74,85,85,170,148,82,82,82,146,148,36,37,41,41,165,148,74,85,170,170,90,85,107,
    181,214,214,214,214,182,182,181,182,214,90,171,86,181,170,42,85,165,82,41,165,164,36,37,41,73,73,73,41,
    165,84,170,82,85,85,173,106,181,86,91,107,107,107,107,107,107,173,181,86,171,90,85,85,85,165,74,165,148,
    82,82,82,146,146,146,148,148,82,74,165,82,85,85,85,213,170,214,90,109,173,173,173,109,109,173,173,181,
    214,170,213,170,170,170,170,82,169,82,74,41,41,37,37,41,41,37,165,148,74,149,170,82,85,171,170,213,106,
    173,181,182,182,182,214,214,214,90,107,181,90,85,171,170,170,82,149,74,165,148,82,82,82,82,82,74,74,41,
    165,82,169,42,85,85,173,170,85,107,173,181,214,214,218,218,90,107,107,181,86,171,86,85,181,84,85,165,82,
    165,148,82,74,73,41,41,37,165,82,74,165,42,85,85,85,85,213,170,213,90,173,181,182,214,218,90,107,173,181,
    90,173,90,85,85,85,85,85,42,85,74,169,148,146,82,74,41,41,149,82,169,84,165,170,170,170,106,85,171,85,
    107,173,181,214,90,107,173,181,86,107,213,170,85,85,85,85,149,170,84,169,148,82,41,41,165,148,82,74,165,
    84,170,84,165,170,170,170,90,213,106,181,90,107,173,181,214,90,173,181,90,173,86,181,170,170,170,170,42,
    85,169,84,41,149,82,41,165,84,74,169,84,42,85,169,170,82,213,170,170,86,181,90,173,213,90,107,173,214,90,
    173,213,170,85,173,170,170,170,170,170,84,165,74,165,82,169,148,74,165,84,42,149,42,85,170,42,85,85,85,
    85,173,170,85,173,86,173,86,107,181,90,173,86,173,106,85,213,170,170,170,84,85,169,42,85,170,84,169,82,
    169,82,165,42,85,169,42,85,85,85,85,85,85,85,173,170,85,181,106,213,170,86,173,90,213,170,106,85,85,85,
    85,85,85,85,85,170,170,84,149,170,82,85,170,42,85,85,170,170,42,85,85,85,85,85,85,181,170,170,106,85,85,
    171,170,85,85,171,170,86,85,85,171,170,170,170,170,170,170,170,170,42,85,85,85,165,170,170,74,85,85,149,
    170,170,170,74,85,85,85,85,85,85,85
};

#define BIRD2_SAMPLES 5447

uint8_t birdSample2[BIRD2_SAMPLES] PROGMEM = {
    170,170,170,170,170,170,170,170,170,170,170,170,170,170,82,85,85,85,85,85,85,85,85,85,85,85,85,85,85,
    85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,
    85,85,85,85,85,85,85,85,85,85,85,85,181,170,170,170,170,106,85,85,85,85,85,85,171,170,170,170,170,170,74,
    85,85,85,85,85,169,170,170,170,42,85,85,85,85,85,85,85,85,85,213,170,170,170,90,85,85,173,170,170,85,85,
    213,170,170,170,106,85,85,85,85,85,85,149,170,170,170,82,85,85,169,170,74,85,85,85,170,170,170,170,106,
    85,85,213,170,106,85,173,106,85,171,106,85,171,170,85,85,181,170,170,170,170,84,85,165,170,84,165,74,85,
    170,84,169,42,85,165,170,170,170,170,170,90,85,173,90,213,106,213,106,181,106,181,106,213,170,90,85,213,
    170,170,82,85,85,170,82,165,74,165,82,169,84,42,149,74,85,170,82,85,85,85,85,173,170,85,171,86,171,213,
    106,181,90,173,86,173,170,85,85,181,82,85,85,169,74,165,74,169,84,74,165,84,74,165,82,165,42,85,85,85,85,
    85,181,170,86,173,86,171,213,90,173,86,171,213,170,86,181,170,170,170,170,170,74,85,170,84,169,84,74,165,
    84,74,165,82,169,84,169,74,85,85,85,85,85,171,90,181,90,173,214,106,173,214,106,181,90,181,170,86,85,85,
    85,85,170,82,165,82,41,149,82,74,165,148,82,41,85,42,85,165,170,170,170,106,85,171,86,171,181,90,107,173,
    214,90,173,213,170,85,171,170,170,170,170,42,85,170,84,42,165,84,74,41,165,148,74,169,84,170,84,85,85,85,
    85,181,106,181,90,107,173,181,182,214,90,107,173,181,106,181,170,106,165,170,42,85,41,149,82,74,41,41,37,
    37,165,148,82,41,85,169,170,82,173,170,86,173,214,106,109,173,181,181,181,182,214,90,171,213,170,85,85,
    85,85,149,42,85,74,169,164,148,146,82,74,74,41,165,82,169,84,85,170,170,85,213,106,181,214,106,107,109,
    109,109,173,173,181,214,106,181,170,85,85,85,85,170,82,41,165,148,82,82,82,82,74,74,41,165,82,169,42,85,
    85,181,170,85,107,181,214,218,90,91,107,107,173,181,86,171,85,171,170,170,170,170,84,42,149,82,74,74,73,
    73,73,41,165,148,82,169,84,85,106,85,85,173,86,171,181,181,182,182,182,182,182,182,214,90,173,86,173,170,
    170,170,84,169,148,82,74,74,82,146,148,164,164,36,149,82,170,84,85,169,86,213,106,173,182,214,214,214,
    182,182,181,181,181,214,90,173,170,85,85,165,42,149,74,41,41,41,73,74,146,146,148,148,82,74,169,82,85,85,
    85,181,106,181,214,90,219,218,214,182,182,182,181,214,90,173,86,85,85,85,149,42,165,148,82,82,146,148,36,
    37,41,73,41,165,84,42,85,85,85,181,106,181,214,90,219,218,182,182,173,109,107,109,173,214,170,86,85,85,
    149,42,149,82,74,74,74,146,146,36,37,37,37,165,84,169,84,85,85,171,90,173,181,181,182,109,109,219,182,
    173,109,109,173,181,106,85,85,85,149,42,165,148,164,36,73,146,164,36,73,146,82,82,170,84,170,170,90,213,
    90,107,109,107,219,182,109,219,182,109,107,91,171,181,106,85,85,85,42,149,146,82,146,148,36,73,146,36,41,
    73,41,165,84,169,170,170,90,181,90,107,91,219,182,109,219,182,181,109,107,107,173,213,170,106,169,170,82,
    42,37,37,41,73,146,36,73,146,36,37,41,165,82,149,170,90,213,106,109,173,109,219,182,237,182,109,219,182,
    181,182,86,173,170,170,170,84,74,41,73,73,146,68,146,36,73,146,36,37,149,82,149,170,106,85,107,173,181,
    109,109,187,109,219,182,109,91,219,106,173,85,213,82,85,169,84,82,74,146,36,73,146,36,73,82,146,82,82,
    169,82,85,85,173,86,107,109,109,219,182,109,219,182,173,109,107,173,213,170,85,85,169,82,41,165,36,41,73,
    36,73,146,36,73,74,74,42,165,170,84,181,170,214,90,107,107,219,182,109,219,182,109,107,91,107,173,86,181,
    42,85,165,82,41,41,73,74,146,36,73,146,36,73,73,41,149,74,85,85,181,106,173,181,182,109,219,182,109,219,
    182,109,107,107,173,86,171,170,74,85,74,41,73,73,146,36,146,36,146,36,73,73,74,165,84,85,85,173,90,107,
    219,218,118,219,118,219,110,219,182,173,173,214,170,86,149,170,148,82,82,146,36,137,36,145,36,145,148,36,
    165,148,42,85,85,171,90,107,109,219,182,109,187,109,183,109,219,182,181,214,170,85,85,85,42,165,148,164,
    68,146,36,145,36,146,36,73,74,73,165,74,85,181,106,181,182,182,109,219,110,219,109,183,109,219,182,182,
    90,173,170,170,74,165,148,148,36,73,36,145,68,146,72,146,148,148,84,42,85,85,171,213,214,214,182,219,118,
    187,237,118,219,110,107,219,106,173,170,86,169,82,73,73,137,36,73,36,145,72,34,73,146,148,148,74,85,169,
    86,173,214,214,182,109,183,221,118,187,237,182,109,107,107,173,106,85,165,74,41,41,73,146,72,34,145,68,
    18,73,146,148,84,74,85,85,181,106,109,109,219,182,219,110,183,219,110,219,182,181,181,90,85,85,85,42,37,
    41,73,18,73,36,146,72,36,73,146,146,82,169,170,170,106,181,214,182,118,219,118,187,237,182,219,182,109,
    109,173,86,85,85,85,42,41,41,137,36,137,68,34,145,72,18,37,73,169,148,170,170,90,173,182,181,237,182,221,
    237,118,187,237,182,109,107,171,213,170,170,84,74,73,82,36,145,136,68,36,34,146,72,146,148,84,74,85,85,
    107,181,182,109,187,221,110,119,183,187,237,182,109,109,173,86,85,85,41,165,36,41,146,72,36,34,17,137,36,
    146,164,36,85,42,181,170,90,107,109,219,118,219,237,110,183,219,109,219,182,182,86,181,42,85,41,165,36,
    73,36,34,17,17,137,72,36,145,148,164,84,169,170,90,173,181,109,219,221,238,238,118,119,119,219,110,107,
    107,181,170,170,84,73,74,146,72,36,34,34,17,145,136,36,73,74,41,85,85,173,213,214,182,221,110,119,119,
    183,187,219,237,214,182,213,90,85,85,41,165,36,73,36,18,17,145,136,136,68,18,37,73,169,84,85,85,107,173,
    109,219,110,183,187,187,219,221,110,187,173,109,181,106,149,170,82,82,146,36,18,145,136,136,136,68,36,
    145,164,164,84,169,106,85,107,107,187,109,119,119,119,119,183,187,237,182,181,181,106,85,165,74,73,74,34,
    145,136,136,72,68,36,34,73,146,74,169,170,170,214,90,219,110,187,219,221,221,221,221,109,183,109,109,181,
    106,169,42,37,37,73,34,17,137,136,136,72,68,146,36,41,41,85,85,213,170,173,109,219,110,183,187,221,221,
    110,183,109,91,171,181,170,170,148,82,146,36,145,136,136,136,68,68,36,145,36,37,149,42,171,106,173,173,
    219,110,119,119,119,239,238,238,118,187,181,181,85,181,82,169,36,37,145,136,136,136,16,33,34,34,146,36,
    41,37,85,85,107,173,109,219,237,238,222,189,123,119,119,183,109,219,90,173,170,84,73,73,18,137,8,17,33,
    132,8,17,17,137,164,36,85,170,106,213,182,182,219,221,221,187,247,222,189,187,219,109,91,91,213,170,84,
    74,73,34,17,17,17,66,132,16,17,17,137,36,165,148,170,85,173,173,221,110,247,238,222,123,239,221,187,221,
    109,107,171,85,85,169,36,41,18,17,33,66,8,65,132,16,145,136,148,148,74,85,173,214,182,109,239,238,189,
    247,222,251,238,221,237,182,173,181,170,170,148,146,36,18,33,66,8,65,8,33,68,68,34,73,169,84,85,173,182,
    109,187,187,247,222,251,190,247,222,221,221,181,109,213,170,74,169,36,146,136,16,33,8,65,8,34,68,68,34,
    37,165,170,170,214,214,110,183,119,239,125,223,251,222,187,123,219,182,181,90,85,170,36,73,34,34,132,16,
    130,16,132,8,17,137,36,37,85,85,181,90,183,237,238,221,123,223,123,223,187,119,183,221,90,107,85,149,82,
    82,34,18,33,66,8,65,8,33,34,34,73,146,82,165,86,173,109,219,238,238,222,123,239,123,239,222,221,110,91,
    107,85,85,165,164,68,34,33,68,8,66,16,130,136,8,73,146,84,74,181,170,109,91,119,119,239,189,239,125,239,
    189,187,221,182,181,90,85,41,165,68,18,33,66,8,65,8,66,136,16,73,34,165,82,85,181,182,182,221,221,189,
    247,190,247,189,119,119,119,91,219,170,90,169,148,18,73,68,68,8,33,132,16,34,68,36,145,82,74,85,85,107,
    107,219,237,238,221,123,239,221,123,119,187,109,219,170,86,149,74,73,146,136,72,8,33,132,16,66,132,68,34,
    37,165,170,170,90,107,219,237,238,222,189,239,221,123,119,119,219,214,90,213,84,169,36,73,68,68,8,33,132,
    16,66,68,36,145,82,74,85,173,214,182,237,238,222,187,239,125,239,189,187,219,181,109,85,173,148,74,146,
    68,68,132,16,132,32,4,17,33,146,72,74,165,170,106,109,109,187,187,123,239,125,239,123,239,221,238,182,
    173,85,171,74,165,36,145,136,136,16,132,32,4,17,66,36,18,165,164,170,170,218,90,183,187,123,247,190,239,
    123,223,187,119,187,109,171,213,82,149,146,36,17,34,130,16,4,130,16,132,136,136,36,41,85,170,214,90,183,
    221,221,123,223,247,125,223,123,239,118,219,214,90,85,170,36,73,34,34,132,32,8,130,16,130,72,68,146,146,
    170,170,90,107,187,221,189,247,190,239,251,190,119,239,182,221,106,173,42,85,146,36,17,33,130,16,4,65,8,
    33,34,34,73,73,85,170,182,214,182,219,189,119,223,251,190,247,222,189,221,182,181,90,85,170,36,73,34,34,
    132,16,4,65,8,33,34,34,73,74,169,170,214,90,183,219,189,119,223,247,125,223,123,247,182,237,106,181,42,
    85,146,36,17,17,66,16,4,65,8,66,68,36,74,82,170,170,90,91,187,221,221,123,223,247,190,247,221,189,221,
    182,213,106,169,82,73,146,136,136,32,132,32,8,33,132,136,68,82,82,170,170,181,182,110,119,247,222,251,
    222,247,190,123,119,219,110,181,90,165,74,73,146,136,136,16,132,32,8,33,132,136,68,82,82,170,170,214,218,
    118,187,123,239,189,239,251,222,187,119,219,109,173,90,149,74,73,18,17,34,4,129,32,16,4,33,68,34,74,74,
    85,85,109,107,119,119,239,251,126,191,223,247,189,123,187,109,173,86,165,82,18,137,8,33,16,8,4,4,130,16,
    34,18,41,37,173,170,182,109,123,119,223,247,251,251,251,253,238,189,219,182,85,171,74,41,137,72,8,66,64,
    64,64,64,16,4,145,136,148,148,170,86,109,219,238,221,247,253,254,254,190,223,123,247,218,182,85,181,148,
    148,68,34,130,16,4,2,129,64,8,33,34,137,82,41,171,86,219,109,247,222,251,126,191,223,247,190,187,219,181,
    182,170,170,36,73,34,34,4,33,16,8,130,32,68,68,146,36,85,85,173,173,221,237,222,247,190,239,251,190,187,
    119,219,182,86,181,84,74,137,36,34,68,8,66,16,66,136,16,73,18,149,74,181,170,173,109,183,187,123,247,222,
    187,119,119,183,221,90,107,85,85,41,165,36,137,68,68,68,68,68,34,146,68,74,41,85,165,86,173,182,181,109,
    183,221,118,219,109,219,182,181,214,170,90,165,170,148,82,74,74,146,146,164,36,37,165,84,74,85,169,170,
    86,213,170,213,90,107,181,214,90,171,181,90,181,170,106,85,85,169,170,82,149,170,84,169,74,85,170,42,85,
    85,85,85,85,85,171,170,86,85,171,170,86,85,213,170,170,170,170,82,85,85,85,169,170,170,170,170,170,170,
    170,170,170,86,85,213,170,170,90,85,85,85,85,85,85,85,149,170,170,84,85,170,170,84,85,149,170,170,170,
    170,170,170,85,85,213,170,170,85,85,171,170,170,85,85,85,85,85,85,85,169,170,170,84,85,85,170,170,170,
    170,170,170,170,85,85,171,106,85,173,106,85,213,170,170,170,170,170,84,85,170,82,149,170,84,85,170,170,
    170,170,106,85,171,213,90,173,181,214,90,173,86,171,106,85,165,42,85,74,73,73,74,146,36,73,146,146,148,
    82,74,85,169,90,85,171,181,182,181,109,219,182,109,107,219,90,107,213,170,170,170,42,85,74,41,37,165,164,
    164,148,82,42,149,170,84,85,85,173,170,213,170,213,106,181,106,85,171,170,86,85,85,85,170,170,170,170,
    170,106,85,173,90,173,86,107,181,86,171,85,173,170,170,84,169,164,36,73,18,137,68,36,34,145,68,74,82,42,
    85,85,171,182,173,219,237,238,238,221,221,221,221,182,109,173,86,85,85,74,74,146,36,34,17,137,136,36,146,
    164,36,85,169,106,85,107,107,219,182,109,219,182,109,107,107,173,90,85,85,42,149,82,74,74,74,74,73,165,
    82,85,169,86,173,182,214,182,109,219,218,182,182,86,171,170,170,164,148,36,146,136,136,8,17,17,17,73,36,
    165,148,170,170,181,182,237,118,247,238,189,247,238,222,237,118,109,173,85,85,41,165,36,73,34,18,145,136,
    68,34,73,146,148,82,170,170,170,85,107,109,109,91,219,182,181,173,173,181,90,181,170,170,170,170,170,82,
    85,165,170,170,170,170,170,85,181,170,85,173,90,85,171,170,170,84,165,84,82,82,146,36,146,36,146,36,41,
    73,165,82,85,85,107,173,109,219,118,187,221,237,118,187,109,219,106,173,170,170,74,41,73,74,36,145,72,36,
    146,36,73,74,169,82,85,173,90,107,91,219,118,219,182,237,218,182,213,90,171,170,170,82,165,148,146,146,
    148,164,148,148,82,165,170,170,170,213,90,91,219,214,182,109,91,219,90,173,86,85,169,84,146,148,72,68,34,
    34,66,68,34,145,164,36,85,165,86,173,181,109,183,187,187,123,119,119,183,219,181,173,85,171,42,85,82,146,
    36,145,72,36,18,137,36,73,74,74,85,169,90,181,218,90,219,182,109,219,182,109,107,107,173,86,85,173,84,
    149,74,169,148,82,74,165,84,169,82,85,85,85,171,86,171,181,86,107,181,90,181,170,170,82,165,148,146,36,
    73,34,137,36,146,36,73,73,169,84,85,181,218,90,187,109,183,219,221,237,118,219,181,109,181,90,85,170,82,
    74,146,36,73,36,73,36,73,73,42,165,170,170,170,213,90,91,219,182,182,109,107,91,107,181,106,85,85,165,74,
    165,148,82,74,41,165,84,170,82,85,181,106,213,218,90,91,219,218,218,106,173,85,85,85,169,146,146,36,137,
    68,36,34,34,146,72,146,148,82,149,85,171,109,219,238,238,222,123,239,189,119,119,219,109,173,86,85,169,
    36,37,17,145,16,33,66,132,68,68,146,146,82,165,106,85,91,219,182,221,110,119,187,219,109,183,109,109,171,
    213,170,170,82,165,82,74,41,41,37,165,148,82,169,84,169,170,170,170,170,170,106,85,213,170,170,170,85,
    165,170,170,82,85,170,84,165,42,85,85,106,85,85,173,90,173,181,214,90,107,107,181,86,171,85,85,85,85,169,
    84,74,74,73,82,146,36,41,73,41,165,84,165,106,85,173,181,182,109,219,110,219,109,187,109,219,182,214,90,
    173,170,170,170,148,82,82,146,36,145,36,146,72,146,36,73,42,165,82,85,213,170,182,214,182,109,219,110,
    219,237,182,109,91,219,106,181,42,85,165,164,36,145,68,68,68,68,68,68,36,145,164,164,84,165,90,181,218,
    214,118,219,221,221,221,221,221,221,109,219,182,214,170,170,74,165,36,73,18,137,136,136,136,136,72,68,
    146,36,149,74,85,85,107,107,219,110,119,119,239,222,189,187,187,221,214,182,170,85,165,82,146,36,17,145,
    16,34,132,16,17,34,18,73,146,82,170,170,106,181,182,109,183,187,187,187,187,187,219,237,182,181,213,106,
    85,170,82,82,146,72,34,34,68,132,136,16,137,72,146,148,82,85,213,106,109,219,221,221,189,247,222,251,238,
    238,109,187,181,214,82,85,41,73,17,137,16,33,4,33,132,16,17,137,164,164,82,85,107,173,237,118,247,238,
    189,239,189,239,221,221,109,219,90,181,82,165,36,145,8,17,2,65,16,8,132,16,34,17,165,148,170,90,109,219,
    222,221,247,251,254,254,126,223,123,119,219,182,170,106,74,82,68,68,8,130,128,64,32,8,34,68,146,36,85,85,
    181,181,221,221,125,223,239,223,223,239,247,222,221,110,107,181,74,165,36,137,8,33,8,8,8,8,4,130,8,17,37,
    41,85,85,219,182,123,247,254,254,254,247,223,191,239,189,219,110,181,170,146,82,68,136,128,128,0,32,0,8,
    32,16,68,34,74,41,171,213,118,187,247,251,253,191,255,223,255,254,189,247,214,182,85,85,146,36,33,132,0,
    1,16,0,8,128,32,8,145,36,149,170,182,181,123,239,239,127,255,255,255,253,223,239,219,221,106,85,37,73,66,
    16,8,0,0,32,0,0,16,16,68,36,170,84,173,109,247,222,239,255,251,255,255,191,127,191,187,221,106,85,73,146,
    8,130,0,0,0,0,64,0,0,1,34,34,165,82,109,107,239,123,255,255,255,255,255,255,255,255,238,221,181,90,41,41,
    65,8,16,0,0,0,0,0,0,0,8,33,42,165,214,182,125,223,255,255,255,255,255,255,255,255,222,221,213,170,82,146,
    16,4,8,0,0,0,0,0,0,0,8,17,165,82,109,91,223,247,255,253,255,255,255,255,255,247,221,221,170,85,73,36,1,
    129,0,0,0,0,0,0,0,0,36,145,170,170,221,221,247,255,255,255,255,255,255,255,255,255,111,219,42,85,66,68,0,
    128,0,0,0,0,0,0,0,0,72,74,173,181,251,190,255,255,255,255,255,255,255,255,255,239,214,106,73,82,8,4,0,0,
    0,0,0,0,0,0,0,17,85,165,237,118,127,255,255,255,255,255,255,255,255,255,109,183,170,170,8,17,1,0,0,0,0,0,
    0,0,0,132,148,74,109,219,254,126,255,255,255,255,255,255,255,255,223,221,170,170,36,34,2,16,0,0,0,0,0,0,
    0,4,37,165,90,107,239,125,255,255,255,255,255,255,255,127,119,119,213,170,36,145,64,64,0,0,0,0,0,0,0,4,
    73,146,86,107,247,222,255,251,255,255,255,255,255,255,187,187,85,85,73,36,8,32,0,0,0,0,0,0,0,128,164,164,
    90,107,223,247,255,255,255,255,255,255,255,255,191,221,170,170,68,68,0,1,0,0,0,0,0,0,0,0,73,73,109,109,
    247,251,255,255,255,255,255,255,255,255,187,221,170,170,136,136,0,4,0,0,0,0,0,0,0,8,41,165,218,218,190,
    239,255,255,255,255,255,255,255,247,109,187,42,85,68,136,0,0,0,0,0,0,0,0,145,136,74,85,187,221,253,247,
    255,255,255,255,255,255,119,239,86,107,74,146,32,16,0,0,0,0,0,128,32,8,41,41,107,181,189,247,253,255,255,
    255,247,255,239,247,109,219,74,85,34,34,32,0,0,0,0,0,2,4,34,145,170,84,183,219,251,247,255,255,255,255,
    255,247,247,238,86,171,146,36,129,64,0,0,0,0,0,0,0,4,41,41,107,109,239,251,255,255,255,255,255,255,255,
    255,237,118,85,170,136,16,4,0,0,0,0,0,0,0,0,33,85,165,221,238,253,255,255,255,255,255,255,255,255,189,
    173,181,148,36,129,128,0,0,0,0,0,0,0,0,36,73,181,90,223,251,255,255,255,255,255,255,255,255,183,187,85,
    85,18,17,1,32,0,0,0,0,0,0,0,32,42,165,182,237,254,253,255,255,255,255,255,255,255,239,181,181,82,74,4,65,
    0,0,0,0,0,0,0,0,73,146,170,85,239,189,255,255,255,255,255,255,255,255,237,182,165,170,16,17,4,0,0,0,0,0,
    0,0,34,34,170,42,183,221,253,223,255,255,255,255,255,255,255,255,90,107,73,74,8,4,0,0,0,0,0,0,0,0,18,137,
    170,170,221,221,223,255,255,255,255,255,255,255,187,187,85,181,72,34,2,4,0,0,0,0,0,32,8,33,41,149,182,
    173,247,125,255,255,255,255,255,255,222,247,218,90,37,165,32,4,1,0,0,0,64,0,8,65,146,164,106,181,238,222,
    247,223,255,255,247,255,251,190,219,182,165,170,136,68,32,64,0,0,32,0,8,4,145,72,170,170,109,219,125,223,
    255,254,255,254,239,239,219,221,90,213,36,73,130,32,64,0,0,1,64,128,16,17,149,42,91,219,222,251,254,191,
    255,255,239,255,222,189,173,181,82,82,132,16,128,0,0,0,0,0,65,16,74,146,214,106,247,238,191,255,255,255,
    255,255,191,127,183,219,170,170,136,68,64,0,0,0,0,0,0,2,68,132,82,149,109,219,126,191,255,255,255,255,
    255,251,239,189,107,107,165,148,8,17,32,0,0,0,0,0,2,1,73,18,171,170,221,221,247,223,255,255,255,255,251,
    239,219,221,90,85,73,146,64,16,0,16,0,0,64,0,17,33,165,74,219,218,125,191,255,255,247,255,255,239,247,
    238,214,90,165,148,8,33,64,0,0,32,0,0,33,8,73,74,86,171,221,221,223,127,255,255,255,253,239,247,110,219,
    170,170,68,18,129,64,0,32,0,0,1,2,17,17,149,42,219,218,222,123,255,251,255,247,127,255,222,189,173,181,
    74,165,136,136,128,128,0,16,128,0,33,132,164,36,171,86,187,219,247,251,239,255,251,191,223,251,182,109,
    85,149,34,145,32,16,128,0,64,0,4,130,72,36,85,85,237,182,251,125,255,191,255,255,253,251,221,221,85,171,
    82,82,8,33,128,0,0,0,1,0,65,8,73,146,106,85,119,119,191,127,255,255,255,251,127,127,119,183,213,170,146,
    164,32,4,2,0,1,0,128,0,66,8,41,73,181,90,119,119,255,253,255,223,255,255,251,253,118,219,170,170,18,73,8,
    2,1,0,16,0,32,128,8,17,149,82,109,173,123,239,239,255,254,255,223,255,189,247,182,182,42,85,34,34,4,4,0,
    1,0,32,64,64,68,36,169,42,219,214,189,239,247,127,255,255,247,127,223,123,91,219,84,165,68,36,8,8,128,0,
    0,4,32,32,68,34,169,74,91,219,222,251,254,223,255,255,223,255,125,223,109,91,85,165,36,34,4,4,0,1,0,0,4,
    8,68,68,42,149,182,181,123,239,223,255,255,239,255,255,126,223,219,182,85,85,18,73,16,8,128,0,0,0,8,0,33,
    8,41,73,213,170,221,110,127,191,255,223,255,255,253,251,221,189,173,181,82,41,145,136,128,128,0,64,0,32,
    32,16,34,18,149,170,218,90,239,221,239,223,255,247,255,254,125,223,109,183,86,85,73,146,8,33,16,64,0,4,
    32,64,8,33,74,146,170,86,237,182,123,239,251,251,247,223,239,247,221,221,182,214,42,85,146,36,66,8,2,4,8,
    32,32,16,66,68,146,146,170,85,219,182,123,239,253,253,251,239,239,247,187,123,219,218,170,170,36,73,66,
    132,64,64,128,0,129,128,8,17,73,82,170,170,181,109,119,239,126,223,223,223,239,247,221,187,219,218,170,
    86,73,41,145,136,16,66,32,8,4,65,132,8,73,146,82,169,90,181,182,221,238,238,222,123,239,189,119,119,187,
    109,173,213,84,85,82,146,68,34,34,68,136,16,17,17,137,36,165,148,170,170,181,214,182,109,183,187,221,237,
    110,183,221,182,214,214,170,170,170,74,165,148,148,164,36,73,146,36,37,37,165,84,170,84,85,85,85,171,90,
    173,214,90,107,173,181,214,90,107,181,90,213,170,170,90,169,170,82,165,74,165,82,42,165,82,42,149,74,165,
    42,85,169,170,170,170,170,90,85,171,86,173,86,171,213,106,181,90,173,90,213,170,106,85,85,85,165,170,82,
    165,82,41,149,82,74,165,84,42,149,170,84,85,85,213,170,86,171,213,90,107,173,181,214,106,173,90,181,170,
    170,170,170,74,149,42,165,82,41,165,82,41,85,42,85,169,170,170,170,170,170,90,85,171,90,181,106,213,170,
    85,171,90,85,171,170,90,85,85,85,169,170,170,84,149,170,82,149,170,82,85,170,42,85,85,85,85,85,85,85,85,
    85,171,170,170,86,85,213,170,170,106,85,85,85,171,170,170,170,85,85,85,85,85,85,85,85,169,170,42,85,149,
    170,82,85,169,170
};

void bird(uint8_t repetitions) {
    while(repetitions--) {
        shiftOut(BIRD1_SAMPLES,birdSample1);
        delay(63);
        shiftOut(BIRD2_SAMPLES,birdSample2);
        if(repetitions) {
            // insert a random delay from 100-355 ms
            nextRandom();
            delay(100+(randomValue&0xff));
        }
    }
}

/***
#define TEST_PATTERN_SAMPLES 4156

uint8_t testPattern[TEST_PATTERN_SAMPLES] PROGMEM = {
    170,170,85,85,85,85,85,85,85,85,171,170,170,106,85,85,181,170,170,85,85,171,170,85,213,170,90,85,171,
    90,213,170,86,181,106,213,170,86,171,86,173,90,173,86,173,86,171,213,106,181,90,173,213,106,173,214,106,
    173,214,90,173,181,214,106,173,181,214,90,107,173,173,181,214,214,90,91,107,107,109,173,173,173,173,173,
    173,173,173,109,109,109,107,91,219,214,182,182,173,109,91,219,182,181,109,219,214,182,109,219,182,109,
    219,182,109,219,182,109,219,182,237,182,109,219,110,219,118,219,118,219,118,219,110,219,109,183,221,118,
    219,109,183,219,110,183,219,110,183,219,237,118,183,219,237,110,183,187,221,237,110,119,183,187,219,221,
    221,238,238,238,118,119,119,119,119,119,119,119,119,119,119,239,238,238,222,221,221,187,123,119,247,238,
    221,189,123,247,238,221,187,119,239,222,187,247,238,189,119,239,189,119,239,189,247,222,123,239,189,247,
    222,123,223,123,239,123,239,125,239,123,239,123,223,247,190,247,125,239,251,190,247,125,223,247,125,223,
    247,125,223,247,251,190,239,247,253,190,223,247,251,126,191,239,247,251,253,126,191,223,239,247,251,253,
    253,126,127,191,191,191,223,223,223,223,223,223,223,223,223,223,191,191,191,127,255,254,254,253,251,247,
    239,223,191,255,254,253,251,239,191,255,254,251,239,191,255,254,247,223,255,254,247,191,255,253,239,127,
    255,247,127,255,247,127,255,247,255,254,239,255,253,127,255,239,255,251,255,254,191,255,239,255,247,255,
    251,255,251,255,253,255,251,255,251,255,247,255,223,255,127,255,255,253,255,239,255,255,254,255,239,255,
    255,253,255,127,255,255,191,255,255,191,255,255,127,255,255,255,253,255,255,239,255,255,255,253,255,255,
    255,254,255,255,255,251,255,255,255,223,255,255,255,255,239,255,255,255,255,127,255,255,255,255,255,255,
    254,255,255,255,255,255,255,254,255,255,255,255,255,255,255,247,255,255,255,255,255,255,255,255,255,255,
    253,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,223,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,255,255,255,255,255,
    255,255,255,255,255,255,239,255,255,255,255,255,255,255,255,223,255,255,255,255,255,255,255,253,255,255,
    255,255,255,239,255,255,255,255,255,253,255,255,255,255,251,255,255,255,127,255,255,255,255,253,255,255,
    255,253,255,255,127,255,255,255,247,255,255,191,255,255,127,255,255,127,255,255,191,255,255,223,255,255,
    251,255,127,255,255,247,255,191,255,255,253,255,247,255,223,255,191,255,127,255,127,255,127,255,191,255,
    191,255,223,255,247,255,251,255,254,223,255,247,255,254,223,255,251,127,255,247,127,255,247,127,255,247,
    191,255,251,223,255,254,247,223,255,254,247,223,127,255,253,247,223,127,255,253,251,239,223,191,127,255,
    254,253,251,247,239,223,223,191,191,191,127,127,127,127,127,127,127,127,191,191,191,223,223,239,239,247,
    247,251,253,126,191,223,239,247,251,253,126,223,239,247,253,190,223,247,251,190,223,247,125,223,247,251,
    190,239,251,222,247,125,223,247,190,239,125,223,251,222,247,190,247,189,247,189,247,189,247,222,247,222,
    123,239,189,247,222,123,239,189,123,239,189,123,239,221,123,247,222,189,123,239,222,189,123,119,239,222,
    221,187,123,119,239,238,238,221,221,189,187,187,187,187,187,187,187,187,187,187,187,187,187,221,221,221,
    238,238,118,119,187,219,221,238,118,183,187,221,110,119,187,221,110,183,219,109,183,219,109,183,219,110,
    187,237,182,219,118,219,109,219,109,187,109,219,109,219,110,219,182,109,183,109,219,182,109,219,182,109,
    219,182,109,219,218,182,109,91,219,182,173,109,91,219,214,182,181,173,109,109,107,91,91,219,218,218,218,
    218,218,90,91,91,107,107,109,173,181,181,214,90,91,107,173,181,214,90,171,181,214,90,173,181,90,171,181,
    90,171,181,90,173,86,171,213,106,181,90,173,90,173,90,173,90,181,106,85,171,86,173,106,85,171,106,85,171,
    106,85,181,170,90,85,181,170,170,85,85,181,170,170,170,86,85,85,85,85,85,171,170,170,170,170,170,170,42,
    85,85,85,85,85,149,170,170,170,82,85,85,170,170,82,85,165,170,82,85,169,42,85,169,42,85,169,74,149,42,85,
    169,82,165,74,165,74,165,74,165,82,169,84,42,149,74,165,82,42,165,82,42,165,82,74,165,148,82,42,165,148,
    82,74,41,37,165,148,82,82,74,73,41,41,37,37,165,164,164,164,164,164,36,37,37,41,73,73,74,82,146,148,36,
    37,73,74,146,36,37,73,146,164,36,73,146,36,73,146,36,73,146,36,73,18,73,146,36,137,36,73,36,73,34,73,36,
    73,36,145,36,146,72,34,137,36,18,73,36,18,73,36,18,137,68,34,17,137,68,34,18,145,136,68,36,34,17,145,136,
    136,68,68,68,34,34,34,34,34,34,34,34,34,34,34,34,34,66,68,68,136,136,8,17,33,34,68,132,8,17,33,66,132,8,
    33,66,132,16,33,68,8,33,66,8,33,66,8,33,132,16,66,8,33,132,16,132,16,66,16,66,16,66,16,130,16,132,32,4,
    65,8,130,16,4,65,16,132,32,8,130,32,16,4,65,16,4,130,32,16,4,130,64,16,8,4,129,64,32,16,8,4,2,129,64,32,
    16,16,8,8,4,4,2,2,2,1,1,1,1,1,1,1,1,2,2,2,4,4,8,16,32,64,128,0,1,2,4,8,32,64,0,1,4,16,64,0,1,4,16,128,0,
    4,16,128,0,4,32,0,2,16,0,1,16,0,1,16,0,1,32,0,4,128,0,16,0,4,128,0,32,0,16,0,4,0,2,0,2,0,1,0,1,0,1,0,2,0,
    4,0,16,0,64,0,0,2,0,16,0,0,1,0,32,0,0,4,0,0,2,0,0,1,0,0,1,0,0,2,0,0,16,0,0,0,1,0,0,64,0,0,0,64,0,0,0,0,1,
    0,0,0,32,0,0,0,0,64,0,0,0,0,0,8,0,0,0,0,0,64,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,128,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,64,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,128,0,0,0,0,0,0,128,0,0,
    0,0,0,0,1,0,0,0,0,8,0,0,0,0,4,0,0,0,32,0,0,0,128,0,0,0,64,0,0,0,8,0,0,64,0,0,0,1,0,0,2,0,0,2,0,0,1,0,64,
    0,0,8,0,128,0,0,8,0,64,0,0,1,0,4,0,16,0,32,0,32,0,64,0,32,0,32,0,16,0,8,0,2,128,0,32,0,8,0,1,64,0,8,128,
    0,16,0,1,16,0,1,16,0,1,8,64,0,2,16,128,0,4,16,128,0,2,8,32,128,0,2,8,32,64,128,0,2,4,8,16,32,64,128,128,
    0,1,2,2,2,4,4,4,4,4,4,4,4,4,4,2,2,2,1,129,64,64,32,16,8,4,2,129,64,32,16,8,2,129,32,16,4,130,64,16,8,130,
    32,16,4,65,16,4,65,16,4,65,16,130,32,8,65,16,130,16,4,33,8,33,8,65,8,33,8,33,4,33,132,16,66,8,33,132,16,
    66,8,17,66,8,17,66,136,16,34,132,8,17,34,68,136,16,33,66,68,136,16,17,33,34,68,68,132,136,136,8,17,17,17,
    17,17,17,17,17,17,17,145,136,136,136,68,68,36,34,18,17,137,72,68,34,18,137,72,36,18,145,72,36,18,137,36,
    18,137,36,18,73,36,145,68,18,73,36,137,36,145,36,145,36,145,36,137,36,73,146,72,146,36,73,146,36,73,146,
    36,73,146,36,73,146,148,36,73,82,146,36,37,73,74,146,146,148,36,37,41,73,73,73,74,74,74,74,74,74,74,74,
    73,41,41,37,165,148,148,82,74,74,41,165,148,82,74,169,148,82,74,165,148,74,169,148,74,169,84,74,165,82,
    169,84,42,149,74,149,74,165,74,149,42,149,170,84,169,82,149,170,84,165,42,85,165,170,84,85,170,42,85,85,
    170,170,82,85,85,169,170,170,42,85,85,85,85,85,85,85,85,170,170,85,85,85,85,85,173,170,170,85,85,171,106,
    85,173,106,213,170,85,171,86,171,86,171,213,90,173,214,90,173,181,214,90,107,173,181,214,214,218,218,218,
    218,218,218,214,182,182,181,109,107,219,182,173,109,219,182,109,219,109,219,182,221,182,221,182,219,110,
    187,237,118,219,237,118,187,221,237,110,119,187,187,221,221,237,238,238,238,238,238,221,221,221,187,123,
    119,239,222,189,123,247,222,189,247,222,187,247,222,247,222,123,239,123,223,251,222,247,190,239,251,190,
    239,251,190,239,247,253,190,223,239,251,253,126,127,191,223,223,239,239,239,239,239,239,223,223,191,127,
    255,254,253,247,239,191,255,254,251,223,255,254,247,191,255,251,191,255,247,255,254,223,255,247,255,251,
    255,253,255,254,255,253,255,247,255,223,255,255,253,255,191,255,255,223,255,255,223,255,255,255,253,255,
    255,255,254,255,255,255,239,255,255,255,255,127,255,255,255,255,255,255,127,255,255,255,255,255,255,255,
    255,255,255,255,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,127,255,255,255,255,255,255,255,255,255,255,255,255,255,253,255,255,255,255,255,255,
    223,255,255,255,255,255,247,255,255,255,255,254,255,255,255,254,255,255,239,255,255,223,255,255,239,255,
    255,251,255,191,255,255,253,255,251,255,247,255,247,255,251,255,253,127,255,239,255,253,191,255,251,191,
    255,251,223,255,254,251,223,127,255,253,251,239,223,191,127,255,254,254,253,253,253,253,253,253,253,126,
    127,191,223,239,247,251,125,191,239,247,253,190,239,251,190,239,251,222,247,189,239,125,239,125,239,189,
    247,190,119,239,189,247,222,189,119,239,222,189,123,247,238,238,221,221,189,187,187,187,187,187,187,221,
    221,237,238,118,183,187,221,110,183,219,237,118,219,109,183,221,182,219,182,219,182,237,182,109,219,182,
    109,219,182,173,109,219,218,182,181,173,109,109,109,107,107,109,109,173,173,181,214,90,107,173,181,86,
    107,173,214,106,181,90,173,86,171,86,173,90,181,170,85,173,170,85,213,170,170,85,85,85,171,170,170,170,
    170,170,170,170,170,42,85,85,85,170,170,84,85,170,74,85,170,82,165,74,149,42,149,74,165,82,169,148,74,41,
    149,82,74,41,165,148,82,74,74,73,73,41,41,73,73,73,74,82,146,164,36,73,74,146,36,73,146,36,73,146,72,146,
    36,146,36,146,68,18,73,36,145,72,36,18,137,68,34,18,145,136,72,68,68,34,34,34,34,34,34,66,68,68,136,136,
    16,33,66,132,8,17,66,132,16,66,8,17,130,16,66,8,65,8,65,8,66,16,132,32,8,130,32,8,130,64,16,8,2,65,32,16,
    8,4,2,1,129,64,64,64,64,64,64,64,128,128,0,1,2,4,8,32,64,0,1,4,32,128,0,4,32,0,2,32,0,2,64,0,8,0,1,64,0,
    32,0,16,0,16,0,32,0,64,0,0,2,0,32,0,0,8,0,0,4,0,0,8,0,0,128,0,0,0,128,0,0,0,0,16,0,0,0,0,0,4,0,0,0,0,0,0,
    64,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,8,0,0,0,128,0,0,0,64,0,0,
    0,4,0,0,4,0,0,2,0,64,0,0,4,0,16,0,64,0,128,0,64,0,32,0,16,0,4,128,0,16,0,2,32,0,2,16,128,0,4,32,128,0,2,
    8,16,64,128,0,1,2,4,4,8,8,8,8,8,8,4,4,2,1,129,64,32,8,4,130,64,16,8,130,32,8,130,32,8,130,16,132,32,4,33,
    8,33,132,16,132,16,34,132,16,66,132,16,33,66,132,8,17,33,34,68,68,68,136,136,136,136,136,72,68,68,34,34,
    17,137,72,68,34,145,72,36,145,72,34,137,36,146,68,146,68,146,36,73,36,73,146,36,73,74,146,36,41,73,82,
    146,146,148,164,164,164,164,164,164,148,148,82,74,41,165,148,82,74,165,148,74,165,84,42,149,42,149,42,85,
    170,84,169,74,85,169,42,85,85,170,170,74,85,85,85,85,85,170,170,85,85,85,181,170,90,213,170,85,171,213,
    106,181,86,107,173,181,181,182,214,182,182,181,109,107,219,182,109,219,109,219,109,187,237,118,187,221,
    237,110,119,119,119,119,119,247,238,222,189,119,239,189,247,222,123,223,251,222,247,125,223,247,251,126,
    191,223,239,239,239,239,239,223,223,127,255,254,251,223,255,254,239,255,254,191,255,239,255,239,255,223,
    255,255,253,255,255,254,255,255,247,255,255,255,127,255,255,255,255,255,255,255,253,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,251,255,255,255,255,255,255,255,251,255,255,255,255,254,255,255,239,255,255,247,255,127,255,255,
    253,255,253,127,255,223,255,251,191,255,253,247,191,127,255,254,253,251,251,251,251,251,253,126,191,239,
    247,125,223,247,125,239,123,239,125,239,189,123,239,222,189,123,119,119,239,238,110,119,119,187,219,237,
    118,219,237,182,221,182,109,219,182,109,219,182,182,173,173,173,173,173,181,214,90,171,181,90,173,86,171,
    86,181,170,85,85,171,170,170,170,170,170,170,42,85,85,170,82,149,42,149,74,165,82,42,165,148,82,74,74,74,
    74,74,146,146,36,73,146,36,73,146,68,146,72,36,145,72,36,34,17,17,137,136,8,17,17,33,66,132,8,33,66,8,65,
    8,33,8,65,16,4,65,16,8,2,129,64,32,32,32,32,32,64,128,0,1,2,16,64,0,2,32,0,4,0,1,64,0,64,0,0,1,0,16,0,0,
    8,0,0,128,0,0,0,0,32,0,0,0,0,0,0,0,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    64,0,0,0,0,0,0,0,1,0,0,0,16,0,0,128,0,0,64,0,0,4,0,8,0,8,0,2,128,0,8,128,0,4,32,128,0,1,4,4,8,8,8,8,8,4,
    2,129,32,16,4,65,16,132,32,4,33,132,16,66,8,17,66,132,136,16,17,17,17,17,17,137,72,68,34,145,72,34,73,36,
    73,36,73,146,36,41,73,82,146,146,148,146,82,82,74,41,149,82,169,84,42,85,170,84,165,170,82,85,85,85,170,
    170,85,85,173,90,181,90,107,173,181,181,181,109,219,182,109,187,221,118,119,187,187,187,119,239,222,123,
    223,251,190,239,251,253,254,126,255,254,253,247,127,255,247,255,253,255,247,255,255,251,255,255,255,191,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,251,255,255,255,255,
    251,255,255,251,255,239,255,251,127,255,251,223,191,127,191,191,223,239,251,125,239,123,239,189,119,239,
    238,238,238,238,118,187,109,183,109,219,182,181,173,181,181,214,106,181,106,85,171,170,170,170,170,42,85,
    169,82,169,148,82,82,74,82,146,36,73,18,73,34,145,136,136,136,136,8,17,66,8,33,8,65,32,8,4,2,2,1,2,4,32,
    0,1,32,0,8,0,32,0,0,32,0,0,0,0,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,32,0,0,16,0,64,0,16,
    0,1,16,64,128,0,129,128,64,32,8,130,32,4,33,132,8,17,34,34,34,17,145,68,34,73,146,36,73,82,82,82,74,41,
    165,82,165,74,85,85,170,170,85,173,214,90,219,218,118,219,238,110,239,222,123,223,247,251,253,251,239,
    255,254,255,253,255,255,255,255,247,255,255,255,255,255,255,255,255,255,255,255,251,255,223,255,253,239,
    223,239,247,125,239,189,123,183,187,221,182,109,91,107,173,85,171,170,170,42,85,74,41,37,73,146,68,34,18,
    33,66,8,65,16,8,4,8,64,0,4,0,32,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,64,0,128,0,8,32,64,32,16,4,33,132,8,137,
    136,36,145,164,36,165,148,74,85,170,170,181,214,182,221,222,189,247,251,251,191,255,255,255,255,255,255,
    255,247,255,255,239,255,190,223,123,247,182,219,90,107,85,85,41,165,36,146,16,33,4,130,0,8,0,0,16,0,0,0,
    0,0,0,0,2,32,32,16,66,132,68,146,148,82,170,170,237,182,247,251,223,255,255,255,255,255,190,223,219,118,
    85,85,145,36,4,130,0,0,0,0,0,4,32,16,146,72,170,170,247,251,255,255,125,191,85,85,2,65,0,0,32,16,170,170,
    255,255,170,170,0,0,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,
    170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,
    170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170
};

void audioTest() {
    shiftOut(TEST_PATTERN_SAMPLES,testPattern);
}
***/