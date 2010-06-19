#include "utilities.h"
#include "wireless.h"
#include "random.h"

#include <math.h>
#include <avr/eeprom.h>

#include "WProgram.h" // arduino header

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
