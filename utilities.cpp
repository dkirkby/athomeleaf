#include "utilities.h"
#include "wireless.h"

#include <math.h>
#include <avr/eeprom.h>

#include "WProgram.h" // arduino header

// The tick() function should be called regularly during any long-running
// function to ensure continuous audio level feedback.
extern void tick();

// ---------------------------------------------------------------------
// Locally shared globals
// ---------------------------------------------------------------------
static uint8_t _u8val;
static uint16_t _u16val;
static float _fval;

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
            tick();
        }        
        offset++;
    }
}

// =====================================================================
// Print integer values in upper-case hex with zero padding. No
// globals are used and output is only via Serial.write()
// =====================================================================

#define PRINT_BYTE(C) Serial.write((uint8_t)(C))

void pprint(uint8_t value) {
    // print high-order nibble
    if((value >> 4) < 0x0A) {
        // hex digit is 0-9
        PRINT_BYTE((value >> 4) + '0');
    }
    else {
        // hex digit is A-F
        PRINT_BYTE((value >> 4) + 'A' - 10);
    }
    // print low-order nibble
    value &= 0x0F;
    if(value < 0x0A) {
        // hex digit 0-9
        PRINT_BYTE(value + '0');
    }
    else {
        // hex digit A-F
        PRINT_BYTE(value + 'A' - 10);
    }
}
void pprint(uint16_t value) {
    pprint((uint8_t)(value >> 8));
    pprint((uint8_t)value);
}
void pprint(uint32_t value) {
    pprint((uint8_t)(value >> 24));
    pprint((uint8_t)(value >> 16));
    pprint((uint8_t)(value >> 8));
    pprint((uint8_t)value);
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
// assumed to be 10-bit and packed accordingly. The first 11 bytes
// of the dump buffer provided will be included in the first packet
// sent (and overwritten as subsequent packets are sent). The dump
// buffer should already have its networkID field set (and this will
// not be overwritten). In total, 11 packets will be sent.
// ---------------------------------------------------------------------
void dumpBuffer(uint8_t dumpType, BufferDump *dump) {
    uint16_t src[4];
    // the first packet can be identified by its zero sequence number
    dump->sequenceNumber = 0;
    // skip over the first 11 bytes of packed data then...
    // ...send the dump type, and timestamp
    dump->packed[11] = dumpType;
    *(uint32_t*)(&dump->packed[12]) = timestamp;
    // ...the first two samples are unpacked
    *(uint16_t*)(&dump->packed[16]) = WAVEDATA(0);
    *(uint16_t*)(&dump->packed[18]) = WAVEDATA(1);
    // ...the next 8 samples are packed into the remainder of the buffer packet
    src[0]= WAVEDATA(2); src[1]= WAVEDATA(3); src[2]= WAVEDATA(4); src[3]= WAVEDATA(5);
    packSamples(src,&dump->packed[20]);
    src[0]= WAVEDATA(6); src[1]= WAVEDATA(7); src[2]= WAVEDATA(8); src[3]= WAVEDATA(9);
    packSamples(src,&dump->packed[25]);
    // try to send the first packet now
    if(0x0f < sendNordic(dumpAddress, (uint8_t*)dump, sizeof(BufferDump))) {
        // don't keep going if our first packet didn't get through
        return;
    }
    tick();
    // the remaining 10 packets use the same structure
    _u16val = 10;
    for(dump->sequenceNumber = 1; dump->sequenceNumber < 11; dump->sequenceNumber++) {
        src[0] = WAVEDATA(_u16val++); src[1] = WAVEDATA(_u16val++);
        src[2] = WAVEDATA(_u16val++); src[3] = WAVEDATA(_u16val++);
        packSamples(src,&dump->packed[0]);
        src[0] = WAVEDATA(_u16val++); src[1] = WAVEDATA(_u16val++);
        src[2] = WAVEDATA(_u16val++); src[3] = WAVEDATA(_u16val++);
        packSamples(src,&dump->packed[5]);
        src[0] = WAVEDATA(_u16val++); src[1] = WAVEDATA(_u16val++);
        src[2] = WAVEDATA(_u16val++); src[3] = WAVEDATA(_u16val++);
        packSamples(src,&dump->packed[10]);
        src[0] = WAVEDATA(_u16val++); src[1] = WAVEDATA(_u16val++);
        src[2] = WAVEDATA(_u16val++); src[3] = WAVEDATA(_u16val++);
        packSamples(src,&dump->packed[15]);
        src[0] = WAVEDATA(_u16val++); src[1] = WAVEDATA(_u16val++);
        src[2] = WAVEDATA(_u16val++); src[3] = WAVEDATA(_u16val++);
        packSamples(src,&dump->packed[20]);
        src[0] = WAVEDATA(_u16val++); src[1] = WAVEDATA(_u16val++);
        src[2] = WAVEDATA(_u16val++); src[3] = WAVEDATA(_u16val++);
        packSamples(src,&dump->packed[25]);
        // try to send this packet now
        if(0x0f < sendNordic(dumpAddress, (uint8_t*)dump, sizeof(BufferDump))) return;
        tick();
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
#define RMS_NORM 5.52427172802e-3 // sqrt(2)/NPOWERSAMP
#define RAD_TO_MICROS_60 2652.5823848649225 // 10^6/(2pi*60)
#define RAD_TO_MICROS_120 1326.2911924324612 // 10^6/(2pi*120)
#define POWER_CYCLE_MICROS 16666.6666667 // 10^6/60
#define POWER_CYCLE_MICROS_BY_2 8333.33333333333
#define POWER_CYCLE_MICROS_BY_4 4166.66666666667
#define POWER_CYCLE_MICROS_BY_8 2083.33333333333
#define MICROS_PER_SAMPLE 200
#define ONE_OVER_NPOWERSAMP 4e-3 // 1/250
#define ONE_OVER_NPOWERSAMP_SQ 16e-6 // 1/(250*250)

// ---------------------------------------------------------------------
// Lighting and power analysis shared globals
// ---------------------------------------------------------------------
static uint8_t cycle,nzero;
static float sink,cosk,cosSum,sinSum;
static uint32_t elapsed;

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

void lightingAnalysis(float scale, uint16_t delay, BufferDump *dump) {
    
    static float beta0,beta1,beta2,alpha00,alpha01,alpha02,alpha11,alpha12,alpha22;
    
    nzero = 0;
    alpha00 = NLIGHTSAMP;
    alpha11 = alpha22 = NLIGHTSAMPBY2;
    alpha01 = alpha02 = alpha12 = 0;
    beta0 = beta1 = beta2 = 0;
    for(_u8val = 0; _u8val < NLIGHTSAMPBY4; _u8val++) {
        if(_u8val == 0) {
            sink = 0;
            cosk = 1;
        }
        else {
            _fval = DPHI120*_u8val;
            sink = sin(_fval);
            cosk = cos(_fval);
            tick();
        }
        for(cycle = 0; cycle < 4; cycle++) {
            if((cycle%2) && (_u8val==0)) continue;
            switch(cycle) {
                case 0:
                _u16val = WAVEDATA(_u8val);
                break;
                case 1:
                _u16val = WAVEDATA(NLIGHTSAMPBY2-_u8val);
                break;
                case 2:
                _u16val = WAVEDATA(NLIGHTSAMPBY2+_u8val);
                break;
                case 3:
                _u16val = WAVEDATA(NLIGHTSAMP-_u8val);
                break;
            }
            if(_u16val > ADCMIN && _u16val < ADCMAX) {
                _fval = _u16val;
                beta0 += _fval;
                beta1 += _fval*cosk;
                beta2 += _fval*sink;
            }
            else {
                if(_u16val <= ADCMIN) nzero++;
                alpha00 -= 1;
                alpha01 -= cosk;
                alpha02 -= sink;
                alpha11 -= cosk*cosk;
                alpha12 -= sink*cosk;
                alpha22 -= sink*sink;
            }
            sink = -sink;
        }
        tick();
    }
    
    if(0 != dump) {
        /* zero out the dump header */
        for(_u8val = 0; _u8val < 15; _u8val++) dump->packed[_u8val] = 0;
        DUMP_ANALYSIS_SAVE(0,uint16_t,(uint16_t)(10*alpha00));
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
        tick();
    
        // Solve the linear equations alpha . x = beta for the vector of coeficients x.
        // Replace each beta(i) with x(i)
        beta0 /= alpha00;
        beta1 = (beta1 - alpha01*beta0)/alpha11;
        beta2 = (beta2 - alpha02*beta0 - alpha12*beta1)/alpha22;
        beta1 = (beta1 - alpha12*beta2)/alpha11;
        beta0 = (beta0 - alpha01*beta1 - alpha02*beta2)/alpha00;
        tick();
        
        // Calculate the delay (in us) of the current sampling compared
        // with the earlier voltage-fiducial sampling.
        if(timestamp > tzero) {
            elapsed = timestamp - tzero;
        }
        else {
            // handle the (unlikely) case where the micros counter rolled over
            elapsed = 0xffffffff - tzero;
            elapsed += timestamp;
        }

        // Calculate the raw phase (in us) of an equivalent 120 Hz
        // sine wave. Offset is relative to WAVEDATA(0)=sample[6].
        _fval = atan2(beta2,beta1)*RAD_TO_MICROS_120 - POWER_CYCLE_MICROS_BY_8;
        if(_fval < 0) _fval += POWER_CYCLE_MICROS_BY_2;
        if(0 != dump) {
            DUMP_ANALYSIS_SAVE(2,uint16_t,(uint16_t)(_fval+0.5));
        }
        tick();
        
        // Calculate the delay (in us) of the 120 Hz zero crossing relative
        // to the voltage zero crossing, modulus a 120 Hz cycle.
        _fval = fmod(_fval + elapsed - voltagePhase - delay,
            POWER_CYCLE_MICROS_BY_2);
        if(0 != dump) {
            DUMP_ANALYSIS_SAVE(4,uint16_t,(uint16_t)(_fval + 0.5));
        }

        // Store the 120 Hz peak amplitude in beta1
        beta1 = sqrt(beta1*beta1 + beta2*beta2);
        
        // save our analysis results (in rounded 16-bit ADC/10 counts)
        // in case this buffer gets dumped
        if(0 != buffer) {
            DUMP_ANALYSIS_SAVE(6,uint16_t,(uint16_t)(10*beta0));
            DUMP_ANALYSIS_SAVE(8,uint16_t,(uint16_t)(10*beta1));
        }
        
        // scale beta0 to lightingMean
        beta0 = scale*beta0 + 0.5;
        if(beta0 <= 0) {
            lightingMean = 0;
        }
        else if(beta0 >= 0xfffe) {
            lightingMean = 0xfffe;
        }
        else {
            lightingMean = (uint16_t)beta0;
        }
        tick();
        
        // scale beta1 to lighting120Hz
        beta1 = scale*beta1 + 0.5;
        if(beta1 <= 0) {
            lighting120Hz = 0;
        }
        else if(beta1 >= 0xfffe) {
            lighting120Hz = 0xfffe;
        }
        else {
            lighting120Hz = (uint16_t)beta1;
        }
        
    }
}

// =====================================================================
// Analyzes an AC current waveform to determine its RMS power at 60 Hz.
// Uses the gain (mW/ADC) and phase delay (us) provided.
// This function makes the following math calls: 62*sin, 62*cos, 1*sqrt.
//
// Returns the following values via globals:
//  nClipped: number of samples/250 clipped at the ADC hi/lo limits
//  currentComplexity: 0-255 measure of signal variance not at 60 Hz
//  apparentPower: calculated apparent power consumption in mW
//  zeroXingDelay: delay of the current zero crossing relative to
//    the voltage zero crossing in microseconds, modulo a 120 Hz cycle.
// =====================================================================

uint8_t nClipped,currentComplexity;
float apparentPower,zeroXingDelay;

static float totalVariance;

void powerAnalysis(uint16_t gain, uint16_t delay, BufferDump *dump) {
    nClipped = 0;
    cosSum = sinSum = 0;
    moment0 = moment1 = 0;
    for(_u8val = 0; _u8val < NPOWERSAMPBY4; _u8val++) {
        if(_u8val == 0) {
            sink = 0;
            cosk = 1;
        }
        else {
            _fval = DPHI60*_u8val;
            sink = sin(_fval);
            cosk = cos(_fval);
            tick();
        }
        for(cycle = 0; cycle < 4; cycle++) {
            if((cycle%2) && (_u8val==0)) {
                cosk = -1;
                continue;
            }
            switch(cycle) {
                case 0:
                _u16val = WAVEDATA(_u8val);
                break;
                case 1:
                _u16val = WAVEDATA(NPOWERSAMPBY2-_u8val);
                cosk = -cosk;
                break;
                case 2:
                _u16val = WAVEDATA(NPOWERSAMPBY2+_u8val);
                sink = -sink;
                break;
                case 3:
                _u16val = WAVEDATA(NPOWERSAMP-_u8val);
                cosk = -cosk;
                break;
            }
            if(_u16val < 3 || _u16val > 1020) nClipped++;
            moment0 += _u16val;
            moment1 += (uint32_t)_u16val*_u16val;
            _fval = _u16val;
            cosSum += _fval*cosk;
            sinSum += _fval*sink;
        }
        tick();
    }
    // store the floating point 60 Hz RMS in ADC units
    _fval = sqrt(cosSum*cosSum+sinSum*sinSum)*RMS_NORM;

    // convert to milliWatts using the gain provided
    apparentPower = gain*_fval;
    tick();
    
    // calculate the fraction of the total variance that is not at 60 Hz
    totalVariance = ONE_OVER_NPOWERSAMP*moment1 -
        ONE_OVER_NPOWERSAMP_SQ*moment0*moment0;
    currentComplexity =
        (uint8_t)(255*(totalVariance - _fval*_fval)/totalVariance + 0.5);

    // initialize our optional dump header
    if(0 != dump) {
        /* zero out the dump header */
        for(_u8val = 0; _u8val < 15; _u8val++) dump->packed[_u8val] = 0;
        /* start filling our dump header */
        DUMP_ANALYSIS_SAVE(0,uint8_t,nClipped);
        DUMP_ANALYSIS_SAVE(1,uint8_t,currentComplexity);
        DUMP_ANALYSIS_SAVE(2,float,_fval);
    }
    tick();

    // Calculate the delay (in us) of the current sampling compared
    // with the earlier voltage-fiducial sampling.
    if(timestamp > tzero) {
        elapsed = timestamp - tzero;
    }
    else {
        // handle the (unlikely) case where the micros counter rolled over
        elapsed = 0xffffffff - tzero;
        elapsed += timestamp;
    }
    
    // Calculate the raw phase (in us) of an equivalent 60 Hz
    // sine wave. Offset is relative to WAVEDATA(0)=sample[6].
    _fval = atan2(sinSum,cosSum)*RAD_TO_MICROS_60 - POWER_CYCLE_MICROS_BY_4;
    if(_fval < 0) _fval += POWER_CYCLE_MICROS;    
    if(0 != dump) {
        DUMP_ANALYSIS_SAVE(6,uint16_t,(uint16_t)(_fval+0.5));
    }
    tick();

    // Calculate the delay (in us) of the current zero crossing relative
    // to the voltage zero crossing, modulus a 120 Hz cycle. The power
    // factor is related to this delay via:
    //
    //  PF = fabs(cos(zeroXingDelay*TWOPI*60*1e-6))
    //
    // The reason we do not return the power factor here is that the
    // delay is what we actually measure more directly so it is more
    // suitable for averaging (taking care of modulus issues).
    zeroXingDelay = fmod(_fval + elapsed - voltagePhase - delay,
        POWER_CYCLE_MICROS_BY_2);
    if(0 != dump) {
        DUMP_ANALYSIS_SAVE(8,uint16_t,(uint16_t)(zeroXingDelay + 0.5));
    }
}

// =====================================================================
// Aanalyzes the zero-crossing fiducial signal to determine the
// AC power factor.
// =====================================================================

uint8_t wrapOffset;
uint16_t voltagePhase;
uint32_t moment0,moment1,tzero;

void phaseAnalysis(BufferDump *dump) {
    // save the timestamp of the voltage phase analysis for later
    // comparison with the current analysis timestamps
    tzero = timestamp;
    moment0 = moment1 = 0;
    // use an 8 sample window to check if the fiducial pulse wraps around
    if((WAVEDATA(0)+WAVEDATA(1)+WAVEDATA(2)+WAVEDATA(3) > 400) &&
        (WAVEDATA(NPOWERSAMP-4)+WAVEDATA(NPOWERSAMP-3)+
        WAVEDATA(NPOWERSAMP-2)+WAVEDATA(NPOWERSAMP-1) > 400)) {
        wrapOffset = NPOWERSAMPBY2;
    }
    else {
        wrapOffset = 0;
    }
    // Calculate the center-of-gravity moments of the fiducial pulse.
    for(_u8val = 0; _u8val < NPOWERSAMP; _u8val++) {
        _u16val = WAVEDATA(_u8val);
        moment0 += _u16val;
        // the UL below is to force the RHS to be evaluated as uint32_t
        moment1 += (wrapOffset+(6UL*_u8val+wrapOffset)%NPOWERSAMP)*_u16val;
    }
    tick();
    
    // The denominator measures the integral of the fiducial signal and
    // provides a phase-independent check that we have a valid signal.
    // If this check fails then there is either something wrong with the
    // fiducial circuit or else there is no AC input voltage.
    if(moment0 < 15000 || moment0 > 150000) {
        voltagePhase = 0xffff;
    }
    else {
        // Convert the fiducial offset to microseconds with rounding.
        // Offset is relative to WAVEDATA(0)=sample[6].
        voltagePhase = (uint16_t)(
            (MICROS_PER_SAMPLE*moment1+3*moment0)
            /(6*moment0));
    }
    tick();
    
    if(0 != dump) {
        /* zero out the dump header */
        for(_u8val = 0; _u8val < 15; _u8val++) dump->packed[_u8val] = 0;
        // Save the COG moments and wrap offset
        DUMP_ANALYSIS_SAVE(0,uint32_t,moment1);
        DUMP_ANALYSIS_SAVE(4,uint32_t,moment0);
        DUMP_ANALYSIS_SAVE(8,uint16_t,voltagePhase);
        DUMP_ANALYSIS_SAVE(10,uint8_t,wrapOffset);
    }
}
