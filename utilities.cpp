#include "utilities.h"

// ---------------------------------------------------------------------
// Shared globals
// ---------------------------------------------------------------------
byte byteValue;
unsigned int uintValue;
float floatValue;
Packet packet;

// =====================================================================
// Print a floating point value as a fixed-precision decimal.
// Uses Serial.write() and Serial.print(). Does not emit a newline.
// =====================================================================

float floatToPrint;
unsigned long printMultiplier;

void _printFloat() {

    static unsigned long printULong;

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
void LCDinit(byte backlightLevel) {
    Serial.begin(9600);
    // turn cursor off
    Serial.write(0xfe);
    Serial.write(0x0c);
    delay(100); // needed to allow LCD serial decoder to keep up?
    // turn backlight off
    Serial.write(0x7c);
    Serial.write((byte)(0x80 | (backlightLevel % 30)));
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
void LCDpos(byte row, byte col) {
    Serial.write(0xfe);
    if(row == 0) {
        Serial.write((byte)(0x80 | (col & 0x0f)));
    }
    else {
        Serial.write((byte)(0xc0 | (col & 0x0f)));
    }
    //delay(100);
}

// =====================================================================
// Delays for about 1ms and generates an audible and visible "Geiger"
// click at pseudo-random intervals with an average rate controlled
// by the value of the clickThreshold global.
// =====================================================================

unsigned long clickThreshold = 0;

void tick() {

    static byte doClick;
    static unsigned long randomState = 24071966UL;

    // generate a pseudo-random unsigned long
    randomState = randomState*2891336453UL + 1640531513UL;
    randomState ^= randomState >> 13;
    randomState ^= randomState << 17;
    randomState ^= randomState >> 5;
    doClick = (randomState < clickThreshold) ? 1 : 0;
    digitalWrite(RED_LED_PIN,doClick);
    delayMicroseconds(250);
    digitalWrite(PIEZO_PIN,doClick);
    delayMicroseconds(500);
    digitalWrite(PIEZO_PIN,LOW);
    delayMicroseconds(250);
    digitalWrite(RED_LED_PIN,LOW);
}

// ---------------------------------------------------------------------
// Globals used for 60/120 Hz waveform captures (lighting + AC power)
// ---------------------------------------------------------------------
#define BUFFER_SIZE 512
unsigned int buffer[BUFFER_SIZE],*bufptr,delayCycles;
byte counter = 0;

#define WAVEDATA(K) buffer[(((K)+6)<<1)|1]

// ---------------------------------------------------------------------
// Dump the buffer contents via the wireless interface.
// Assumes that a global packet object already exists and is
// initialized with our device ID.
// ---------------------------------------------------------------------
void dumpBuffer(byte dumpType) {
    // set the high bit in our ID to signal that this is not a normal
    // measurement packet
    packet.deviceID |= 0x8000;
    // record the type of dump in the status byte
    packet.status = dumpType;
    // loop over buffer samples
    packet.sequenceNumber = 0;
    uintValue = BUFFER_SIZE/2;
    counter = 0;
    bufptr = buffer;
    while(--uintValue) { // loop over pairs of buffer bytes to write
        packet.data[counter++] = (*bufptr++) << 8 | (*bufptr++);
        if(counter == PACKET_VALUES || uintValue == 0) {
            // send the current packet contents
            // ...
            // get ready for the next packet
            packet.sequenceNumber++;
            counter = 0;
        }
    }
    // clear the high ID bit to return to normal packets
    packet.deviceID &= 0x7fff;
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
// The overall power conversion factor in Watts(rms)/ADC
// calculated as 120V(rms)*sqrt(2)/NPOWERSAMP*(5000mV)/(1024ADC)/(90mV/A(pk))
#define POWERSCALE 0.036828478186799352

// ---------------------------------------------------------------------
// Lighting and power analysis shared globals
// ---------------------------------------------------------------------
static byte cycle,nzero;
static float sink,cosk,cosSum,sinSum;

// =====================================================================
// Analyzes a lighting waveform to determine its mean and 120Hz
// amplitude, using a linear least-squares technique that is robust
// against clipping and can reconstruct waveforms whose baseline exceeds
// the ADC input range as long as some 120Hz is visible within the range.
// This function makes the following math calls: 62*sin, 62*cos, 3*sqrt.
// =====================================================================

unsigned int lightingMean,lighting120Hz;

void lightingAnalysis() {
    
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
    
    // Check for an almost singular matrix which signals an over/under-flow condition
    // in the sampling. At this point, alpha00 contains the number of unclipped samples
    // out of a possible NLIGHTSAMP maximum.
    if(alpha00 < MINUNCLIPPED) {
        beta1 = 0;
        if(nzero > NLIGHTSAMPBY2) {
            // underflow
            beta0 = 0;
        }
        else {
            // overflow
            beta0 = -1;
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
    
        // beta0 is the mean lighting level in 0.1 ADC units, clipped at zero.
        lightingMean = (beta0 < 0) ? 0 : (unsigned int)(0.1*beta0);
        // Calculate the 120Hz peak amplitude in 0.1 ADC units
        lighting120Hz = (unsigned int)(0.1*sqrt(beta1*beta1 + beta2*beta2));
    }
}

// =====================================================================
// Analyzes an AC current waveform to determine its true RMS power
// relative to a 120V RMS 60Hz voltage. The algorithm assumes that
// there is no clipping and that the load is purely resistive.
// This function makes the following math calls: 62*sin, 62*cos, 1*sqrt.
// =====================================================================

float rmsPower;

void powerAnalysis() {
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
    // store the RMS power (Watts) in cosSum
    rmsPower = POWERSCALE*sqrt(cosSum*cosSum+sinSum*sinSum);
}

// =====================================================================
// Initializes the Nordic nRF24L01+ transciever. Sets the value of
// the global nordicOK byte to either 1 (true) or 0 (false) to indicate
// if the initialization was successful.
// =====================================================================

byte nordicOK;

void initNordic(byte isHub) {
    
    // nordic wireless initialization
    Mirf.csnPin = SPI_SSEL;
    Mirf.cePin = NORDIC_CE;
    Mirf.init();

    if(isHub) {
        Mirf.setRADDR(HUB_ADDRESS);
    }
    else {
        Mirf.setTADDR(HUB_ADDRESS);
    }
    Mirf.payload = sizeof(Packet);
    Mirf.channel = RADIO_CHANNEL;
    Mirf.config();
    
    // read back the payload size to check that we really are talking
    // to a Nordic transceiver
    uint8_t rv;
	Mirf.readRegister(RX_PW_P1,&rv,1);
	if(rv == sizeof(Packet)) {
        nordicOK = 1;
	}
	else {
        nordicOK = 0;
	}
}

// =====================================================================
// Generates a frequency chirp
// =====================================================================

void chirp(byte cycles, byte timebase) {

    byte counter = cycles;
    unsigned int delay = cycles*timebase;
    
    digitalWrite(RED_LED_PIN,HIGH);
    do {
        digitalWrite(PIEZO_PIN,HIGH);
        delayMicroseconds(delay);
        digitalWrite(PIEZO_PIN,LOW);
        delayMicroseconds(delay);
        delay -= timebase;
    } while(--counter);

    counter = cycles;
    digitalWrite(RED_LED_PIN,LOW);
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

void tone(unsigned int halfPeriod,unsigned int cycles) {
    unsigned int counter = cycles;
    while(counter--) {
        digitalWrite(PIEZO_PIN,HIGH);
        delayMicroseconds(halfPeriod);
        digitalWrite(PIEZO_PIN,LOW);
        delayMicroseconds(halfPeriod);
    }
}

// =====================================================================
// Shifts a binary waveform out of the specified pin at a bitrate of
// 6.0625 us/bit = 97 clocks/bit or approximately 165 kbaud. Each
// byte of the input data is shifted from LSB to MSB.
// =====================================================================

void shiftOut(byte pin,unsigned int ndata,const byte *data) {
    
    pinMode(pin,OUTPUT);
    unsigned int index;
    for(index = 0; index < 8*ndata; index++) {
        digitalWrite(pin,data[index >> 3] & (1 << (index%8)));
    }
}

// =====================================================================
// Generates a cricket-like sound
// =====================================================================

#define CRICKET_SAMPLES 413

byte cricketSample[CRICKET_SAMPLES] = {
    86, 85, 85, 149, 170, 170, 90, 85, 85, 170, 170, 90, 85, 85, 169, \
    170, 106, 85, 85, 169, 210, 170, 213, 74, 165, 82, 173, 213, 170, \
    148, 82, 181, 214, 170, 146, 82, 213, 218, 170, 74, 82, 89, 219, 90, \
    41, 73, 106, 109, 91, 37, 73, 170, 182, 91, 149, 36, 169, 186, 109, \
    85, 34, 169, 218, 110, 77, 137, 40, 109, 183, 173, 68, 36, 181, 189, \
    173, 34, 34, 213, 238, 174, 146, 16, 85, 123, 183, 74, 132, 100, 221, \
    187, 37, 65, 164, 238, 222, 149, 8, 162, 186, 239, 86, 130, 144, 218, \
    125, 87, 17, 136, 106, 223, 91, 133, 64, 170, 251, 221, 36, 16, 210, \
    190, 223, 18, 1, 81, 247, 119, 75, 16, 72, 189, 191, 43, 1, 66, 237, \
    247, 173, 16, 32, 181, 255, 174, 4, 4, 213, 251, 183, 18, 128, 84, \
    255, 189, 9, 1, 100, 251, 223, 37, 0, 145, 125, 255, 22, 1, 144, 246, \
    127, 151, 64, 64, 186, 255, 91, 2, 32, 218, 127, 95, 17, 0, 105, 255, \
    111, 9, 64, 168, 247, 191, 37, 0, 164, 253, 223, 21, 2, 160, 246, \
    255, 150, 0, 136, 250, 254, 87, 4, 64, 218, 255, 91, 1, 8, 234, 254, \
    111, 9, 0, 169, 255, 123, 69, 0, 164, 253, 191, 37, 0, 162, 222, 255, \
    150, 0, 144, 246, 127, 87, 16, 64, 186, 255, 91, 2, 32, 218, 223, \
    111, 17, 0, 105, 255, 119, 9, 32, 100, 247, 191, 69, 0, 146, 253, \
    247, 38, 4, 80, 237, 127, 43, 1, 66, 245, 253, 171, 8, 32, 181, 127, \
    175, 4, 129, 212, 251, 183, 34, 32, 212, 126, 223, 18, 2, 81, 247, \
    119, 75, 32, 72, 123, 191, 77, 4, 162, 218, 239, 85, 66, 144, 234, \
    126, 87, 17, 68, 106, 239, 91, 137, 32, 106, 251, 110, 69, 136, 168, \
    189, 183, 37, 66, 148, 237, 221, 42, 17, 82, 117, 111, 171, 136, 72, \
    181, 187, 171, 36, 36, 181, 221, 173, 36, 145, 180, 238, 214, 148, \
    72, 170, 182, 91, 149, 36, 170, 182, 109, 85, 34, 169, 218, 182, 85, \
    146, 148, 218, 182, 86, 74, 82, 86, 219, 170, 74, 74, 85, 107, 171, \
    74, 41, 85, 107, 173, 74, 165, 84, 171, 213, 74, 165, 74, 171, 86, \
    85, 149, 170, 170, 86, 85, 85, 170, 170, 90, 85, 85, 170, 170, 106, \
    85, 85, 85, 1
};

void cricket(void) {
    digitalWrite(RED_LED_PIN,HIGH);
    shiftOut(PIEZO_PIN,CRICKET_SAMPLES,cricketSample);
    delay(20);
    shiftOut(PIEZO_PIN,CRICKET_SAMPLES,cricketSample);
    delay(20);
    shiftOut(PIEZO_PIN,CRICKET_SAMPLES,cricketSample);
    digitalWrite(RED_LED_PIN,LOW);
}
