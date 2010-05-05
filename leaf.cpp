#include "utilities.h"

//#define PRINT_LIGHTING

// ---------------------------------------------------------------------
// Connection state machine
// ---------------------------------------------------------------------
#define STATE_DISCONNECTED 0
#define STATE_CONNECTING   1
#define STATE_CONNECTED    2
byte connectionState = STATE_DISCONNECTED;

// ---------------------------------------------------------------------
// Declare our 'look-at-me' packet
// ---------------------------------------------------------------------
LookAtMe LAM = {
    0, // serial number will be copied from EEPROM
#ifdef COMMIT_INFO
    COMMIT_INFO
#elif
    0, { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }, 0
#endif
};

// ---------------------------------------------------------------------
// Temperature monitoring parameters
// ---------------------------------------------------------------------
#define NTEMPSUM        2048
#define NTEMPSUMBY2     1024
// (0.1F/mV)(5000mV/1024ADC)/NTEMPSUM
#define ADC2DEGF 0.0002384185791015625
// 2pi/NTEMPSUM
#define DPHIGLOW 0.0030679615757712823

// number of cyles (256 readings each) to wait until self-heating has stabilized
#define SELF_HEATING_DELAY 2
// the size of the self heating correction to apply after the delay (degF x 100)
#define SELF_HEATING_CORRECTION 763
// the blue/red LED will flash every Nth readings for below/above comfort range
#define TEMP_FLASH_INTERVAL 8
// blue/red flash duration (ms) to indicate below/above comfort range
#define TEMP_FLASH_DURATION 20

// ---------------------------------------------------------------------
// Lighting analysis parameters
// ---------------------------------------------------------------------
// If mean lighting level on the high-gain channel is below this threshold
// we consider the room to be dark
#define DARK_THRESHOLD 250
// If the high-gain mean lighting level is above this value, give preference
// to the low-gain channel. With a crossover at 10,000, the low-gain mean
// should be at least 500.
#define LIGHTING_CROSSOVER 10000
// Whenever the mean lighting level exceeds the DARK_THRESHOLD, artificial
// lighting is considered to be present if the ratio of the 120 Hz amplitude
// to mean lighting level is at least 1/ARTIFICIAL_THRESHOLD. Whether to use
// the high- or low-gain channel for this test depends on LIGHTING_CROSSOVER.
#define ARTIFICIAL_THRESHOLD 100

// ---------------------------------------------------------------------
// Power analysis parameters
// ---------------------------------------------------------------------

// Power to threshold conversion factor: (1<<27)/(1000W)^2
#define THRESHOLDSCALE 134.21772799999999

// =====================================================================
// Global variable declarations. All variables must fit within 2K
// of SRAM, including any variables allocated on the stack at runtime.
// =====================================================================

unsigned short cycleCount = 0; // counts the number of completed 256-packet sequences

// ---------------------------------------------------------------------
// Temperature monitoring globals
// ---------------------------------------------------------------------
byte whichLED;
unsigned int temperatureIndex;
unsigned long temperatureSum;
unsigned short temperatureMax = 7800; // degF x 100
unsigned short temperatureMin = 7400; // degF x 100
unsigned short selfHeatingCorrection = 0; // degF x 100

// =====================================================================
// The setup() function is called once on startup.
// =====================================================================

void setup() {
    // copy our serial number from EEPROM to our LAM packet
    copySerialNumber(&LAM);
    
    // setup our digital outputs
    pinMode(AMBER_LED_PIN,OUTPUT);
    pinMode(GREEN_LED_PIN,OUTPUT);
    pinMode(RED_LED_PIN,OUTPUT);
    pinMode(BLUE_LED_PIN,OUTPUT);
    pinMode(PIEZO_PIN,OUTPUT);
    pinMode(STROBE_PIN,OUTPUT);
    
    // configure the nordic interrupt line
    //pinMode(NORDIC_IRQ,INPUT);
    
    // run-through the outputs once to show we are alive (and also provide
    // an audio-visual self-test of each output)
    digitalWrite(BLUE_LED_PIN,HIGH);
    delay(500);
    digitalWrite(BLUE_LED_PIN,LOW);
    digitalWrite(RED_LED_PIN,HIGH);
    delay(500);
    digitalWrite(RED_LED_PIN,LOW);
    digitalWrite(AMBER_LED_PIN,HIGH);
    delay(500);
    digitalWrite(AMBER_LED_PIN,LOW);
    digitalWrite(GREEN_LED_PIN,HIGH);
    delay(500);
    digitalWrite(GREEN_LED_PIN,LOW);

    // startup the serial port for talking to an optional debugging LCD
    LCDinit();
    
    // nordic wireless initialization
    initNordic((unsigned short)LAM.serialNumber,0);
    
    // initialize wireless packets
    packet.deviceID = (unsigned short)(LAM.serialNumber & 0x7fff); // make sure the MSB is clear
    packet.sequenceNumber = 0;
    packet.status = 0;
    for(byteValue = 0; byteValue < DATA_PACKET_VALUES; byteValue++) {
        packet.data[byteValue] = 0;
    }
    // Set the MSB in the dump packet to distinguish it from a normal
    // measurement packet. The status byte encodes what type of special
    // packet this is.
    dumpPacket.deviceID = packet.deviceID | 0x8000;
    
    // send an initial Look-at-Me packet to test if there is a hub out there
    if(sendNordic(lamAddress, (byte*)&LAM, sizeof(LAM)) < 0x10) {
        LCDprint("uci@home","connected to hub");
    }
    else {
        if(!nordicOK) {
            LCDprint("uci@home","wireless hardware error");
        }
        else {
            LCDprint("uci@home","no hub found");
        }        
    }
}

void loop() {
    // update our packet counter (which cycles from $00-$ff)
    if(++packet.sequenceNumber == 0) cycleCount++;

    // LEDs off during light measurements
    digitalWrite(AMBER_LED_PIN,LOW);
    digitalWrite(GREEN_LED_PIN,LOW);

    // =====================================================================
    // Do a burst of 256 x 5kHz ADC samples lasting exactly 51,200 us
    // 250 samples span exactly 6 120Hz powerline cycles.
    // First time round is the high-gain photoamp output.
    // =====================================================================
    bufptr = buffer;
    noInterrupts();
    do {
        // toggle pin13 to allow scope timing measurements
        digitalWrite(STROBE_PIN, HIGH);
        *bufptr++ = TCNT0;
        *bufptr++ = analogRead(LIGHTING_PIN_HI);
        digitalWrite(STROBE_PIN, LOW);
        // insert some idle delay (borrowed from delayMicroseconds() in wiring.c)
        delayCycles = 328; // 4 CPU cycles = 0.25us per iteration
        __asm__ __volatile__ (
            "1: sbiw %0,1" "\n\t" // 2 cycles
            "brne 1b" : "=w" (delayCycles) : "0" (delayCycles) // 2 cycles
            );
    } while(++counter); // wraps around at 256
    interrupts();

    // Analyze the captured waveform
    lightingAnalysis(16.0);
    
    if(lightingMean < DARK_THRESHOLD) {
        whichLED = 0; // signals that the room is dark and the LED should be off
    }
    else if(lightingMean < LIGHTING_CROSSOVER) {
        if(lighting120Hz > lightingMean/ARTIFICIAL_THRESHOLD) {
            whichLED = AMBER_LED_PIN;
        }
        else {
            whichLED = GREEN_LED_PIN;
        }
    }
    else {
        whichLED = 0xff; // signals that we defer to the low-gain analysis
    }    
    
    packet.data[0] = lightingMean;
    packet.data[1] = lighting120Hz;
    
    #ifdef PRINT_LIGHTING
    LCDclear();
    Serial.print(lightingMean,DEC);
    Serial.write(' ');
    Serial.print(lighting120Hz,DEC);
    #endif
    
    // =====================================================================
    // Do a burst of 256 x 5kHz ADC samples lasting exactly 51,200 us
    // 250 samples span exactly 6 120Hz powerline cycles.
    // Second time round is the low-gain photoamp output.
    // =====================================================================
    bufptr = buffer;
    noInterrupts();
    do {
        // toggle pin13 to allow scope timing measurements
        digitalWrite(STROBE_PIN, HIGH);
        *bufptr++ = TCNT0;
        *bufptr++ = analogRead(LIGHTING_PIN);
        digitalWrite(STROBE_PIN, LOW);
        // insert some idle delay (borrowed from delayMicroseconds() in wiring.c)
        delayCycles = 328; // 4 CPU cycles = 0.25us per iteration
        __asm__ __volatile__ (
            "1: sbiw %0,1" "\n\t" // 2 cycles
            "brne 1b" : "=w" (delayCycles) : "0" (delayCycles) // 2 cycles
            );
    } while(++counter); // wraps around at 256
    interrupts();

    // Analyze the captured waveform
    lightingAnalysis(16.0);
    
    if(whichLED == 0xff) {
        if(lighting120Hz > lightingMean/ARTIFICIAL_THRESHOLD) {
            whichLED = AMBER_LED_PIN;
        }
        else {
            whichLED = GREEN_LED_PIN;
        }
    }
    
    #ifdef PRINT_LIGHTING
    LCDpos(1,0);
    Serial.print(lightingMean,DEC);
    Serial.write(' ');
    Serial.print(lighting120Hz,DEC);
    #endif
    
    // Dump every 16th lighting waveform
    //if((packet.sequenceNumber & 0x0f) == 0) dumpBuffer(PACKET_DUMP_LIGHTING);
    
    // =====================================================================
    // Calculate average of NTEMPSUM temperature ADC samples.
    // Result is stored in 32-bit unsigned, so can average up to 2^22
    // 10-bit ADC samples without overflow. The temperature sampling is
    // split into two sets, with the LED ramping on during the first set
    // and off during the second set. AC power sampling is performed
    // between the two sets. The green LED is used if there is no artificial
    // lighting detected, and otherwise the amber LED is used.
    // =====================================================================
    
    temperatureSum = 0;

    //----------------------------------------------------------------------
    // Temperature sampling set 1
    //----------------------------------------------------------------------
    // The first sample sometimes reads low so don't use it
    analogRead(TEMPERATURE_PIN);
    tick();
    for(temperatureIndex = 0; temperatureIndex <  NTEMPSUMBY2; temperatureIndex++) {
        temperatureSum += analogRead(TEMPERATURE_PIN);
        // gradually ramp the LED on during the first set of temperature samples
        byteValue = (byte)(127.*(1.-cos(temperatureIndex*DPHIGLOW))+0.5);
        if(whichLED) analogWrite(whichLED,byteValue);
        tick();
    }
    //----------------------------------------------------------------------
    // Do a burst of 256 x 5kHz ADC samples lasting exactly 51,200 us
    // 250 samples span exactly 3 60Hz powerline cycles
    //----------------------------------------------------------------------
    bufptr = buffer;
    noInterrupts();
    do {
        // toggle pin13 to allow scope timing measurements
        digitalWrite(STROBE_PIN, HIGH);
        *bufptr++ = TCNT0;
        *bufptr++ = analogRead(ACPOWER_PIN);
        digitalWrite(STROBE_PIN, LOW);
        // insert some idle delay (borrowed from delayMicroseconds() in wiring.c)
        delayCycles = 328; // 4 CPU cycles = 0.25us per iteration
        __asm__ __volatile__ (
            "1: sbiw %0,1" "\n\t" // 2 cycles
            "brne 1b" : "=w" (delayCycles) : "0" (delayCycles) // 2 cycles
            );
    } while(++counter); // wraps around at 256
    interrupts();
    
    // Analyze the captured waveform and dump the results
    powerAnalysis();
    
    packet.data[2] = (unsigned int)(10*rmsPower);
    
    // update the click threshold based on the new power estimate
    clickThreshold = (unsigned long)(THRESHOLDSCALE*rmsPower*rmsPower);
    
    //----------------------------------------------------------------------
    // Temperature sampling set 2
    //----------------------------------------------------------------------
    // The first sample sometimes reads low so don't use it
    analogRead(TEMPERATURE_PIN);
    tick();
    for(temperatureIndex = NTEMPSUMBY2; temperatureIndex < NTEMPSUM; temperatureIndex++) {
        temperatureSum += analogRead(TEMPERATURE_PIN);
        // gradually ramp the LED off during the second set of temperature samples
        byteValue = (byte)(127.*(1.-cos(temperatureIndex*DPHIGLOW))+0.5);
        if(whichLED) analogWrite(whichLED,byteValue);
        tick();
    }

    // calculate the average temperature in degF x 100
    packet.data[4] = (unsigned int)(100*temperatureSum*ADC2DEGF);
            
    //----------------------------------------------------------------------
    // Use the red/blue LEDs to indicate if the temperature is beyond the
    // comfort level. Don't flash every reading to minimize distraction.
    // Disable the temperature feedback when the room is dark (whichLED = 0)
    //----------------------------------------------------------------------
    if(whichLED && (cycleCount >= SELF_HEATING_DELAY) && (packet.sequenceNumber % TEMP_FLASH_INTERVAL == 0)) {
        if(packet.data[4] > temperatureMax + SELF_HEATING_CORRECTION) {
            digitalWrite(RED_LED_PIN,HIGH);
        }
        else if(packet.data[4] < temperatureMin + SELF_HEATING_CORRECTION) {
            digitalWrite(BLUE_LED_PIN,HIGH);
        }
    }
    delay(TEMP_FLASH_DURATION);
    digitalWrite(RED_LED_PIN,LOW);
    digitalWrite(BLUE_LED_PIN,LOW);
    
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!! Hijack the packet for lighting
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    packet.data[2] = lightingMean;
    packet.data[3] = lighting120Hz;

    //----------------------------------------------------------------------
    // Transmit our data via the nordic interface. Save the return value
    // to send with the next packet.
    //----------------------------------------------------------------------
    packet.status = sendNordic(dataAddress, (byte*)&packet, sizeof(packet));
    
    //----------------------------------------------------------------------
    // Display readings on the optional LCD
    //----------------------------------------------------------------------
#ifndef PRINT_LIGHTING
    LCDclear();
    Serial.print(packet.data[0],DEC);
    LCDpos(0,5);
    Serial.print('/');
    Serial.print(packet.data[1],DEC);
    // sequence number $00-$FF in hex
    LCDpos(0,14);
    if(packet.sequenceNumber < 0x10) {
        Serial.print('0'); // zero pad
    }
    Serial.print(packet.sequenceNumber,HEX);
    // power reading
    LCDpos(1,0);
    Serial.print(packet.data[2],DEC);
    // temperature reading
    LCDpos(1,7);
    Serial.print(packet.data[4]/100,DEC);
    Serial.print('.');
    uintValue = packet.data[4]%100;
    if(uintValue < 10) {
        Serial.print('0');
    }
    Serial.print(uintValue,DEC);
    // status byte in hex
    LCDpos(1,14);
    if(packet.status < 0x10) {
        Serial.print('0');
    }
    Serial.print(packet.status,HEX);
#endif
}

int main(void) {
    init();
    setup();
    for (;;) {
        loop();
    }
    return 0;
}