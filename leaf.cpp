#include "utilities.h"

// ---------------------------------------------------------------------
// Temperature monitoring parameters
// ---------------------------------------------------------------------
#define NTEMPSUM        2048
#define NTEMPSUMBY2     1024
// (0.1F/mV)(5000mV/1024ADC)/NTEMPSUM
#define ADC2DEGF 0.0002384185791015625
// 2pi/NTEMPSUM
#define DPHIGLOW 0.0030679615757712823

// Power to threshold conversion factor: (1<<27)/(1000W)^2
#define THRESHOLDSCALE 134.21772799999999


// =====================================================================
// Global variable declarations. All variables must fit within 2K
// of SRAM, including any variables allocated on the stack at runtime.
// =====================================================================

Packet packet;

// ---------------------------------------------------------------------
// Temperature monitoring globals
// ---------------------------------------------------------------------
byte whichLED;
unsigned int temperatureIndex;
unsigned long temperatureSum;

// =====================================================================
// The setup() function is called once on startup.
// =====================================================================

void setup() {
    // setup our digital outputs
    pinMode(AMBER_LED_PIN,OUTPUT);
    pinMode(GREEN_LED_PIN,OUTPUT);
    pinMode(RED_LED_PIN,OUTPUT);
    pinMode(BLUE_LED_PIN,OUTPUT);
    pinMode(PIEZO_PIN,OUTPUT);
    
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
    cricket();
    delay(250);

    // startup the serial port for talking to an optional debugging LCD
    LCDinit();
    
    // nordic wireless initialization
    initNordic(0);
    if(nordicOK) {        
        LCDprint("uci@home","wireless ok");
    }
    else {
        LCDprint("uci@home","no wireless");
    }
    
    // initialize wireless packet
    packet.deviceID = 0xbeef;
    packet.sequenceNumber = 0;
    for(byteValue = 0; byteValue < PACKET_VALUES; byteValue++) {
        packet.data[byteValue] = 0;
    }
}

void loop() {

    // LEDs off during light measurements
    digitalWrite(AMBER_LED_PIN,LOW);
    digitalWrite(GREEN_LED_PIN,LOW);

    // =====================================================================
    // Do a burst of 256 x 5kHz ADC samples lasting exactly 51,200 us
    // 250 samples span exactly 6 120Hz powerline cycles
    // =====================================================================
    bufptr = buffer;
    noInterrupts();
    do {
        // toggle pin13 to allow scope timing measurements
        digitalWrite(BLUE_LED_PIN, HIGH);
        *bufptr++ = TCNT0;
        *bufptr++ = analogRead(LIGHTING_PIN);
        digitalWrite(BLUE_LED_PIN, LOW);
        // insert some idle delay (borrowed from delayMicroseconds() in wiring.c)
        delayCycles = 328; // 4 CPU cycles = 0.25us per iteration
        __asm__ __volatile__ (
            "1: sbiw %0,1" "\n\t" // 2 cycles
            "brne 1b" : "=w" (delayCycles) : "0" (delayCycles) // 2 cycles
            );
    } while(++counter); // wraps around at 256
    interrupts();

    // Analyze the captured waveform and dump the results
    lightingAnalysis();
    
    packet.data[0] = lightingMean;
    packet.data[1] = lighting120Hz;

//    if((lighting120Hz > 20) && (10*lighting120Hz > lightingMean)) {
    if(packet.sequenceNumber % 2) {
        whichLED = AMBER_LED_PIN;
    }
    else {
        whichLED = GREEN_LED_PIN;
    }

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
        analogWrite(whichLED,(int)(127.*(1.-cos(temperatureIndex*DPHIGLOW))+0.5));
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
        digitalWrite(BLUE_LED_PIN, HIGH);
        *bufptr++ = TCNT0;
        *bufptr++ = analogRead(ACPOWER_PIN);
        digitalWrite(BLUE_LED_PIN, LOW);
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
        analogWrite(whichLED,(int)(127.*(1.-cos(temperatureIndex*DPHIGLOW))+0.5));
        tick();
    }
    packet.data[3] = (unsigned int)(100*temperatureSum*ADC2DEGF);
    
    packet.sequenceNumber++;
    if(nordicOK) {
        // Append the nordic transmit observer register contents to our packet
        Mirf.readRegister(OBSERVE_TX,(byte*)&(packet.status),1);
    
        // Transmit our new sensor readings
        Mirf.send((byte*)&packet);
        while(1) { // should probably add a timeout to this loop...
            byteValue = Mirf.getStatus();
            // did we reach the max retransmissions limit?
            if(byteValue & (1 << MAX_RT)) {
                // flash the red LED and play a low tone to signal the communications error
                digitalWrite(RED_LED_PIN,HIGH);
                //tone(2273,110);
                delay(250);
                digitalWrite(RED_LED_PIN,LOW);
                break;
            }
            // was the transmission acknowledged by the receiver?
            if(byteValue & (1 << TX_DS)) {
                //cricket();
                break;
            }
        }

        Mirf.powerUpRx(); // return to Rx mode
    }
}

int main(void) {
    init();
    setup();
    for (;;) {
        loop();
    }
    return 0;
}