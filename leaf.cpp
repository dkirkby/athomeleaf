#include "utilities.h"

#define PRINT_LIGHTING

// ---------------------------------------------------------------------
// Define this device's 16-bit ID. MSB must be clear so there are
// 2^15 = 32,768 choices.
// ---------------------------------------------------------------------
#define MY_ID 0x00ED

// ---------------------------------------------------------------------
// Temperature monitoring parameters
// ---------------------------------------------------------------------
#define NTEMPSUM        2048
#define NTEMPSUMBY2     1024
// (0.1F/mV)(5000mV/1024ADC)/NTEMPSUM
#define ADC2DEGF 0.0002384185791015625
// 2pi/NTEMPSUM
#define DPHIGLOW 0.0030679615757712823

// number of readings to wait until self-heating has stabilized
// (must be less than 256, set to zero to disable)
#define SELF_HEATING_DELAY 0
// the size of the self heating correction to apply after the delay (degF x 100)
#define SELF_HEATING_CORRECTION 0
// the blue/red LED will flash every Nth readings for below/above comfort range
#define TEMP_FLASH_INTERVAL 8
// blue/red flash duration (ms) to indicate below/above comfort range
#define TEMP_FLASH_DURATION 20

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
unsigned short temperatureMax = 8000; // degF x 100
unsigned short temperatureMin = 7400; // degF x 100
unsigned short selfHeatingCorrection = 0; // degF x 100

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
    cricket();
    delay(250);

    // startup the serial port for talking to an optional debugging LCD
    LCDinit();
    
    // nordic wireless initialization
    initNordic(MY_ID,0);
    if(nordicOK) {        
        LCDprint("uci@home","wireless ok");
    }
    else {
        LCDprint("uci@home","no wireless");
    }
    
    // initialize wireless packets
    packet.deviceID = MY_ID & 0x7fff; // make sure the MSB is clear
    packet.sequenceNumber = 0;
    for(byteValue = 0; byteValue < PACKET_VALUES; byteValue++) {
        packet.data[byteValue] = 0;
    }
    // Set the MSB in the dump packet to distinguish it from a normal
    // measurement packet. The status byte encodes what type of special
    // packet this is.
    dumpPacket.deviceID = packet.deviceID | 0x8000;
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
    
    if(lightingMean < 300) {
        whichLED = 0; // signals that the LED should be off
    }
    else if(lightingMean < 25000) {
        if(lighting120Hz > (lightingMean >> 3)) {
            whichLED = AMBER_LED_PIN;
        }
        else {
            whichLED = GREEN_LED_PIN;
        }
    }
    else {
        whichLED = 0xff; // signals that we defer to the low-gain analysis
    }    
    
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
        if(lighting120Hz > (lightingMean >> 3)) {
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
    
    packet.data[0] = lightingMean;
    packet.data[1] = lighting120Hz;

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
    packet.data[3] = (unsigned int)(100*temperatureSum*ADC2DEGF);
            
    //----------------------------------------------------------------------
    // Use the red/blue LEDs to indicate if the temperature is beyond the
    // comfort level. Don't flash every reading to minimize distraction.
    //----------------------------------------------------------------------
    if((cycleCount > 0 || packet.sequenceNumber > SELF_HEATING_DELAY) &&
        (packet.sequenceNumber % TEMP_FLASH_INTERVAL == 0)) {
        if(packet.data[3] > temperatureMax + SELF_HEATING_CORRECTION) {
            digitalWrite(RED_LED_PIN,HIGH);
        }
        else if(packet.data[3] < temperatureMin + SELF_HEATING_CORRECTION) {
            digitalWrite(BLUE_LED_PIN,HIGH);
        }
    }
    delay(TEMP_FLASH_DURATION);
    digitalWrite(RED_LED_PIN,LOW);
    digitalWrite(BLUE_LED_PIN,LOW);
    
    //----------------------------------------------------------------------
    // Transmit our data via the nordic interface
    //----------------------------------------------------------------------
    if(nordicOK) {
        // Append the nordic transmit observer register contents to our packet
        Mirf.readRegister(OBSERVE_TX,(byte*)&(packet.status),1);
    
        // Transmit our new sensor readings
        Mirf.send((byte*)&packet);
        while(1) { // should probably add a timeout to this loop...
            byteValue = Mirf.getStatus();
            // did we reach the max retransmissions limit?
            if(byteValue & (1 << MAX_RT)) {
                break;
            }
            // was the transmission acknowledged by the receiver?
            if(byteValue & (1 << TX_DS)) {
                break;
            }
        }

        Mirf.powerUpRx(); // return to Rx mode
    }
    
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
    Serial.print(packet.data[3]/100,DEC);
    Serial.print('.');
    uintValue = packet.data[3]%100;
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