#include "utilities.h"

#define PRINT_LIGHTING

// ---------------------------------------------------------------------
// Connection state machine
// ---------------------------------------------------------------------
#define STATE_CONNECTING   1
#define STATE_CONNECTED    2
byte connectionState;

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
// Declare our run-time configuration data
// ---------------------------------------------------------------------
Config config;

// ---------------------------------------------------------------------
// Temperature monitoring parameters
// ---------------------------------------------------------------------
#define NTEMPSUM        2048
#define NTEMPSUMBY2     1024
// (0.1F/mV)(5000mV/1024ADC)/NTEMPSUM
#define ADC2DEGF 0.0002384185791015625
// 2pi/NTEMPSUM
#define DPHIGLOW_SLOW 0.0030679615757712823
// 4pi/NTEMPSUM
#define DPHIGLOW_FAST 0.0061359231515425647

// number of cyles (256 readings each) to wait until self-heating has stabilized
#define SELF_HEATING_DELAY 2
// the size of the self heating correction to apply after the delay (degF x 100)
#define SELF_HEATING_CORRECTION 0 // 763
// the blue/red LED will flash every Nth readings for below/above comfort range
#define TEMP_FLASH_INTERVAL 2
// blue/red flash duration (ms) to indicate below/above comfort range
#define TEMP_FLASH_DURATION 20
// the maximum number of degrees above/below to indicate with individual flashes
#define TEMP_MAX_FLASHES 3
// spacing between individual flashes (ms)
#define TEMP_FLASH_SPACING 200

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

// ---------------------------------------------------------------------
// Handles a newly received packet on PIPELINE_CONFIG
// ---------------------------------------------------------------------

void handleConfigUpdate() {
    
    // Because our config address incoporates our serial number, it may
    // have higher error rates (eg, if it is mostly zeros or ones).
    // Start with some sanity checks...
    if(config.header != CONFIG_HEADER ||
        (connectionState == STATE_CONNECTED && config.networkID != packet.deviceID)) {
        // This looks like a spurious config packet so ignore it.
        packet.status |= STATUS_GOT_INVALID_CONFIG;
        tone(750,100);
        tone(1000,75);
        tone(750,100);
        tone(1000,75);
        tone(750,100);
        tone(1000,75);
        return;
    }
    
    // Use the provided networkID to identify our data packets
    packet.deviceID = config.networkID;

    // Remember this config in EEPROM so we have it available in case there
    // is no hub the next time we power up.
    saveConfig(&config);

    // Have we been waiting for this config data?
    if(connectionState == STATE_CONNECTING) {
        packet.status |= STATUS_GOT_INITIAL_CONFIG;
        tone(1000,75);
        tone(750,100);
        // we are now officially connected
        connectionState = STATE_CONNECTED;
    }
    else {
        packet.status |= STATUS_GOT_UPDATED_CONFIG;
        // Play alternating tones to indicate that we have updated our config
        tone(1000,75);
        tone(750,100);
        tone(1000,75);
        tone(750,100);
    }
}

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
    initNordic(serialNumber());
    
    // Send an initial Look-at-Me packet to test if there is a hub out there.
    // At this point, the LAM serial number is zero, which indicates that we
    // don't expect the hub to respond with a Config packet. The reason for
    // this is that the first packet sent after a reset does not seem to be
    // reliably received by the hub (even though it is reliably acknowledged
    // by the leaf's nordic chip... would be good to understand this better)
    if(sendNordic(lamAddress, (byte*)&LAM, sizeof(LAM)) < 0x10) {
        LCDprint("uci@home","connecting...");
        tone(1500,50);
        tone(1000,75);
    }
    else {
        if(!nordicOK) {
            LCDprint("uci@home","nordic error");
            // play alternating tones to indicate a wireless hardware problem
            tone(1000,75);
            tone(1500,50);
            tone(1000,75);
            tone(1500,50);
        }
        else {
            LCDprint("uci@home","no hub found");
            // play a falling sequence of tones to indicate that no hub was found
            tone(750,100);
            tone(1000,75);
            tone(1500,50);
        }        
    }

    // copy our serial number from EEPROM to our LAM packet
    LAM.serialNumber = serialNumber();
    
    // copy our config data from EEPROM
    loadConfig(&config);

    // Print our startup config to the optional LCD display
    delay(2000);
    LCDclear();
    Serial.print(LAM.serialNumber,HEX);
    LCDpos(0,11);
    Serial.print(config.temperatureMax,DEC);
    LCDpos(1,0);
    Serial.print(config.capabilities,BIN);
    LCDpos(1,11);
    Serial.print(config.temperatureMin,DEC);

    // Send another LAM with our real serial number after a short delay.
    delay(2000);
    sendNordic(lamAddress, (byte*)&LAM, sizeof(LAM));

    // Initialize our data packet. The device ID will be filled in later.
    // Even if there is no hub listening, we still use the packet's sequence
    // number counter for the passive feedback algorithms.
    packet.sequenceNumber = 0;
    packet.status = 0;
    for(byteValue = 0; byteValue < DATA_PACKET_VALUES; byteValue++) {
        packet.data[byteValue] = 0;
    }
    
    // We start out in the connecting state, waiting for a config packet
    // in response to our LAM packet.
    connectionState = STATE_CONNECTING;
}

// =====================================================================
// The loop() function is called repeatedly forever after setup().
// =====================================================================

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
        // gradually ramp the LED during the first set of temperature samples
        if(whichLED == AMBER_LED_PIN) {
            byteValue = (byte)(127.*(1.-cos(temperatureIndex*DPHIGLOW_FAST))+0.5);
        }
        else {
            byteValue = (byte)(127.*(1.-cos(temperatureIndex*DPHIGLOW_SLOW))+0.5);
        }
        // Supress visual lighting feedback if this capability has been disabled
        if((config.capabilities & CAPABILITY_LIGHT_FEEDBACK) && whichLED) {
            analogWrite(whichLED,byteValue);
        }
        tick();
    }
    
    //----------------------------------------------------------------------
    // Check for any incoming wireless data
    //----------------------------------------------------------------------
    if(getNordic((byte*)&config,sizeof(config)) == PIPELINE_CONFIG) {
        handleConfigUpdate();
    }
    else {
        // We clobbered our config in RAM above, so restore it from EEPROM now.
        // (This should never happen since we only have one pipeline active,
        // but just in case...)
        loadConfig(&config);
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
        // gradually ramp the LED during the second set of temperature samples
        if(whichLED == AMBER_LED_PIN) {
            byteValue = (byte)(127.*(1.-cos(temperatureIndex*DPHIGLOW_FAST))+0.5);
        }
        else {
            byteValue = (byte)(127.*(1.-cos(temperatureIndex*DPHIGLOW_SLOW))+0.5);
        }
        // Supress visual lighting feedback if this capability has been disabled
        if((config.capabilities & CAPABILITY_LIGHT_FEEDBACK) && whichLED) {
            analogWrite(whichLED,byteValue);
        }
        tick();
    }

    // calculate the average temperature in degF x 100
    packet.data[4] = (unsigned int)(100*temperatureSum*ADC2DEGF);
            
    //----------------------------------------------------------------------
    // Use the red/blue LEDs to indicate if the temperature is beyond the
    // comfort level. Don't flash every reading to minimize distraction.
    // Disable the temperature feedback when the room is dark (whichLED = 0)
    //----------------------------------------------------------------------
    uintValue = packet.data[4] - SELF_HEATING_CORRECTION;
    if(whichLED) {
        whichLED = 0;
        if((cycleCount >= SELF_HEATING_DELAY) &&
            (packet.sequenceNumber % TEMP_FLASH_INTERVAL == 0)) {
            if(uintValue > config.temperatureMax) {
                // how many degrees over are we? (round up so the answer is at least one)
                uintValue = 1 + (uintValue - config.temperatureMax)/100;
                whichLED = RED_LED_PIN;
            }
            else if(uintValue < config.temperatureMin) {
                // how many degrees under are we? (round up so the answer is at least one)
                uintValue = 1 + (config.temperatureMin - uintValue)/100;
                whichLED = BLUE_LED_PIN;
            }
            // The degree excess determines how many times we will flash. Max this out
            // at a small value.
            if(whichLED && uintValue > TEMP_MAX_FLASHES) uintValue = TEMP_MAX_FLASHES;
            // Supress visual temperature feedback if this capability has been disabled
            if(!(config.capabilities & CAPABILITY_TEMP_FEEDBACK)) whichLED = 0;
        }
    }
    // We always cycle through the max flash sequence so that the overall timing
    // is independent of how the LEDs are actually driven.
    for(byteValue = 0; byteValue < TEMP_MAX_FLASHES; byteValue++) {
        if(byteValue) delay(TEMP_FLASH_SPACING);
        if(whichLED && byteValue < uintValue) digitalWrite(whichLED,HIGH);
        delay(TEMP_FLASH_DURATION);        
        if(whichLED && byteValue < uintValue) digitalWrite(whichLED,LOW);
    }
    
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!! Hijack the packet for lighting
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    packet.data[2] = lightingMean;
    packet.data[3] = lighting120Hz;

    if(connectionState == STATE_CONNECTING) {
        // We are still waiting for a response from the hub. Send out another
        // request here...
        sendNordic(lamAddress, (byte*)&LAM, sizeof(LAM));
    }
    else {
        // Transmit our data via the nordic interface. Save the return value
        // to send with the next packet.
        packet.status =
            sendNordic(dataAddress, (byte*)&packet, sizeof(packet)) &
            STATUS_NUM_RETRANSMIT_MASK;
    }
    
    //----------------------------------------------------------------------
    // Display readings on the optional LCD
    //----------------------------------------------------------------------
#ifdef PRINT_SUMMARY
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