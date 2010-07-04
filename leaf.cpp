// =====================================================================
// The leaf program runs on an embedded microcontroller collecting
// sensor data, giving immediate audio-visual feedback and sending
// sensor data wirelessly to a hub.
//
// Copyright (c) 2010 David Kirkby dkirkby@uci.edu
// =====================================================================

#include "serialno.h"
#include "wireless.h"
#include "packet.h"
#include "pins.h"
#include "utilities.h"
#include "audio.h"
#include "lcd.h"

#include "WProgram.h" // arduino header

// ---------------------------------------------------------------------
// Temperature monitoring parameters
// ---------------------------------------------------------------------
#define NTEMPSUM        2048
#define NTEMPSUMBY2     1024
// (0.1F/mV)(5000mV/1024ADC)/NTEMPSUM
#define ADC2DEGF 0.0002384185791015625

// ---------------------------------------------------------------------
// LED feedback parameters
// ---------------------------------------------------------------------
// slow-glow radians per temperature sample = 2pi/NTEMPSUM
#define DPHIGLOW_SLOW 0.0030679615757712823
// fast-glow radians per temperature sample = 4pi/NTEMPSUM
#define DPHIGLOW_FAST 0.0061359231515425647
// LED flash duration (ms)
#define FLASH_DURATION 20

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

#define HI_GAIN_NO_LOAD_THRESHOLD  100.0 // mW, corresponds to ~0.45 ADC
#define LO_GAIN_NO_LOAD_THRESHOLD 2000.0 // mW, corresponds to ~0.45 ADC

#define DELAY_WRAP_AROUND 4166.666666666667 // microseconds, 10^6/(4*60)
#define POWER_FACTOR_OMEGA 376.99111843077515e-6 // 2pi*60/10^6

// =====================================================================
// Global variable declarations. All variables must fit within 2K
// of SRAM, including any variables allocated on the stack at runtime.
// =====================================================================

// ---------------------------------------------------------------------
// Locally shared globals
// ---------------------------------------------------------------------
static uint8_t _u8val;
static uint16_t _u16val;
static float _fval;

// ---------------------------------------------------------------------
// Connection state machine
// ---------------------------------------------------------------------
#define STATE_CONNECTING      0x80
#define STATE_CONNECTED       0x40
#define DROPPED_PACKETS_MASK  0x3F
#define MAX_DROPPED_PACKETS   0x14 // corresponds to about one minute
uint8_t connectionState;

// ---------------------------------------------------------------------
// Declare and initialize our 'look-at-me' packet
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
// Declare our other packet buffers
// ---------------------------------------------------------------------
Config config;
DataPacket packet;
BufferDump dump;

// ---------------------------------------------------------------------
// Sequence numbering is maintained via the 8-bit packet.sequenceNumber
// and cycleCount which counts the number of completed 256-sequence
// cycles. At 4s per sequence, each cycle lasts about 17min and the
// cycle counter rolls over after about 2 years.
// ---------------------------------------------------------------------
unsigned short cycleCount = 0;

// ---------------------------------------------------------------------
// Control register for LED feedback
// ---------------------------------------------------------------------
uint8_t ledControl;

#define GREEN_GLOW  0
#define AMBER_GLOW  1
#define RED_GLOW    2
#define GREEN_FLASH 3
#define AMBER_FLASH 4
#define RED_FLASH   5
#define BLUE_FLASH  6
#define RAMP_UP     7

#define LED_INIT { ledControl = 0x7f & (uint8_t)(config.capabilities >> 7); }

#define LED_ALL_OFF { ledControl &= (1<<RAMP_UP); }

#define LED_ENABLE(FEATURE) { ledControl |= (1<<(FEATURE)); }
#define LED_DISABLE(FEATURE) { ledControl &= ~(1<<(FEATURE)); }
#define LED_IS_ENABLED(FEATURE) (ledControl & (1<<(FEATURE)))

#define LED_RAMP_TOGGLE { ledControl ^= (1<<RAMP_UP); }
#define LED_IS_RAMPING_UP (ledControl & (1<<RAMP_UP))

// Lighting globals
uint8_t roomIsDark;

// Temperature globals
uint8_t whichLED;
uint32_t temperatureSum;

// ---------------------------------------------------------------------
// Handles a newly received packet on PIPELINE_CONFIG
// ---------------------------------------------------------------------

void handleConfigUpdate() {
    
    // Because our config address incoporates our serial number, it may
    // have higher error rates (eg, if it is mostly zeros or ones).
    // Start with some sanity checks...
    if(config.header != CONFIG_HEADER ||
        ((connectionState & STATE_CONNECTED) && config.networkID != packet.networkID)) {
        // This looks like a spurious config packet so ignore it.
        packet.status |= STATUS_GOT_INVALID_CONFIG;
        if(config.capabilities & CAPABILITY_AUDIO_DIAGNOSTICS) {
            // a repeated (3X) falling tone sequence
            tone(750,100);
            tone(1000,75);
            tone(750,100);
            tone(1000,75);
            tone(750,100);
            tone(1000,75);
        }
        return;
    }
    
    // Use the provided networkID to identify our data packets
    packet.networkID = config.networkID;
    dump.networkID = config.networkID;

    // Remember this config in EEPROM so we have it available in case there
    // is no hub the next time we power up.
    saveConfig(&config);

    // Have we been waiting for this config data?
    if(connectionState & STATE_CONNECTING) {
        packet.status |= STATUS_GOT_INITIAL_CONFIG;
        if(config.capabilities & CAPABILITY_AUDIO_DIAGNOSTICS) {
            // a single rising tone sequence
            tone(1000,75);
            tone(750,100);
        }
        // we are now officially connected
        connectionState = STATE_CONNECTED;
    }
    else {
        packet.status |= STATUS_GOT_UPDATED_CONFIG;
        if(config.capabilities & CAPABILITY_AUDIO_DIAGNOSTICS) {
            // a repeated (2X) rising tone sequence
            tone(1000,75);
            tone(750,100);
            tone(1000,75);
            tone(750,100);
        }
    }
}

// ---------------------------------------------------------------------
// Prints the current configuration to the optional LCD screen.
// There are two screens of hex values that are each displayed
// for 30 seconds.
// ---------------------------------------------------------------------
void printConfig() {
    // page 1, line 1
    LCDclear();
    pprint(LAM.serialNumber);
    LCDpos(0,8);
    pprint(config.networkID); // will not be valid until after handshake with hub
    LCDpos(0,10);
    pprint(config.capabilities);
    LCDpos(0,14);
    pprint(config.dumpInterval);
    // page 1, line 2
    LCDpos(1,0);
    pprint(config.powerGainLo);
    LCDpos(1,4);
    pprint(config.powerGainHi);
    LCDpos(1,8);
    pprint(config.comfortTempMin);
    LCDpos(1,10);
    pprint(config.comfortTempMax);
    LCDpos(1,12);
    pprint(config.selfHeatOffset);
    // wait 30s before displaying the next page
    delay(30000);
    // page 2, line 1
    LCDclear();
    pprint(config.fiducialShiftHi);
    LCDpos(0,4);
    pprint(config.fiducialHiLoDelta);
    LCDpos(0,6);
    pprint(config.nClipCut);
    LCDpos(0,8);
    pprint(config.selfHeatDelay);
    // keep the last page displayed for 30s
    delay(30000);
}

// ---------------------------------------------------------------------
// Prints the current sample to the optional LCD screen. There is
// no built-in display delay so this routine assumes that the LCD will
// not be updated again immediately.
// ---------------------------------------------------------------------
void printSample() {
    LCDclear();
    Serial.print(packet.lightLevelHiGain,HEX);
    LCDpos(0,4);
    Serial.print(packet.light120HzHiGain,HEX);
    LCDpos(0,8);
    Serial.print(packet.lightLevelLoGain,HEX);
    LCDpos(0,12);
    Serial.print(packet.light120HzLoGain,HEX);
    LCDpos(1,0);
    Serial.print(packet.powerHiGain,HEX);
    LCDpos(1,4);
    Serial.print(packet.powerLoGain,HEX);
    LCDpos(1,8);
    Serial.print(packet.acPhase,HEX);
    LCDpos(1,10);
    Serial.print((packet.temperature/100)%100,DEC);
    LCDpos(1,12);
    Serial.print(packet.sequenceNumber,HEX);
    LCDpos(1,14);
    Serial.print(packet.status,HEX);    
}

// =====================================================================
// Perform an acquisition and analysis sequence for lighting conditions.
// If dump is non-zero, a buffer dump might be performed based on the
// current configuration.
//
// Results are saved in:
//  -roomIsDark: 0/1
//  -packet: lightLevelHiGain, lightLevelLoGain, light120HzHiGain, light120HzLoGain
//  -ledControl: enables GREEN/AMBER_GLOW if lightingFeedback capability is set
// =====================================================================
void lightingSequence(BufferDump *dump) {
    
    // Ensure that all LEDs are off during light measurements
    // although this is normally already taken care of.
    digitalWrite(AMBER_LED_PIN,LOW);
    digitalWrite(GREEN_LED_PIN,LOW);
    digitalWrite(RED_LED_PIN,LOW);
    digitalWrite(BLUE_LED_PIN,LOW);

    // ---------------------------------------------------------------------
    // First, sample the high-gain photodiode amplifier signal.
    // ---------------------------------------------------------------------
    acquireADCSamples(LIGHTING_PIN_HI);

    // Analyze the captured waveform
    lightingAnalysis(16.0,dump);
    
    _u8val = 0;
    roomIsDark = 0;
    if(lightingMean < DARK_THRESHOLD) {
        // the room is dark 
        roomIsDark = 1;
    }
    else if(lightingMean < LIGHTING_CROSSOVER) {
        if(lighting120Hz > lightingMean/ARTIFICIAL_THRESHOLD) {
            // artificial light is present
            if(config.capabilities & CAPABILITY_LIGHT_FEEDBACK) LED_ENABLE(AMBER_GLOW);
        }
        else {
            // no artificial light detected
            if(config.capabilities & CAPABILITY_LIGHT_FEEDBACK) LED_ENABLE(GREEN_GLOW);
        }
    }
    else {
        _u8val = 1; // signals that we defer to the low-gain analysis
    }    
    
    packet.lightLevelHiGain = lightingMean;
    packet.light120HzHiGain = lighting120Hz;
    
    // Periodically dump sample buffer if requested
    if(dump && (config.capabilities & CAPABILITY_LIGHT_DUMP) &&
        connectionState == STATE_CONNECTED &&
        (packet.sequenceNumber % config.dumpInterval) == (config.dumpInterval>>1)) {
        dumpBuffer(DUMP_BUFFER_LIGHT_HI,dump);
    }
    
    // ---------------------------------------------------------------------
    // Second, sample the low-gain photodiode amplifier signal.
    // ---------------------------------------------------------------------
    acquireADCSamples(LIGHTING_PIN);

    // Analyze the captured waveform
    lightingAnalysis(16.0,dump);
    
    if(_u8val) {
        if(lighting120Hz > lightingMean/ARTIFICIAL_THRESHOLD) {
            // artificial light is present
            if(config.capabilities & CAPABILITY_LIGHT_FEEDBACK) LED_ENABLE(AMBER_GLOW);
        }
        else {
            // no artificial light detected
            if(config.capabilities & CAPABILITY_LIGHT_FEEDBACK) LED_ENABLE(GREEN_GLOW);
        }
    }
    
    packet.lightLevelLoGain = lightingMean;
    packet.light120HzLoGain = lighting120Hz;
    
    // Periodically dump sample buffer if requested
    if(dump && (config.capabilities & CAPABILITY_LIGHT_DUMP) &&
        connectionState == STATE_CONNECTED &&
        (packet.sequenceNumber % config.dumpInterval) == (config.dumpInterval>>1)) {
        dumpBuffer(DUMP_BUFFER_LIGHT_LO,dump);
    }    
}

// =====================================================================
// Perform an acquisition and analysis sequence for power consumption.
// If dump is non-zero, a buffer dump might be performed based on the
// current configuration.
//
// Results are saved in:
//  -packet: powerLoGain, powerHiGain, (acPhase)
//  -(clickThreshold)
// =====================================================================
void powerSequence(BufferDump *dump) {
    
    // prepare to combine the high- and low-gain analysis results
    float apparentPowerSave = 0;
    float zeroXingDelaySave = 0;
    uint8_t nClipHi,complexitySave = 0;

    //----------------------------------------------------------------------
    // Start the power analysis by measuring the AC voltage phase
    //----------------------------------------------------------------------
    acquireADCSamples(ACPHASE_PIN);
    
    // Analyze the captured waveform
    phaseAnalysis(dump);
    
    // Periodically dump sample buffer if requested
    if(dump && (config.capabilities & CAPABILITY_POWER_DUMP) &&
        connectionState == STATE_CONNECTED &&
        (packet.sequenceNumber % config.dumpInterval) == 0) {
        dumpBuffer(DUMP_BUFFER_AC_PHASE,dump);
    }

    //----------------------------------------------------------------------
    // First time round uses the high-gain power channel.
    //----------------------------------------------------------------------    
    acquireADCSamples(ACPOWER_PIN_HI);
    
    // Analyze the captured high-gain waveform
    powerAnalysis(config.powerGainHi,config.fiducialShiftHi,dump);
    
    // Periodically dump sample buffer if requested
    if(dump && (config.capabilities & CAPABILITY_POWER_DUMP) &&
        connectionState == STATE_CONNECTED &&
        (packet.sequenceNumber % config.dumpInterval) == 0) {
        dumpBuffer(DUMP_BUFFER_POWER_HI,dump);
    }
    
    // Is there a detectable load?
    if(apparentPower > HI_GAIN_NO_LOAD_THRESHOLD) {
        // remember the high-gain analysis values
        apparentPowerSave = apparentPower;
        zeroXingDelaySave = zeroXingDelay;
        complexitySave = currentComplexity;
        nClipHi = nClipped;
    }
    
    //----------------------------------------------------------------------
    // Second time round uses the low-gain power channel.
    //----------------------------------------------------------------------
    acquireADCSamples(ACPOWER_PIN);
    
    // Analyze the captured low-gain waveform
    powerAnalysis(config.powerGainLo,config.fiducialShiftHi-config.fiducialHiLoDelta,dump);

    // Periodically dump sample buffer if requested
    if(dump && (config.capabilities & CAPABILITY_POWER_DUMP) &&
        connectionState == STATE_CONNECTED &&
        (packet.sequenceNumber % config.dumpInterval) == 0) {
        dumpBuffer(DUMP_BUFFER_POWER_LO,dump);
    }

    // Is the low-gain signal large enough to use, based on the high gain RMS?
    if(apparentPowerSave > LO_GAIN_NO_LOAD_THRESHOLD) {
        if(nClipHi < config.nClipCut) {
            // Perform a weighted average of the hi- and low-gain RMS values.
            // The weight here is based an eyeballing of the relative spreads
            // observed with a reference 71 Watt incandescent bulb.
            apparentPowerSave = (20*apparentPowerSave + apparentPower)/21.0;
        }
        else {
            // Defer to the less clipped low-gain RMS and complexity measurements
            apparentPowerSave = apparentPower;
            complexitySave = currentComplexity;
        }
        // A clipped high-gain zero crossing delay is still ok so always
        // combine the high- and low-gain results, taking care of
        // wrap-around issues.
        if(abs(zeroXingDelaySave - zeroXingDelay) < DELAY_WRAP_AROUND) {
            zeroXingDelaySave = 0.5*(zeroXingDelaySave + zeroXingDelay);
        }
        else {
            zeroXingDelaySave =
                0.5*(zeroXingDelaySave + zeroXingDelay) + DELAY_WRAP_AROUND;
        }
    }

    // Calculate the power factor (will be one if no load detected)
    _fval = fabs(cos(zeroXingDelaySave*POWER_FACTOR_OMEGA));
    
    LCDclear();
    Serial.print(apparentPowerSave);
    LCDpos(0,12);
    Serial.print(_fval);
    
    // Calculate the real power
    _fval *= apparentPowerSave;
    
    LCDpos(1,0);
    Serial.print(_fval);
    LCDpos(1,12);
    Serial.print(complexitySave,DEC);
    
    // update the click threshold based on the new power estimate
    // clickThreshold = (unsigned long)(THRESHOLDSCALE*rmsPower*rmsPower);
}

// =====================================================================
// Performs a "glow" sequence in which LEDs slowly glow on/off, the
// temperature is repeatedly sampled, and audio feedback on the
// power level is provided.
//
// Results are saved in:
//  - temperatureSum
//  - MS-bit of ledControl (tracks phase of glow ramp up/down)
// =====================================================================
void glowSequence() {

    // =====================================================================
    // Calculates the average of NTEMPSUM temperature ADC samples.
    // Result is stored in 32-bit unsigned, so can average up to 2^22
    // 10-bit ADC samples without overflow. The temperature sampling is
    // split into two phases, with the green and red LEDs ramping on
    // during the first phase and then off in the second phase. The
    // amber LED ramps twice as fast, completing a full cycle each phase.
    // =====================================================================

    // The first temperature sample sometimes reads low so don't use it
    analogRead(TEMPERATURE_PIN);

    // toggle the glow on/off phase
    LED_RAMP_TOGGLE;
    
    // disable all LEDs if the room is dark
    if(roomIsDark) LED_ALL_OFF;

    // Perform half of the temperature cycles
    for(_u16val = 0; _u16val < NTEMPSUMBY2; _u16val++) {
        // accumulate another temperature sample
        temperatureSum += analogRead(TEMPERATURE_PIN);
        // calculate the slow glow amplitude
        if(LED_IS_RAMPING_UP) {
            // ramp from 0-255 over NTEMPSUMBY2 temperature samples
            _u8val = (uint8_t)(127.*(1.-cos(_u16val*DPHIGLOW_SLOW))+0.5);
        }
        else {
            // ramp from 255-0 over NTEMPSUMBY2 temperature samples
            _u8val = (uint8_t)(127.*(1.+cos(_u16val*DPHIGLOW_SLOW))+0.5);
        }
        // set the green and red LEDs to this level if requested
        if(LED_IS_ENABLED(GREEN_GLOW)) analogWrite(GREEN_LED_PIN,_u8val);
        if(LED_IS_ENABLED(RED_GLOW)) analogWrite(RED_LED_PIN,_u8val);
        // calculate the fast glow amplitude: 0-255-0 in NTEMPSUMBY2 samples
        _u8val = (uint8_t)(127.*(1.-cos(_u16val*DPHIGLOW_FAST))+0.5);
        // set the amber LED to this level if requested
        if(LED_IS_ENABLED(AMBER_GLOW)) analogWrite(AMBER_LED_PIN,_u8val);
        // perform audio level feedback
        if(config.capabilities & CAPABILITY_POWER_LEVEL_AUDIO) tick();
    }
}

// =====================================================================
// Analyzes the temperature samples collected during previous
// glowSequences and performs LED flashing.
//
// Results are saved in:
//  - packet.temperature
//  - ledControl: flash bits are updated based on temperature
// =====================================================================
void flashSequence() {

    // calculate the average temperature in degF x 100
    packet.temperature = (unsigned int)(100*temperatureSum*ADC2DEGF);

    // adjust for self heating
    if(packet.temperature > config.selfHeatOffset) {
        _u16val = packet.temperature - config.selfHeatOffset;
        // has the self-heating delay expired? (the cycleCount check protects
        // against millis() wrap-around after 50 days)
        if((millis() > 10000UL*config.selfHeatDelay) || (cycleCount > 500)) {
            if(_u16val > 100U*config.comfortTempMax) {
                // we are above the comfort zone
                if(config.capabilities & CAPABILITY_TEMP_FEEDBACK) LED_ENABLE(RED_FLASH);
            }
            else if(_u16val < 100U*config.comfortTempMin) {
                // we are below the comfort zone
                if(config.capabilities & CAPABILITY_TEMP_FEEDBACK) LED_ENABLE(BLUE_FLASH);
            }
        }
    }

    // disable all LEDs if the room is dark
    if(roomIsDark) LED_ALL_OFF;
    
    // do the flashing now
    if(LED_IS_ENABLED(GREEN_FLASH)) digitalWrite(GREEN_LED_PIN,HIGH);
    if(LED_IS_ENABLED(AMBER_FLASH)) digitalWrite(AMBER_LED_PIN,HIGH);
    if(LED_IS_ENABLED(RED_FLASH)) digitalWrite(RED_LED_PIN,HIGH);
    if(LED_IS_ENABLED(BLUE_FLASH)) digitalWrite(BLUE_LED_PIN,HIGH);
    delay(FLASH_DURATION);
    digitalWrite(GREEN_LED_PIN,LOW);
    digitalWrite(AMBER_LED_PIN,LOW);
    digitalWrite(RED_LED_PIN,LOW);
    digitalWrite(BLUE_LED_PIN,LOW);
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
    if(sendNordic(lamAddress, (uint8_t*)&LAM, sizeof(LAM)) < 0x10) {
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

    // Dispaly our startup config to the optional LCD display.
    // Since this displays for 30s it should only be used for debug.
    // printConfig();

    // Send another LAM with our real serial number after a short delay.
    delay(2000);
    sendNordic(lamAddress, (uint8_t*)&LAM, sizeof(LAM));
    
    // Initialize our data packet. The device ID will be filled in later.
    // Even if there is no hub listening, we still use the packet's sequence
    // number counter for the passive feedback algorithms.
    packet.sequenceNumber = 0;
    packet.status = 0;
    
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
    
    // initialize the LED control register
    LED_INIT;
    
    // initialize our slow average temperature accumulator
    temperatureSum = 0;

    //----------------------------------------------------------------------
    // Measure the lighting conditions
    //----------------------------------------------------------------------
    lightingSequence(&dump);

    //----------------------------------------------------------------------
    // Ramp the glowing LEDs up while measuring temperature and giving
    // audio feedback on the power consumption level
    //----------------------------------------------------------------------
    glowSequence();
    
    //----------------------------------------------------------------------
    // Check for any incoming wireless data
    //----------------------------------------------------------------------
    if(getNordic((uint8_t*)&config,sizeof(config)) == PIPELINE_CONFIG) {
        handleConfigUpdate();
    }
    else {
        // We clobbered our config in RAM above, so restore it from EEPROM now.
        // (This should never happen since we only have one pipeline active,
        // but just in case...)
        loadConfig(&config);
    }
    
    //----------------------------------------------------------------------
    // Measure the power consumption
    //----------------------------------------------------------------------
    powerSequence(&dump);
    
    //----------------------------------------------------------------------
    // Ramp the glowing LEDs down while measuring temperature and giving
    // audio feedback on the power consumption level
    //----------------------------------------------------------------------
    glowSequence();

    //----------------------------------------------------------------------
    // Analyze the temperature samples and flash the LEDs
    //----------------------------------------------------------------------
    flashSequence();
        
    if(connectionState & STATE_CONNECTING) {
        // We are still waiting for a response from the hub. Send out another
        // request here...
        sendNordic(lamAddress, (uint8_t*)&LAM, sizeof(LAM));
    }
    else {
        // Transmit our data via the nordic interface. Save the return value
        // to send with the next packet.
        packet.status =
            sendNordic(dataAddress, (uint8_t*)&packet, sizeof(packet)) &
            STATUS_NUM_RETRANSMIT_MASK;
        // Keep track of the number of consecutive dropped data packets
        if(packet.status) {
            // increment dropped packet counter
            if((++connectionState & DROPPED_PACKETS_MASK) >= MAX_DROPPED_PACKETS) {
                // give up on the hub and try reconnecting
                connectionState = STATE_CONNECTING;
                // play the same falling sequence that, on startup, indicates
                // that no hub was found
                if(config.capabilities & CAPABILITY_AUDIO_DIAGNOSTICS) {
                    tone(750,100);
                    tone(1000,75);
                    tone(1500,50);
                }
            }
        }
        else {
            // zero out dropped-packet counter
            connectionState = STATE_CONNECTED;
        }
    }
    
    //----------------------------------------------------------------------
    // Display readings on the optional LCD
    //----------------------------------------------------------------------
    // printSample();
}

int main(void) {
    init();
    setup();
    for (;;) {
        loop();
    }
    return 0;
}