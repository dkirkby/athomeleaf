// =====================================================================
// The hub program runs on an embedded microcontroller and coordinates
// a network of leaf nodes via wireless nordic transceivers. The hub
// passes on the data it receives via serial output and also listens
// for commands via serial input (115200 baud).
//
// Copyright (c) 2010 David Kirkby dkirkby@uci.edu
// =====================================================================

#include "serialno.h"
#include "wireless.h"
#include "packet.h"
#include "float16.h"

#include "WProgram.h" // arduino header

#define ADC_TO_MV 4.79421875 // 4910 mV / 1024
#define MV_TO_DEGF 0.1 // nominal conversion for LM34

// based on values in the HIH-5030 datasheet
#define RH_SLOPE 0.1535475629 // 1/(1024*0.00636)
#define RH_OFFSET 23.820754717 // 0.1515/0.00636

// Sensor ADC pin assignments
#define HUB_HUMIDITY_PIN 0
#define HUB_TEMPERATURE_PIN 6

// report sensor readings every minute ( = 600 x 100ms)
#define SENSOR_READING_PERIOD 100 // interval in ms between readings
#define SENSOR_READING_COUNT 600 // number of readings to accumulate for each update

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

// A pipeline number allows us to distinguish between different types
// of incoming wireless messages. 
uint8_t pipeline;

// Define a buffer big enough for any nordic packet.
uint8_t packetBuffer[32];

// Use the pipeline number to cast the generic buffer to a pointer
// of the appropriate packet type.
const DataPacket *data;
const LookAtMe *lam;
const BufferDump *dump;

// Serial input will be buffered here until we reach a newline that
// indicates a complete command.
#define SERIAL_BUFFER_SIZE 128
uint8_t serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialBytes = 0;

// Config data received via serial input will be assembled here
Config configData;

// Sensor reading globals
uint32_t now,lastReading = 0;
uint16_t elapsed,nSensorSum = 0;
uint32_t temperatureSum,humiditySum;
float temperature,humidity;

// Private globals used as shared temporaries to avoid stack locals
static uint8_t _u8val;
static uint16_t _u16val;
static uint32_t _u32val;
static float _fval;

// =====================================================================
// Dump the contents of a Look-at-Me packet to the serial port on
// a single line.
// =====================================================================

void printLookAtMe(const LookAtMe *lam) {
    uint8_t index;

    Serial.print(lam->serialNumber,HEX);
    Serial.write(' ');
    Serial.print(lam->commitTimestamp,DEC);
    Serial.write(' ');
    for(index = 0; index < 20; index++) {
        _u8val = lam->commitID[index];
        if(_u8val < 0x10) {
            Serial.write('0');
        }
        Serial.print(_u8val,HEX);
    }
    Serial.write(' ');
    Serial.println(lam->status,DEC);    
}

// =====================================================================
// Print the contents of a single dump buffer packet on one line.
// =====================================================================

void printBufferDump(const BufferDump *dump) {
    uint16_t unpacked[4];
    uint32_t timestamp;
    
    Serial.print(dump->networkID,HEX);
    Serial.write(' ');
    Serial.print(dump->sequenceNumber,HEX);
    if(dump->sequenceNumber == 0) {
        Serial.write(' ');
        // dump the first 11 bytes as one long hex string
        for(_u8val = 0; _u8val < 11; _u8val++) {
            if(dump->packed[_u8val] < 0x10) Serial.write('0');
            Serial.print(dump->packed[_u8val],HEX);
        }
        Serial.write(' ');
        // print the dump type
        Serial.print(dump->packed[11],HEX);
        Serial.write(' ');
        // print the timestamp
        timestamp = *(uint32_t*)(&dump->packed[12]);
        Serial.print(timestamp,HEX);
        // the first two samples are unpacked
        unpacked[0] = *(uint16_t*)(&dump->packed[16]);
        Serial.write(' ');
        Serial.print(unpacked[0],HEX);
        unpacked[1] = *(uint16_t*)(&dump->packed[18]);
        Serial.write(' ');
        Serial.print(unpacked[1],HEX);
        // the next 8 samples are packed        
        unpackSamples(&dump->packed[20],unpacked);
        for(_u8val = 0; _u8val < 4; _u8val++) {
            Serial.write(' ');
            Serial.print(unpacked[_u8val],HEX);
        }
        unpackSamples(&dump->packed[25],unpacked);
        for(_u8val = 0; _u8val < 4; _u8val++) {
            Serial.write(' ');
            Serial.print(unpacked[_u8val],HEX);
        }
    }
    else {
        // The next 10 packets have identical formats and pack 24
        // 10-bit ADC samples into 30 bytes. We unpack each sample
        // and print its hex value here.
        for(_u8val = 0; _u8val < 24; _u8val++) {
            if(_u8val % 4 == 0) {
                // refill our array of unpacked data
                unpackSamples(&dump->packed[5*(_u8val>>2)],unpacked);
            }
            Serial.write(' ');
            Serial.print(unpacked[_u8val % 4],HEX);
        }
    }
    Serial.println();
}

// =====================================================================
// Parse the characters ptr[0:1] as a hex string and return the result
// in the uintValue global. Returns a value > 0xff in case of a parse
// error. Upper or lower-case hex digits are allowed.
// =====================================================================

void parseHex(uint8_t *ptr) {
    _u16val = 0;
    if(*ptr >= '0' && *ptr <= '9') {
        _u16val = (*ptr-'0') << 4;
    }
    else if(*ptr >= 'A' && *ptr <= 'F') {
        _u16val = ((*ptr-'A') + 0xA) << 4;
    }
    else if(*ptr >= 'a' && *ptr <= 'f') {
        _u16val = ((*ptr-'a') + 0xA) << 4;
    }
    else {
        _u16val = 0x100;
    }
    ptr++;
    if(*ptr >= '0' && *ptr <= '9') {
        _u16val |= (*ptr-'0');
    }
    else if(*ptr >= 'A' && *ptr <= 'F') {
        _u16val |= ((*ptr-'A') + 0xA);
    }
    else if(*ptr >= 'a' && *ptr <= 'f') {
        _u16val |= ((*ptr-'a') + 0xA);
    }
    else {
        _u16val |= 0x200;
    }
}

// =====================================================================
// Handle a  config command in serialBuffer[0:serialBytes-1] and return
// zero for success or else a non-zero error code (1-6).
// =====================================================================

uint8_t handleConfigCommand() {

    uint8_t *ptr;

    // A valid config command looks like:
    // C ssssssss dddddd...dd\0
    // where ssssssss is exactly 8 hex digits of device serial number
    // and dd...dd is an even number of hex digits of config data.
    if(serialBytes < 10 || serialBuffer[0] !='C' || serialBuffer[1] != ' ' ||
        serialBuffer[10] != ' ' || (serialBytes-10)%2 != 0) return 1;
    
    // Check that the amount of data provided matches sizeof(Config) corrected
    // for the fixed config header.
    if(serialBytes != 12 + 2*(sizeof(configData) - sizeof(configData.header))) {
        return 2;
    }

    // Extract the least-significant 2 bytes of serial number and
    // copy them into the configAddress buffer
    parseHex(serialBuffer+6);
    if(_u16val > 0xff) return 3;
    configAddress[1] = (uint8_t)_u16val;
    parseHex(serialBuffer+8);
    if(_u16val > 0xff) return 4;
    configAddress[0] = (uint8_t)_u16val;

    // Config data always starts with a fixed header
    configData.header = CONFIG_HEADER;

    // Do a byte-wise copy of data from the command into the
    // remaining bytes of our config buffer
    ptr = (uint8_t*)&configData + sizeof(configData.header);
    _u8val = 11;
    while(_u8val < serialBytes-1) { // up to but not including the final \0
        parseHex(serialBuffer + _u8val);
        if(_u16val > 0xff) return 5;
        *ptr++ = (uint8_t)_u16val;
        _u8val += 2;
    }

    // If we get here, configAdddress and configData should be ready to go.
    // Send the config packet now.
    _u8val = sendNordic(configAddress,(uint8_t*)&configData,sizeof(configData));
    if(_u8val > 0x0f) {
        // Config packet was never acknowledged
        return 6;
    }
    else {
        // Config command has been validated and sucessfully executed
        return 0;
    }
}

// =====================================================================
// The setup() function is called once on startup.
// =====================================================================

void setup() {
    
    // copy our serial number from EEPROM to our LAM buffer
    LAM.serialNumber = serialNumber();

    // startup the serial port
    Serial.begin(115200);
    
    // print out our look-at-me config data (leading \n\n ensures that this
    // message is cleanly detected even with garbage in the serial input buffer)
    Serial.print("\n\nHUB ");
    printLookAtMe(&LAM);

    // try to initialize the wireless interface and print the result
    initNordic(LAM.serialNumber);
    if(!nordicOK) {
        Serial.println("LOG 1 0"); // Unable to config wireless interface
    }
    else {
        Serial.println("LOG 0 0"); // Hub started normally
    }
}

// =====================================================================
// The loop() function is called repeatedly forever after setup().
// =====================================================================

void loop() {
    
    // is there any wireless data in our receive pipeline?
    pipeline = getNordic(packetBuffer,32);
    if(pipeline == PIPELINE_DATA) {
        data = (const DataPacket*)packetBuffer;
        Serial.print("DATA ");
        Serial.print(data->networkID,HEX);
        Serial.write(' ');
        Serial.print(data->sequenceNumber,HEX);
        Serial.write(' ');
        Serial.print(data->status,HEX);
        Serial.write(' ');
        // Convert float16 values and forward as binary IEEE floats
        _fval = from_float16(data->lighting);
        _u32val = BITS(_fval);
        Serial.print(_u32val,HEX);
        Serial.write(' ');
        Serial.print(data->artificial,HEX);
        Serial.write(' ');
        Serial.print(data->lightFactor,HEX);
        Serial.write(' ');
        _fval = from_float16(data->power);
        _u32val = BITS(_fval);
        Serial.print(BITS(_u32val),HEX);
        Serial.write(' ');
        Serial.print(data->powerFactor,HEX);
        Serial.write(' ');
        Serial.print(data->complexity,HEX);
        Serial.write(' ');
        Serial.println(data->temperature,HEX);
    }
    else if(pipeline == PIPELINE_LOOK_AT_ME) {
        lam = (const LookAtMe*)packetBuffer;
        Serial.print("LAM ");
        printLookAtMe(lam);
    }
    else if(pipeline == PIPELINE_BUFFER_DUMP) {
        dump = (const BufferDump*)packetBuffer;
        Serial.print("DUMP ");
        printBufferDump(dump);
    }
    else if(pipeline < 6) {
        Serial.print("LOG 2 ");  /* Unexpected data in pipeline */
        Serial.println(pipeline,DEC);
    }
    else if(pipeline < 8) {
        Serial.print("LOG 3 "); /* invalid RX_P_NO */
        Serial.println(pipeline,DEC);
    }
    // Is there any serial input data?
    while(Serial.available() > 0 && serialBytes < SERIAL_BUFFER_SIZE) {
        if((serialBuffer[serialBytes++]= (uint8_t)Serial.read()) == '\n') {
            // we now have a complete command in the buffer
            serialBuffer[serialBytes-1] = '\0';
            // we only accept a config command (for now) - handle it here
            _u8val = handleConfigCommand();
            if(0 == _u8val) {
                Serial.print("LOG 4 "); /* config command successfully handled */
                Serial.println(configData.networkID,DEC);
            }
            else {
                Serial.print("CFG ERR ");
                Serial.println((char*)serialBuffer);
                Serial.print("LOG 5 "); /* config handler reported an error */
                Serial.println(_u8val,DEC);
            }
            // Reset the serial buffer
            serialBytes = 0;
        }
        else if(serialBytes == SERIAL_BUFFER_SIZE) {
            // buffer is now full and we still have not received a complete command
            Serial.print("LOG 6 "); /* serial buffer overflow */
            Serial.println(SERIAL_BUFFER_SIZE,DEC);
            serialBytes = 0;
        }
    }
    // Is it time for periodic sensor readings?
    now = millis();
    if(now < lastReading) {
        elapsed = (uint16_t)(0xFFFFFFFF - lastReading + now);
    }
    else {
        elapsed = (uint16_t)(now - lastReading);
    }
    if(elapsed >= SENSOR_READING_PERIOD) {
        temperatureSum += analogRead(HUB_TEMPERATURE_PIN);
        humiditySum += analogRead(HUB_HUMIDITY_PIN);
        lastReading = now;
        nSensorSum++;
        // time to report sensor readings?
        if(nSensorSum >= SENSOR_READING_COUNT) {
            Serial.print("SENS ");
            // calculate the temperature in degF
            temperature = (temperatureSum*ADC_TO_MV*MV_TO_DEGF)/nSensorSum;
            Serial.print(temperature);
            Serial.write(' ');
            // calculate the relative humidity in percent
            humidity = (humiditySum*RH_SLOPE)/nSensorSum - RH_OFFSET;
            // apply temperature correction
            temperature = (temperature - 32.0)/1.8; // degF -> degC
            humidity /= (1.0546 - 0.00216*temperature);
            Serial.println(humidity);
            // reset for next accumulation cycle
            temperatureSum = humiditySum = 0;
            nSensorSum = 0;
        }
    }
}

#include <avr/wdt.h>

int main(void) {
    // record the reason we just started in our LAM packet
    LAM.status |= MCUSR;
    // prevent infinite watchdog timer reset loops
    MCUSR = 0;
    wdt_disable();
    // run the arduino boot sequence (without any watchdog timeout)
    init();
    // run our application boot sequence (without any watchdog timeout)
    setup();
    // enable an 8-second watchdog timer
    wdt_enable(WDTO_8S);
    for (;;) {
        // If the loop takes longer than 8 seconds, we will automatically reset
        loop();
        wdt_reset();
    }
    return 0;
}