// =====================================================================
// The hub program runs on an embedded microcontroller and coordinates
// a network of leaf nodes via wireless nordic transceivers. The hub
// passes on the data it receives via serial output and also listens
// for commands via serial input (115200 baud).
//
// Copyright (c) 2010 David Kirkby dkirkby@uci.edu
// =====================================================================

#include "utilities.h"

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
byte pipeline;

// Define a buffer big enough for any nordic packet.
byte packetBuffer[32];

// Use the pipeline number to cast the generic buffer to a pointer
// of the appropriate packet type.
const DataPacket *data;
const LookAtMe *lam;

// Serial input will be buffered here until we reach a newline that
// indicates a complete command.
#define SERIAL_BUFFER_SIZE 128
byte serialBuffer[SERIAL_BUFFER_SIZE];
byte serialBytes = 0;

// Config data received via serial input will be assembled here
Config configData;

// =====================================================================
// Dump the contents of a Look-at-Me packet to the serial port on
// a single line.
// =====================================================================

void printLookAtMe(const LookAtMe *lam) {
    byte index;

    Serial.print(lam->serialNumber,HEX);
    Serial.write(' ');
    Serial.print(lam->commitTimestamp,DEC);
    Serial.write(' ');
    for(index = 0; index < 20; index++) {
        byteValue = lam->commitID[index];
        if(byteValue < 0x10) {
            Serial.write('0');
        }
        Serial.print(byteValue,HEX);
    }
    Serial.write(' ');
    Serial.println(lam->modified,DEC);    
}

// =====================================================================
// Parse the characters ptr[0:1] as a hex string and return the result
// in the uintValue global. Returns a value > 0xff in case of a parse
// error. Upper or lower-case hex digits are allowed.
// =====================================================================

void parseHex(byte *ptr) {
    uintValue = 0;
    if(*ptr >= '0' && *ptr <= '9') {
        uintValue = (*ptr-'0') << 4;
    }
    else if(*ptr >= 'A' && *ptr <= 'F') {
        uintValue = ((*ptr-'A') + 0xA) << 4;
    }
    else if(*ptr >= 'a' && *ptr <= 'f') {
        uintValue = ((*ptr-'a') + 0xA) << 4;
    }
    else {
        uintValue = 0x100;
    }
    ptr++;
    if(*ptr >= '0' && *ptr <= '9') {
        uintValue |= (*ptr-'0');
    }
    else if(*ptr >= 'A' && *ptr <= 'F') {
        uintValue |= ((*ptr-'A') + 0xA);
    }
    else if(*ptr >= 'a' && *ptr <= 'f') {
        uintValue |= ((*ptr-'a') + 0xA);
    }
    else {
        uintValue |= 0x200;
    }
}

// =====================================================================
// Handle a  config command in serialBuffer[0:serialBytes-1] and return
// zero for success or else a non-zero error code (1-6).
// =====================================================================

byte handleConfigCommand() {

    byte *ptr;

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
    if(uintValue > 0xff) return 3;
    configAddress[1] = (byte)uintValue;
    parseHex(serialBuffer+8);
    if(uintValue > 0xff) return 4;
    configAddress[0] = (byte)uintValue;

    // Config data always starts with a fixed header
    configData.header = CONFIG_HEADER;

    // Do a byte-wise copy of data from the command into the
    // remaining bytes of our config buffer
    ptr = (byte*)&configData + sizeof(configData.header);
    byteValue = 11;
    while(byteValue < serialBytes-1) { // up to but not including the final \0
        parseHex(serialBuffer + byteValue);
        if(uintValue > 0xff) return 5;
        *ptr++ = (byte)uintValue;
        byteValue += 2;
    }

    // If we get here, configAdddress and configData should be ready to go.
    // Send the config packet now.
    byteValue = sendNordic(configAddress,(byte*)&configData,sizeof(configData));
    if(byteValue > 0x0f) {
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

    pinMode(PIEZO_PIN,OUTPUT);
    pinMode(RED_LED_PIN,OUTPUT);
    pinMode(STROBE_PIN,OUTPUT);
    
    // startup the serial port
    Serial.begin(115200);
    
    // tell the world we are alive
    cricket();
    delay(500);
    bird();
    
    // print out our look-at-me config data (leading \n\n ensures that this
    // message is cleanly detected even with garbage in the serial input buffer)
    Serial.print("\n\nHUB ");
    printLookAtMe(&LAM);

    // try to initialize the wireless interface and print the result
    initNordic(LAM.serialNumber);
    if(!nordicOK) {
        Serial.println("ERROR 1 Unable to config wireless interface");
    }
}

// =====================================================================
// The loop() function is called repeatedly forever after setup().
// =====================================================================

void loop() {
    // is there any wireless data in our receive pipeline?
    pipeline = getNordic(packetBuffer,32);
    if(pipeline == PIPELINE_DATA) {
        digitalWrite(RED_LED_PIN,HIGH);
        data = (const DataPacket*)packetBuffer;
        Serial.print("DATA ");
        Serial.print(data->deviceID,HEX);
        Serial.write(' ');
        Serial.print(data->sequenceNumber,HEX);
        Serial.write(' ');
        Serial.print(data->status,HEX);
        for(byteValue = 0; byteValue < DATA_PACKET_VALUES; byteValue++) {
            Serial.write(' ');
            Serial.print(data->data[byteValue],DEC);
        }
        Serial.println();
        digitalWrite(RED_LED_PIN,LOW);
    }
    else if(pipeline == PIPELINE_LOOK_AT_ME) {
        digitalWrite(RED_LED_PIN,HIGH);
        lam = (const LookAtMe*)packetBuffer;
        Serial.print("LAM ");
        printLookAtMe(lam);
        digitalWrite(RED_LED_PIN,LOW);
    }
    else if(pipeline < 6) {
        Serial.print("ERROR 2 unexpected data in P");
        Serial.println(pipeline,DEC);
    }
    else if(pipeline < 8) {
        Serial.print("ERROR 3 invalid RX_P_NO ");
        Serial.println(pipeline,DEC);
    }
    // Is there any serial input data?
    while(Serial.available() > 0 && serialBytes < SERIAL_BUFFER_SIZE) {
        if((serialBuffer[serialBytes++]= (byte)Serial.read()) == '\n') {
            // we now have a complete command in the buffer
            serialBuffer[serialBytes-1] = '\0';
            Serial.print("GOT ");
            Serial.println((const char*)serialBuffer);
            // we only accept a config command (for now) - handle it here
            byteValue = handleConfigCommand();
            if(0 == byteValue) {
                Serial.println("OK");
            }
            else {
                Serial.print("ERROR 4 ");
                // the handler return value is our error sub-code
                Serial.print(byteValue,DEC);
                Serial.print(" bad cmd: ");
                Serial.println((const char*)serialBuffer);                
            }
            // Reset the serial buffer
            serialBytes = 0;
        }
        else if(serialBytes == SERIAL_BUFFER_SIZE) {
            // buffer is now full and we still have not received a complete command
            Serial.println("ERROR 5 serial buffer overflow");
            serialBytes = 0;
        }
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