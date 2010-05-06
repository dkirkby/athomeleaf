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

#define PIPELINE_DATA       1
#define PIPELINE_LOOK_AT_ME 2

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
// The setup() function is called once on startup.
// =====================================================================

void setup() {
    
    // copy our serial number from EEPROM to our LAM packet
    copySerialNumber(&LAM);

    pinMode(PIEZO_PIN,OUTPUT);
    pinMode(RED_LED_PIN,OUTPUT);
    
    // tell the world we are alive
    cricket();
    delay(500);
    cricket();
    delay(500);
    cricket();
    delay(500);
    
    // startup the serial port
    Serial.begin(115200);
    
    // print out our look-at-me config data (leading \n\n ensures that this
    // message is cleanly detected even with garbage in the serial input buffer)
    Serial.print("\n\nHUB ");
    printLookAtMe(&LAM);

    // try to initialize the wireless interface and print the result
    initNordic(0,1);
    if(!nordicOK) {
        Serial.println("ERROR 01 Unable to config wireless interface");
    }
}

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
        for(byteValue = 0; byteValue < DATA_PACKET_VALUES; byteValue++) {
            Serial.write(' ');
            Serial.print(data->data[byteValue],DEC);
        }
        Serial.write(' ');
        Serial.println(data->status,HEX);
        digitalWrite(RED_LED_PIN,LOW);
    }
    else if(pipeline == PIPELINE_LOOK_AT_ME) {
        lam = (const LookAtMe*)packetBuffer;
        Serial.print("LAM ");
        printLookAtMe(lam);
    }
    else if(pipeline < 6) {
        Serial.print("ERROR 02 unexpected data in P");
        Serial.println(pipeline,DEC);
    }
    else if(pipeline < 8) {
        Serial.print("ERROR 03 invalid RX_P_NO ");
        Serial.println(pipeline,DEC);
    }
    // Is there any serial input data?
    while(Serial.available() > 0 && serialBytes < SERIAL_BUFFER_SIZE) {
        if((serialBuffer[serialBytes++]= (byte)Serial.read()) == '\n') {
            // we now have a complete command in the buffer
            serialBuffer[serialBytes-1] = '\0';
            Serial.print("CMD ");
            Serial.println((const char*)serialBuffer);
            // Handle the command here...
            // ...
            // Reset the serial buffer
            serialBytes = 0;
        }
        else if(serialBytes == SERIAL_BUFFER_SIZE) {
            // buffer is now full and we still have not received a complete command
            Serial.println("ERROR 04 serial buffer overflow");
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