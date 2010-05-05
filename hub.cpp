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

#define PIPELINE_DATA 1
#define PIPELINE_LAM  2

// Define a buffer big enough for any nordic packet.
byte packetBuffer[32];

// Use the pipeline number to cast the generic buffer to a pointer
// of the appropriate packet type.
const DataPacket *data;
const LookAtMe *lam;

// =====================================================================
// The setup() function is called once on startup.
// =====================================================================

void setup() {
    byte index;
    
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
    Serial.print(LAM.serialNumber,HEX);
    Serial.write(' ');
    Serial.print(LAM.commitTimestamp,DEC);
    Serial.write(' ');
    for(index = 0; index < 20; index++) {
        byteValue = LAM.commitID[index];
        if(byteValue < 0x10) {
            Serial.write('0');
        }
        Serial.print(byteValue,HEX);
    }
    Serial.write(' ');
    Serial.println(LAM.modified,DEC);

    // try to initialize the wireless interface and print the result
    initNordic(0,1);
    if(!nordicOK) {
        Serial.println("ERROR Unable to config wireless interface");
    }
}

void loop() {
    // is there any wireless data in our receive pipeline?
    pipeline = getNordic(packetBuffer,32);
    if(pipeline == PIPELINE_DATA) {
        digitalWrite(RED_LED_PIN,HIGH);
        data = (const DataPacket*)packetBuffer;
        Serial.print(data->deviceID,HEX);
        Serial.print(" [");
        Serial.print(data->sequenceNumber,HEX);
        Serial.print("]");
        for(byteValue = 0; byteValue < DATA_PACKET_VALUES; byteValue++) {
            Serial.print(' ');
            Serial.print(data->data[byteValue],DEC);
        }
        if(data->status) {
          Serial.print(" *");
          Serial.print(data->status,HEX);
        }
        Serial.println();
        digitalWrite(RED_LED_PIN,LOW);
    }
    else if (pipeline < 6) {
        Serial.print("ERROR unexpected data in P");
        Serial.println(pipeline,DEC);
    }
    else if(pipeline < 8) {
        Serial.print("ERROR invalid RX_P_NO ");
        Serial.println(pipeline,DEC);
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