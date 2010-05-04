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
    
    // print out our look-at-me config data
    Serial.print("HUB SERIAL# ");
    Serial.println(LAM.serialNumber,HEX);
    Serial.print("COMMIT ");
    Serial.print(LAM.commitTimestamp,DEC);
    Serial.write(' ');
    for(index = 0; index < 20; index++) {
        byteValue = LAM.commitID[index];
        if(byteValue < 0x10) {
            Serial.write('0');
        }
        Serial.print(byteValue,HEX);
    }
    if(LAM.modified) {
        Serial.write('+');
    }
    Serial.println();

    // try to initialize the wireless interface and print the result
    initNordic(0,1);
    if(nordicOK) {
        Serial.println("READY");
    }
    else {
        Serial.println("ERROR");
    }
    // flush any pending data before we start looping
    while(Mirf.dataReady()) {
        do {
            Mirf.getData((byte*)&packet);
        } while(!Mirf.rxFifoEmpty());
    }
}

void loop() {
    if(Mirf.dataReady()) {
        digitalWrite(RED_LED_PIN,HIGH);
        do {
            Mirf.getData((byte*)&packet);
            Serial.print(packet.deviceID,HEX);
            Serial.print(" [");
            Serial.print(packet.sequenceNumber,HEX);
            Serial.print("]");
            for(byteValue = 0; byteValue < DATA_PACKET_VALUES; byteValue++) {
                Serial.print(' ');
                Serial.print(packet.data[byteValue],DEC);
            }
            if(packet.status) {
              Serial.print(" *");
              Serial.print(packet.status,HEX);
            }
            Serial.println();
        } while(!Mirf.rxFifoEmpty());
        digitalWrite(RED_LED_PIN,LOW);
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