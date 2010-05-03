#include "utilities.h"

// =====================================================================
// The setup() function is called once on startup.
// =====================================================================

void setup() {

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
    
    // print out the hub configuration on one line
    Serial.print("HUB SIZE ");
    Serial.print(sizeof(DataPacket),DEC);
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