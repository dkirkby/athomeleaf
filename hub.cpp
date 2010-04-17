#include "utilities.h"

// =====================================================================
// Global variable declarations. All variables must fit within 2K
// of SRAM, including any variables allocated on the stack at runtime.
// =====================================================================

Packet packet;

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
    Serial.println("Hub Starting...");

    initNordic(1);
    if(nordicOK) {
        Serial.println("Nordic configured");
    }
    else {
        Serial.println("Nordic config ERROR!");
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
            for(byteValue = 0; byteValue < PACKET_VALUES; byteValue++) {
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