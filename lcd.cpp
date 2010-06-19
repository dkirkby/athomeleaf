#include "lcd.h"

#include "WProgram.h" // arduino header

// =====================================================================
// Initializes an optional 16x2 serial LCD. There is no way to know
// if one is connected since the device is write-only.
// =====================================================================
void LCDinit(uint8_t backlightLevel) {
    Serial.begin(9600);
    // turn cursor off
    Serial.write(0xfe);
    Serial.write(0x0c);
    delay(100); // needed to allow LCD serial decoder to keep up?
    // turn backlight off
    Serial.write(0x7c);
    Serial.write((uint8_t)(0x80 | (backlightLevel % 30)));
    delay(100); // needed to allow LCD serial decoder to keep up?
}

// =====================================================================
// Clears the optional LCD display and writes the specified messages.
// The caller is responsible for ensuring that the message strings
// are no more than 16 characters long.
// =====================================================================
void LCDprint(const char *line1, const char *line2) {
    // clear the display
    Serial.write(0xfe);
    Serial.write(0x01);
    // show the first line
    Serial.print(line1);
    if(0 != line2) {
        // move to the second line
        Serial.write(0xfe);
        Serial.write(0xc0);
        Serial.print(line2);
    }
}

// =====================================================================
// Clears the optional LCD display
// =====================================================================
void LCDclear() {
    Serial.write(0xfe);
    Serial.write(0x01);
    delay(100);    
}

// =====================================================================
// Positions the (invisible) cursor on the optional LCD.
// row = 0 is the top line, row = 1 is the bottom line.
// valid col values are 0 (leftmost) to 15.
// =====================================================================
void LCDpos(uint8_t row, uint8_t col) {
    Serial.write(0xfe);
    if(row == 0) {
        Serial.write((uint8_t)(0x80 | (col & 0x0f)));
    }
    else {
        Serial.write((uint8_t)(0xc0 | (col & 0x0f)));
    }
    //delay(100);
}
