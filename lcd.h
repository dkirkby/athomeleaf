#ifndef LCD_H
#define LCD_H

#include <stdint.h>

// Support for optional 16x2 LCD
extern void LCDinit(uint8_t backlightLevel = 5, uint8_t doConfig = 0);
extern void LCDprint(const char *line1, const char *line2 = 0);
extern void LCDclear();
extern void LCDpos(uint8_t row, uint8_t col=0);

#endif
