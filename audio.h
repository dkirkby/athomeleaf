#ifndef AUDIO_H
#define AUDIO_H

#include <stdint.h>

extern void tone(uint16_t halfPeriod, uint8_t duration);
extern void chirp(uint8_t cycles, uint8_t timebase);
extern void cricket();
// extern void bird(uint8_t repetions=3);

#endif
