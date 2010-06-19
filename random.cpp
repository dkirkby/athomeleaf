#include "random.h"

// =====================================================================
// Generates a pseudo-random sequence of 32-bit unsigned integers
// =====================================================================

uint32_t randomValue = 24071966UL;

void nextRandom() {
    randomValue = randomValue*2891336453UL + 1640531513UL;
    randomValue ^= randomValue >> 13;
    randomValue ^= randomValue << 17;
    randomValue ^= randomValue >> 5;
}
