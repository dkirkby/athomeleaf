#ifndef PINS_H
#define PINS_H
// ---------------------------------------------------------------------
// Leaf node MCU I/O pin assignments
// ---------------------------------------------------------------------

// analog inputs:
// 0-3 referenced to AVCC
// 4-5 referenced to VCC (noisier?)
// 6-7 only available in 32-pin package

#define TEMPERATURE_PIN  3

#define ACPOWER_PIN      0
#define ACPOWER_PIN_HI   2
#define ACPHASE_PIN      1

#define LIGHTING_PIN     6
#define LIGHTING_PIN_HI  7

// PWM outputs (3,5,6,9,10,11 but 5,6 shared with timing routines and 10,11 used for SPI)
#define GREEN_LED_PIN    5
#define AMBER_LED_PIN    6

// digital I/O: use pins 2,4,5,6,7,8
// reserved: 0,1(UART); 3,5,6,9(PWM); 10-13(SPI)
#define PIEZO_PIN        2

#define RED_LED_PIN      3
#define BLUE_LED_PIN     4

#define STROBE_PIN       9

#endif
