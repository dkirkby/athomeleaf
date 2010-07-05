#ifndef WIRELESS_H
#define WIRELESS_H
// =====================================================================
// Declares the wireless code shared between leaf nodes and the hub.
//
// Copyright (c) 2010 David Kirkby dkirkby@uci.edu
// =====================================================================
#include <stdint.h> // for uint8_t,uint32_t

// Hub pipelines
#define PIPELINE_DATA        1
#define PIPELINE_LOOK_AT_ME  2
#define PIPELINE_BUFFER_DUMP 3

// Leaf pipelines
#define PIPELINE_CONFIG      1

// Special return values
#define NORDIC_NOT_READY 0xFF
#define NORDIC_NO_DATA   0xF0

extern uint8_t nordicOK;
extern uint8_t dataAddress[],configAddress[],lamAddress[],dumpAddress[];

extern void initNordic(uint32_t serialNumber);
extern uint8_t getNordic(uint8_t *payload, uint8_t payloadSize);
extern uint8_t sendNordic(uint8_t *address, uint8_t *payload, uint8_t payloadSize);

#endif
