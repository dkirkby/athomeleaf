#ifndef WIRELESS_H
#define WIRELESS_H
// =====================================================================
// Declares the wireless code shared between leaf nodes and the hub.
//
// Copyright (c) 2010 David Kirkby dkirkby@uci.edu
// =====================================================================

// Hub pipelines
#define PIPELINE_DATA        1
#define PIPELINE_LOOK_AT_ME  2
#define PIPELINE_BUFFER_DUMP 3

// Leaf pipelines
#define PIPELINE_CONFIG      1

extern byte nordicOK;
extern byte idleAddress[],dataAddress[],configAddress[],lamAddress[],dumpAddress[];

extern void initNordic(unsigned long serialNumber);
extern byte getNordic(byte *payload, byte payloadSize);
extern byte sendNordic(byte *address, byte *payload, byte payloadSize);

#endif
