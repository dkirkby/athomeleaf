#ifndef PACKET_H
#define PACKET_H
// =====================================================================
// Declares the packet data structures used for
// wireless communication between the leaf nodes and the hub.
//
// Copyright (c) 2010 David Kirkby dkirkby@uci.edu
// =====================================================================
#include <stdint.h>

#define CONFIG_HEADER 0xDeadBeef

#define CAPABILITY_TEMP_FEEDBACK     (1<<0)
#define CAPABILITY_LIGHT_FEEDBACK    (1<<1)
#define CAPABILITY_LIGHT_DUMP        (1<<2)
#define CAPABILITY_POWER_DUMP        (1<<3)

typedef struct {
    uint32_t header; // a fixed header to help filter spurious config packets
    uint8_t networkID; // a short identifier that uniquely identifies us on our local network
    uint8_t capabilities; // a bitmask of user-selectable device capabilities
    uint16_t temperatureMin; // comfort zone upper limit (degF x 100)
    uint16_t temperatureMax; // comfort zone lower limit (degF x 100)
} Config;

#define STATUS_NUM_RETRANSMIT_MASK 0x0f

#define STATUS_GOT_INITIAL_CONFIG 0x10
#define STATUS_GOT_UPDATED_CONFIG 0x20
#define STATUS_GOT_INVALID_CONFIG 0x40

typedef struct { // 18 bytes total
    uint8_t networkID;
    uint8_t sequenceNumber;
    uint8_t status;
    uint8_t acPhase;
    uint16_t powerLoGain;
    uint16_t powerHiGain;
    uint16_t lightLevelLoGain;
    uint16_t lightLevelHiGain;
    uint16_t light120HzLoGain;
    uint16_t light120HzHiGain;
    uint16_t temperature;
} DataPacket;

typedef struct { // 29 bytes total
    uint32_t serialNumber;
    uint32_t commitTimestamp;
    uint8_t commitID[20];
    uint8_t modified;
} LookAtMe;

#define DUMP_BUFFER_POWER_LO 0
#define DUMP_BUFFER_POWER_HI 1
#define DUMP_BUFFER_LIGHT_LO 2
#define DUMP_BUFFER_LIGHT_HI 3
#define DUMP_BUFFER_AC_PHASE 4

typedef struct { // 32 bytes total
    uint8_t networkID;
    uint8_t sequenceNumber;
    uint8_t packed[30];
} BufferDump;

// ---------------------------------------------------------------------
// Packs four consecutive 10-bit samples stored in the 16-bit words
// src[0:3] into the five consecutive bytes dst[0:4].
// ---------------------------------------------------------------------
inline void packSamples(const uint16_t *src, uint8_t *dst) {
    dst[0] = (uint8_t)(src[0] >> 2);
    dst[1] = ((uint8_t)src[0] << 6) | (uint8_t)(src[1] >> 4);
    dst[2] = ((uint8_t)src[1] << 4) | (uint8_t)(src[2] >> 6);
    dst[3] = ((uint8_t)src[2] << 2) | (uint8_t)(src[3] >> 8);
    dst[4] = ((uint8_t)src[3]);
}

// ---------------------------------------------------------------------
// Unpacks the dump buffer packing performed by packSamples
// ---------------------------------------------------------------------
inline void unpackSamples(const uint8_t *src, uint16_t *dst) {
    dst[0] = (src[0] << 2) | (src[1] >> 6);
    dst[1] = ((src[1] & 0x3f) << 4) | (src[2] >> 4);
    dst[2] = ((src[2] & 0x0f) << 6) | (src[3] >> 2);
    dst[3] = ((src[3] & 0x03) << 8) | (src[4]);
}

#endif