#ifndef PACKET_H
#define PACKET_H
// =====================================================================
// Declares the packet data structures used for wireless communication
// between the leaf nodes and the hub.
//
// Copyright (c) 2010 David Kirkby dkirkby@uci.edu
// =====================================================================

#define CONFIG_HEADER 0xDeadBeef

#define CAPABILITY_TEMP_FEEDBACK     (1<<0)
#define CAPABILITY_LIGHT_FEEDBACK    (1<<1)
#define CAPABILITY_LIGHT_DUMP        (1<<2)
#define CAPABILITY_POWER_DUMP        (1<<3)

typedef struct {
    unsigned long header; // a fixed header to help filter spurious config packets
    byte networkID; // a short identifier that uniquely identifies us on our local network
    byte capabilities; // a bitmask of user-selectable device capabilities
    unsigned short temperatureMin; // comfort zone upper limit (degF x 100)
    unsigned short temperatureMax; // comfort zone lower limit (degF x 100)
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

typedef struct { // 32 bytes total
    uint8_t networkID;
    uint8_t sequenceNumber;
    uint8_t packed[30];
} BufferDump;

#endif