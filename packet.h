#ifndef PACKET_H
#define PACKET_H

#define DATA_PACKET_VALUES 5

#define STATUS_NUM_RETRANSMIT_MASK 0x0f

#define STATUS_GOT_INITIAL_CONFIG 0x10
#define STATUS_GOT_UPDATED_CONFIG 0x20
#define STATUS_GOT_INVALID_CONFIG 0x40

typedef struct {
    unsigned short deviceID;
    unsigned char sequenceNumber;
    unsigned char status;
    unsigned short data[DATA_PACKET_VALUES];
} DataPacket;

typedef struct {
    unsigned long serialNumber;
    unsigned long commitTimestamp;
    unsigned char commitID[20];
    unsigned char modified;
} LookAtMe;

// When the MSB of deviceID is set, the status byte identifies one of
// the following special packet types

#define PACKET_DUMP_LIGHTING 0x01

#endif