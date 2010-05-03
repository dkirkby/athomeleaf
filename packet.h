#ifndef PACKET_H
#define PACKET_H

#define DATA_PACKET_VALUES 5

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
    unsigned char synchedToCommit;
} LookAtMe;

// When the MSB of deviceID is set, the status byte identifies one of
// the following special packet types

#define PACKET_DUMP_LIGHTING 0x01

#endif