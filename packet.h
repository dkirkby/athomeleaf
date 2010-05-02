#ifndef PACKET_H
#define PACKET_H

#define PACKET_VALUES 5

typedef struct {
    unsigned short deviceID;
    unsigned char sequenceNumber;
    unsigned char status;
    unsigned short data[PACKET_VALUES];
} Packet;

// When the MSB of deviceID is set, the status byte identifies one of
// the following special packet types

#define PACKET_DUMP_LIGHTING 0x01

#endif