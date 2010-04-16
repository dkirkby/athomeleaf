#ifndef PACKET_H
#define PACKET_H

#define PACKET_VALUES 4

typedef struct {
    unsigned short deviceID;
    unsigned char sequenceNumber;
    unsigned char status;
    unsigned short data[PACKET_VALUES];
} Packet;

#endif