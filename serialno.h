#ifndef SERIALNO_H
#define SERIALNO_H
// =====================================================================
// Declares support for accessing our 32-bit serial number.
//
// Copyright (c) 2010 David Kirkby dkirkby@uci.edu
// =====================================================================
#include <stdint.h> // for uint32_t

#define HUB_SERIAL_NUMBER_MASK 0xff000000 // all hubs have these bits set in their SN
#define IS_HUB(sn) (((sn) & HUB_SERIAL_NUMBER_MASK) == HUB_SERIAL_NUMBER_MASK)

extern uint32_t serialNumber();

#endif
