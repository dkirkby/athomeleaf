// =====================================================================
// Support for accessing our 32-bit serial number.
//
// Copyright (c) 2010 David Kirkby dkirkby@uci.edu
// =====================================================================
#include "serialno.h"

// Core EEPROM access routines
#include <avr/eeprom.h>

#define SERIAL_NUMBER_ADDR 0x10 // EEPROM offset where the serial number starts

// =====================================================================
// Returns our 32-bit serial number read from EEPROM
// =====================================================================
uint32_t serialNumber() {
    return eeprom_read_dword((uint32_t*)SERIAL_NUMBER_ADDR);
}
