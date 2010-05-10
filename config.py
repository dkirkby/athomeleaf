#!/usr/bin/env python

###################################################################################
## Generates an EEPROM memory image for uploading to the MCU. Run with a single
## parameter to specify the device's 32-bit serial number. Either decimal or
## hex is supported, for example:
##
## config.py 123
## config.py 0xDeadBeef
##
## See http://en.wikipedia.org/wiki/SREC_(file_format) for a description of
## the S-record format file this script writes. The ATMega MCU is little endian.
###################################################################################

import sys
import struct
import ctypes

# parse command-line args
if len(sys.argv) != 2:
    sys.stderr.write("Missing SERIAL_NUMBER parameter\n")
    sys.exit(-1)
(serialNumber,) = sys.argv[1:]

# perform string conversions
serialNumber = int(serialNumber,0)

# hard-coded config defaults
networkID = 0
capabilities = 0xff
temperatureMin = 7000 # degF x 100
temperatureMax = 8000 # degF x 100

# pack the config data into a mutable buffer, leaving space for the 3-byte line header
buf = ctypes.create_string_buffer(64)
struct.pack_into('<IBBHH',buf,3,
    serialNumber,networkID,capabilities,temperatureMin,temperatureMax)
dataSize = 4 + 1 + 1 + 2 + 2

# store the header (byte count, start address)
byteCount = 3 + dataSize
startAddr = 0x10
struct.pack_into('>BH',buf,0,byteCount,startAddr)

# calculate the checksum
cksum = 0
for byte in buf[0:byteCount]:
    cksum = (cksum + ord(byte)) & 0xff
buf[byteCount] = chr(0xff - cksum)
byteCount += 1

# write an S-record file for initializing the EEPROM memory
sys.stdout.write("S1")
for byte in buf[0:byteCount]:
    sys.stdout.write("%02X" % ord(byte))
sys.stdout.write("\nS9030000FC\n")
