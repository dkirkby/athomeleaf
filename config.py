#!/usr/bin/env python

###################################################################################
## See http://en.wikipedia.org/wiki/SREC_(file_format) for a description of
## the S-record format file this script writes. The ATMega MCU is little endian.
###################################################################################

import sys
import struct
import ctypes

# parse command-line args
assert(len(sys.argv)==2)
(serialNumber,) = sys.argv[1:]

# perform string conversions
serialNumber = int(serialNumber,0)
print "Using serial number %d = $%08x" % (serialNumber,serialNumber)

# pack the config data into a mutable buffer, leaving space for the 3-byte line header
buf = ctypes.create_string_buffer(64)
struct.pack_into('<I',buf,3,serialNumber)
dataSize = 4

# store the header (byte count, start address)
byteCount = 3 + dataSize
startAddr = 0
struct.pack_into('>BH',buf,0,byteCount,startAddr)

# calculate the checksum
cksum = 0
for byte in buf[0:byteCount-1]:
    cksum = (cksum + ord(byte)) & 0xff
buf[byteCount] = chr(0xff - cksum)
byteCount += 1

# write an S-record file for initializing the EEPROM memory
sys.stdout.write("S1")
for byte in buf[0:byteCount]:
    sys.stdout.write("%02x" % ord(byte))
sys.stdout.write("\nS9030000FC\n")
