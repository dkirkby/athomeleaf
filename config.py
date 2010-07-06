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

# hard-coded config defaults:
networkID = 0 # value written here is ignored
capabilities = 0x03 # see packet.h for bit defns
dumpInterval = 16 # samples
comfortTempMin = 70 # degF
comfortTempMax = 80 # degF
selfHeatOffset = 0 # degF/100
selfHeatDelay = 0 # secs*10
fiducialHiLoDelta = 90 # us
fiducialShiftHi = 3000 # us
powerGainHi = 253 # mW/ADC
powerGainLo = 4500 # mW/ADC
nClipCut = 8 # samples
powerAudioControl = 0
lightFidHiLoDelta = 0 # microsecs
lightFidShiftHi = 0 # microsecs
lightGainHi = 16 # arb./ADC
lightGainHiLoRatio = 46562
darkThreshold = 250
artificialThreshold = 100

# pack the config data into a mutable buffer, leaving space for the 3-byte line header
buf = ctypes.create_string_buffer(64)
struct.pack_into('<IBHBBBHBBHHHBHBHBHHB',buf,3,
    serialNumber,networkID,capabilities,dumpInterval,
    comfortTempMin,comfortTempMax,selfHeatOffset,selfHeatDelay,
    fiducialHiLoDelta,fiducialShiftHi,powerGainHi,powerGainLo,nClipCut,
    powerAudioControl,lightFidHiLoDelta,lightFidShiftHi,lightGainHi,
	lightGainHiLoRatio,darkThreshold,artificialThreshold)
dataSize = 32 # bytes

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
