#!/usr/bin/env python

###################################################################################
# Uses the 'git log' command to get the timestamp and hash of the current commit.
###################################################################################

import sys
import subprocess

# is the working directory synchronized with the last commit?

try:
    proc = subprocess.Popen('git diff --name-only HEAD --exit-code',
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,shell=True)
    proc.wait()
    modified = proc.returncode
    output = proc.communicate()[0]
except OSError,e:
    print >>sys.stderr, "Subprocess execution failed:", e
    sys.exit(-1)

if modified == 0x7f:
    print >>sys.stderr, "Cannot determine if code is in synch with repository."
    sys.exit(-2)

# warn if we are not in synch
if modified == 1:
    print >>sys.stderr, ("Source code not in synch with repository:\n%s" % output),

# lookup the HEAD commit timestamp and 20-byte hash
try:
    proc = subprocess.Popen('git log -n 1 HEAD --pretty="format:%ct %H"',
       stdout=subprocess.PIPE,stderr=subprocess.STDOUT,shell=True)
    proc.wait()
    assert(proc.returncode == 0)
    output = proc.communicate()[0]
    (timestamp,hashString) = output.split()
    timestamp = int(timestamp)
    assert(len(hashString) == 40)
    # convert the hash string to a byte array
    hashData = [int(hashString[2*k:2*k+2],16) for k in range(20)]
except OSError,e:
    print >>sys.stderr, "Subprocess execution failed:", e
    sys.exit(-3)

# format the results as C initializers

status = modified << 4

print "%d, { %s }, %d" % (
    timestamp,','.join(["0x%02x" % b for b in hashData]),status)
