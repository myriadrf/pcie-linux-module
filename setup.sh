#!/bin/sh
set -e
make -B
set +e

if lsmod | grep -Eq "^litepcie"; then
	rmmod litepcie
fi

insmod litepcie.ko

