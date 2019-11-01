#!/bin/sh
# TODO: use udev instead
set -e
make -B
set +e
rmmod altera_cvp
rmmod litepcie
#rm /dev/litepcie0
insmod litepcie.ko

#major=$(awk '/ litepcie$/{print $1}' /proc/devices)
#mknod -m 666 /dev/litepcie0 c $major 0

