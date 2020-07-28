#!/bin/bash
# This script automatically installs litepcie kernel module
SOURCE_LOC="/usr/src/litepcie"
KERNEL_VER="/lib/modules/$(uname -r)"
MODULE_LOC=$KERNEL_VER/extra

echo "$(date -R)" >> /var/tmp/litepcie.log

if ! lsmod | grep -Eq "^litepcie" ; then
        touch /var/tmp/litepcie.log
        echo "litepcie module not loaded" >> /var/tmp/litepcie.log
        #echo $MODULE_LOC
        make -C $SOURCE_LOC/ clean
        make -C $SOURCE_LOC/ || exit 1
        mkdir -p $MODULE_LOC
        cp $SOURCE_LOC/litepcie.ko $MODULE_LOC/
        depmod
        modprobe -v litepcie >> /var/tmp/litepcie.log

        # Configure /etc/modules to load it automatically
        if ! grep -xq "litepcie" /etc/modules ; then
            echo "litepcie" >> /etc/modules
        fi
        
        if lsmod | grep -Eq "^litepcie"; then
            echo "SUCCESS: litepcie module loaded" >> /var/tmp/litepcie.log
        else
            echo "FAILURE: litepcie install failed" >> /var/tmp/litepcie.log
        fi
else
    echo "litepcie module was loaded successfully" >> /var/tmp/litepcie.log
fi

exit 0
