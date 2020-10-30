#!/bin/bash
# This script automatically installs litepcie kernel module
KERNEL_VER:=$(shell uname -r)
MODULE_LOC:=$(KERNEL_VER)/extra
SOURCE_LOC:=/usr/src/litepcie
DEPMOD:=false
# Build and install kernel module
if [ -e $(MODULE_LOC)/litepcie.ko ] ; then
    echo "litepcie module already exists for this kernel"
else
    echo "Building litepcie module" >&2
    make -C $(SOURCE_LOC)/
    mkdir -p $(MODULES_LOC)
    cp $(SOURCE_LOC)/litepcie.ko $(MODULE_LOC)/
    
    if [ -e $(MODULE_LOC)/litepcie.ko ] ; then
        echo "   SUCCESS: litepcie module installed for kernel"
    else
        echo "   FAILURE: failed to install litepcie module"
    fi
    DEPMOD:=true
fi

# Configure /etc/modules
if [ ! grep -xq "litepcie" /etc/modules ] ; then
    echo "litepcie" > /etc/modules
fi

# Run depmod if needed
if [ "$DEPMOD" = true ] ; then
    exec depmod
fi
# Check if module is already loaded
if lsmod | grep -Eq "^litepcie"; then
	echo "litepcie module already loaded"
fi



exit 0
