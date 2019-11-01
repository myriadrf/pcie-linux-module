/*
 * LitePCIe driver
 *
 */
#ifndef _LINUX_LITEPCIE_H
#define _LINUX_LITEPCIE_H

#include <linux/types.h>
#include "config.h"


struct litepcie_ioctl_reg_rw {
    __u32 adress;
    __u32 valWrite;
    __u32 valRead;
};

#define LITEPCIE_IOCTL 'S'

#define LITEPCIE_IOCTL_REG_READ         _IOWR(LITEPCIE_IOCTL, 6, struct litepcie_ioctl_reg_rw)
#define LITEPCIE_IOCTL_REG_WRITE         _IOWR(LITEPCIE_IOCTL, 7, struct litepcie_ioctl_reg_rw)

#endif /* _LINUX_LITEPCIE_H */
