#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* pci */
// 5G
//#define PCI_FPGA_VENDOR_ID 0x10EE
//#define PCI_FPGA_DEVICE_ID 0x7022
//QPCIE
#define PCI_FPGA_VENDOR_ID 0x1172
#define PCI_FPGA_DEVICE_ID 0xe001
#define PCI_FPGA_BAR0_SIZE 0xa000

/* dma */
#define DMA_BUFFER_COUNT 256
#define DMA_ENDPOINT_COUNT 3


#endif /* __HW_CONFIG_H */
