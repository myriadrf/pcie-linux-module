/*
 * LitePCIe driver
 */
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/mmtimer.h>
#include <linux/miscdevice.h>
#include <linux/posix-timers.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
#include <linux/pci-aspm.h>
#endif
#include <linux/pci_regs.h>
#include <linux/delay.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/signal.h>
#endif

#include "litepcie.h"
#include "config.h"
#include "csr.h"
#include "flags.h"

#define LITEPCIE_NAME "litepcie"
#define LITEPCIE__EP_COUNT (DMA_ENDPOINT_COUNT*2+1)
#define LITEPCIE__DEVICE_COUNT 4
#define DMA_BUFFER_SIZE PAGE_ALIGN(65536)
#define DMA_BUFFER_MAP_SIZE (DMA_BUFFER_SIZE * DMA_BUFFER_COUNT)
#define MIN_TRANSFER_SIZE 4096

typedef struct litepcie_device litepcie_device;

typedef struct
{
        uint8_t* dma_buf[DMA_BUFFER_COUNT];
        size_t dma_bufs_addr[DMA_BUFFER_COUNT];
        uint16_t index;
        uint16_t offset;
        uint32_t buf_size;
        uint8_t started;
        uint8_t dma_ep;
        uint8_t is_tx;
        litepcie_device *dev;
} litepcie_channel;


typedef struct litepcie_device
{
	struct pci_dev *dev;
	uint8_t *bar0_addr; /* virtual address of BAR0 */
	uint8_t ch_cnt;
	litepcie_channel *channels;
	struct cdev cdev;
}litepcie_device;

static struct class *litepcie_class;
static litepcie_device *litepcie_device_table[LITEPCIE__DEVICE_COUNT] = {NULL};


static void litepcie_end(struct pci_dev *dev, litepcie_device *s);
static int litepcie_init_chrdev(litepcie_device *s, int index);
static int litepcie_pci_probe(struct pci_dev *dev, const struct pci_device_id *id);

static inline uint32_t litepcie_readl(litepcie_device *s, uint32_t addr)
{
	return readl(s->bar0_addr + addr);
}

static inline void litepcie_writel(litepcie_device *s, uint32_t addr, uint32_t val)
{
	return writel(val, s->bar0_addr + addr);
}

static int litepcie_open(struct inode *inode, struct file *file)
{
	litepcie_device *s = NULL;
        int i = 0;
	int minor = iminor(inode);
        int major = imajor(inode);
	/* find PCI device */
        for (i = 0; i < LITEPCIE__DEVICE_COUNT; i++) {
                if (!litepcie_device_table[i])
                    continue;
                s = litepcie_device_table[i];
                if ((major == MAJOR(s->cdev.dev))
                    && (minor >= MINOR(s->cdev.dev))
                    && (minor < MINOR(s->cdev.dev)+s->ch_cnt))
                        break;
                s = NULL;
        }

	if (!s)
		return -ENODEV;
#ifdef DEBUG_KERN
	printk("open %d %d\n", major, minor);
#endif
        file->private_data = &s->channels[minor-MINOR(s->cdev.dev)];
	litepcie_writel(s, CSR_CNTRL_TEST_ADDR, 55);
	if(litepcie_readl(s, CSR_CNTRL_TEST_ADDR) != 55){
		printk(KERN_ERR LITEPCIE_NAME " CSR register test failed\n");
		return -EIO;
	}
	return 0;
}

static int litepcie_dma_start(litepcie_channel* ch, int buf_size)
{
	int i;
	int ep = ch->dma_ep;
	litepcie_device *s = ch->dev;

	if (buf_size < MIN_TRANSFER_SIZE)
		buf_size = MIN_TRANSFER_SIZE;
	if (buf_size > DMA_BUFFER_SIZE)
		buf_size = DMA_BUFFER_SIZE;
	buf_size = (buf_size/MIN_TRANSFER_SIZE)*MIN_TRANSFER_SIZE;

	ch->buf_size = buf_size;
	ch->index = 0;
	ch->offset = 0;
#ifdef DEBUG_KERN
	printk("BUF SIZE: %d\n", buf_size);
#endif

	if (ch->is_tx) {
		/* init DMA read */
		litepcie_writel(s, CSR_PCIE_DMA_READER_ENABLE_ADDR(ep), 0);
		litepcie_writel(s, CSR_PCIE_DMA_READER_TABLE_FLUSH_ADDR(ep), 1);
		litepcie_writel(s,
			CSR_PCIE_DMA_READER_TABLE_LOOP_PROG_N_ADDR(ep), 0);

		for (i = 0; i < DMA_BUFFER_COUNT; i++)
		{
			litepcie_writel(s,
				CSR_PCIE_DMA_READER_TABLE_VALUE_ADDR(ep), buf_size);
			litepcie_writel(s,
				CSR_PCIE_DMA_READER_TABLE_VALUE_ADDR(ep) + 4,
				ch->dma_bufs_addr[i]);
			litepcie_writel(s,
				CSR_PCIE_DMA_READER_TABLE_WE_ADDR(ep), 1);
		}
		litepcie_writel(s,
			CSR_PCIE_DMA_READER_TABLE_LOOP_PROG_N_ADDR(ep), 1);
#ifdef DEBUG_KERN
		printk(KERN_INFO LITEPCIE_NAME " Starting DMA(%d) TX\n", ep);
#endif
		litepcie_writel(s, CSR_PCIE_DMA_READER_ENABLE_ADDR(ep), 1);
	} else {
		/* init DMA write */
		litepcie_writel(s, CSR_CNTRL_TEST_ADDR, 1);
		litepcie_writel(s, CSR_PCIE_DMA_WRITER_ENABLE_ADDR(ep), 0);
		litepcie_writel(s, CSR_PCIE_DMA_WRITER_TABLE_FLUSH_ADDR(ep), 1);
		litepcie_writel(s,
			CSR_PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_ADDR(ep), 0);

		for (i = 0; i < DMA_BUFFER_COUNT; i++)
		{
			litepcie_writel(s,
				CSR_PCIE_DMA_WRITER_TABLE_VALUE_ADDR(ep),
				buf_size);
			litepcie_writel(s,
				CSR_PCIE_DMA_WRITER_TABLE_VALUE_ADDR(ep) + 4,
				ch->dma_bufs_addr[i]);
			litepcie_writel(s,
				CSR_PCIE_DMA_WRITER_TABLE_WE_ADDR(ep), 1);
		}
		litepcie_writel(s,
			CSR_PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_ADDR(ep), 1);
		litepcie_writel(s,CSR_PCIE_DMA_WRITER_ALLOW_REQUESTS_ADDR(ep),1);
#ifdef DEBUG_KERN
		printk(KERN_INFO LITEPCIE_NAME " Starting DMA(%d) RX\n", ep);
#endif
		litepcie_writel(s, CSR_PCIE_DMA_WRITER_ENABLE_ADDR(ep), 1);
	}
	ch->started = 1;
	return 0;
}

static int litepcie_dma_stop(litepcie_channel* ch)
{
	litepcie_device *s = ch->dev;
	int ep = ch->dma_ep;
	if (!ch->is_tx) {
#ifdef DEBUG_KERN
		printk(KERN_INFO LITEPCIE_NAME " Stop RX DMA(%d)\n", ep);
		printk(KERN_INFO "RX(%d) End_pckt: %d\n",ep, litepcie_readl(s, CSR_PCIE_DMA_WRITER_TABLE_LOOP_STATUS_ADDR(ep)));
#endif
		litepcie_writel(s,CSR_PCIE_DMA_WRITER_ALLOW_REQUESTS_ADDR(ep),0);
		litepcie_writel(s,
			CSR_PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_ADDR(ep),	0);
		litepcie_writel(s, CSR_PCIE_DMA_WRITER_TABLE_FLUSH_ADDR(ep), 1);
		udelay(100);
		litepcie_writel(s, CSR_PCIE_DMA_WRITER_ENABLE_ADDR(ep), 0);
	} else 	{
#ifdef DEBUG_KERN
		printk(KERN_INFO LITEPCIE_NAME " Stop TX DMA(%d)\n", ep);
		printk(KERN_INFO "TX(%d) End_pckt: %d\n",ep,litepcie_readl(s, CSR_PCIE_DMA_READER_TABLE_LOOP_STATUS_ADDR(ep)));
		printk(KERN_INFO "TX(%d) Flush_nbytes: %d\n",ep,litepcie_readl(s, CSR_PCIE_DMA_READER_FLUSH_NBYTES_ADDR(ep)));
#endif
		udelay(100);
		litepcie_writel(s,
			CSR_PCIE_DMA_READER_TABLE_LOOP_PROG_N_ADDR(ep), 0);
		litepcie_writel(s, CSR_PCIE_DMA_READER_TABLE_FLUSH_ADDR(ep), 1);
		udelay(100);
		litepcie_writel(s, CSR_PCIE_DMA_READER_PACKET_NR_ADDR(ep), 0);
		litepcie_writel(s, CSR_PCIE_DMA_READER_FLUSH_NR_ADDR(ep), 0);
		litepcie_writel(s, CSR_PCIE_DMA_READER_FLUSH_NBYTES_ADDR(ep), 0);
		litepcie_writel(s, CSR_PCIE_DMA_READER_ENABLE_ADDR(ep), 0);
	}
	ch->started = 0;
	return 0;
}

static int litepcie_release(struct inode *inode, struct file *file)
{
	litepcie_channel *ch = (litepcie_channel*)file->private_data;
	if (ch && ch->started)
		litepcie_dma_stop(ch);
	return 0;
}

static long litepcie_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    litepcie_channel *ch = (litepcie_channel*)file->private_data;
    litepcie_device *s = ch->dev;
    long ret;

    switch (cmd)
    {
    case LITEPCIE_IOCTL_REG_READ:
    {
        struct litepcie_ioctl_reg_rw m;
        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
        // Currently without protection can be RIP
        m.valRead = litepcie_readl(s,m.adress);
        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
        ret = 0;
    }
    break;
    case LITEPCIE_IOCTL_REG_WRITE:
    {
        struct litepcie_ioctl_reg_rw m;
        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
        // Currently without protection can be RIP
        litepcie_writel(s,m.adress,m.valWrite);
        m.valRead = litepcie_readl(s,m.adress);
        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
        ret = 0;
    }
    break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }
    return ret;
}


static inline int litepcie_fifo_read(litepcie_channel* fifo, char __user *buf, const int count)
{
    int n;
    uint8_t *rx_buf;
    int bytes_read = 0;
    int ep = fifo->dma_ep;
    litepcie_device *s = fifo->dev;
    const uint8_t back = litepcie_readl(s, CSR_PCIE_DMA_WRITER_TABLE_LOOP_STATUS_ADDR(ep));
    if ((back+DMA_BUFFER_COUNT-fifo->index)%DMA_BUFFER_COUNT > DMA_BUFFER_COUNT*2/3) //assume overflow
    {
        fifo->offset = 0;
        fifo->index = (back + DMA_BUFFER_COUNT*2/3)%DMA_BUFFER_COUNT;
    }

    while (count > bytes_read)
    {
        if (fifo->index == (uint8_t)(back-1) || fifo->index == back)
            return bytes_read;

        rx_buf = fifo->dma_buf[fifo->index]+fifo->offset;
        n = fifo->buf_size-fifo->offset;
        if (n > count-bytes_read)
        {
            fifo->offset += count-bytes_read;
	    copy_to_user(buf+bytes_read, rx_buf, count-bytes_read);
            return count;
        }
        fifo->offset = 0;
        fifo->index = (fifo->index+1)%DMA_BUFFER_COUNT;
		copy_to_user(buf+bytes_read, rx_buf, n);
        bytes_read += n;
    }
    return bytes_read;
}

static inline int litepcie_fifo_write(litepcie_channel* fifo, const char __user *buf, int count)
{
	uint8_t *tx_buf;
	litepcie_device *s = fifo->dev;
	int ep = fifo->dma_ep;
	int n, bytes_written = 0;
	int timeout;
	const uint16_t front = litepcie_readl(s, CSR_PCIE_DMA_READER_CURRENT_PACKET_ADDR(ep));
	
	while (count > bytes_written)
	{
		if (((front + DMA_BUFFER_COUNT) & 0xFFFF) == fifo->index)
			return bytes_written;
		tx_buf = fifo->dma_buf[(fifo->index&0xFF)]+fifo->offset;
		n = fifo->buf_size-fifo->offset;
		if (n > count-bytes_written)
		{
			fifo->offset += count-bytes_written;
			copy_from_user(tx_buf, buf+bytes_written, count-bytes_written);
			//flush
			//TODO: wait if previous flush has not completed yet
#ifdef DEBUG_KERN
			printk("Flush_nr: %d\n", fifo->index + 1);
			printk("Flush nbytes: %d\n", fifo->offset/8*8);
#endif
			timeout = 1000;
			while (--timeout && litepcie_readl(s, CSR_PCIE_DMA_READER_FLUSH_NBYTES_ADDR(ep)))
				usleep_range(20, 700);
			if (timeout == 0)
				return bytes_written;
			litepcie_writel(s, CSR_PCIE_DMA_READER_FLUSH_NR_ADDR(ep), ++fifo->index);
			litepcie_writel(s, CSR_PCIE_DMA_READER_FLUSH_NBYTES_ADDR(ep), fifo->offset/8*8);
			litepcie_writel(s, CSR_PCIE_DMA_READER_PACKET_NR_ADDR(ep), fifo->index);
			fifo->offset = 0;
			return count;
		}
		fifo->offset = 0;
		copy_from_user(tx_buf, buf+bytes_written, n);
		litepcie_writel(s, CSR_PCIE_DMA_READER_PACKET_NR_ADDR(ep), ++fifo->index);
		bytes_written += n;
	}
	return bytes_written;
}

static inline int litepcie_ctrl_write(litepcie_channel* ch, const char __user *userbuf, int count)
{
	uint32_t i, value;
	if (count >  CSR_CNTRL_CNTRL_SIZE*sizeof(uint32_t))
	count =  CSR_CNTRL_CNTRL_SIZE*sizeof(uint32_t);
	for (i = 0; i< count; i+=sizeof(value))
	{
		if (copy_from_user(&value, userbuf+i, sizeof(value)))
		    return -EFAULT;
		litepcie_writel(ch->dev, CSR_CNTRL_BASE+i, value);
	}
	return i;
}

static inline int litepcie_ctrl_read(litepcie_channel* ch, char __user *userbuf, int count)
{
	uint32_t i, value;
	if (count > CSR_CNTRL_CNTRL_SIZE*sizeof(uint32_t))
	count =  CSR_CNTRL_CNTRL_SIZE*sizeof(uint32_t);
	for (i = 0; i< count; i+=sizeof(value))
	{
		value = litepcie_readl(ch->dev, CSR_CNTRL_BASE+i);
		if (copy_to_user(userbuf+i, &value, sizeof(value)))
			return -EFAULT;
	}
	return count;
}

static ssize_t litepcie_read(struct file *f, char __user *buf, size_t cnt, loff_t *p)
{
	litepcie_channel* ch = (litepcie_channel*)f->private_data;

	if (ch->dma_buf[0]) {
		if (!ch->started)
			litepcie_dma_start(ch, cnt);
		return litepcie_fifo_read(ch, buf, cnt);
	}
	return litepcie_ctrl_read(ch, buf, cnt);
}

static ssize_t litepcie_write(struct file *f, const char __user *buf, size_t cnt, loff_t *p)
{
	litepcie_channel* ch = (litepcie_channel*)f->private_data;

	if (ch->dma_buf[0]) {
		if (!ch->started)
			litepcie_dma_start(ch, cnt);
		return litepcie_fifo_write(ch, buf, cnt);
	}
	return litepcie_ctrl_write(ch, buf, cnt);
}

static const struct file_operations litepcie_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = litepcie_ioctl,
	.open = litepcie_open,
	.release = litepcie_release,
	.read = litepcie_read,
	.write = litepcie_write,
	.llseek = no_llseek,
};

static void litepcie_end(struct pci_dev *dev, litepcie_device *s)
{
	int i, j;
	enum dma_data_direction dir;
#ifdef DEBUG_KERN
	printk(KERN_INFO LITEPCIE_NAME " litepcie_end\n");
#endif
	
	for (i = 0; i < s->ch_cnt; i++) {
		dir = s->channels[i].is_tx ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
		for (j = 0; j < DMA_BUFFER_COUNT; j++) {
			if (s->channels[i].dma_bufs_addr[j])
				dma_unmap_single(&dev->dev,
						s->channels[i].dma_bufs_addr[j],
					        DMA_BUFFER_SIZE, dir);
			if (s->channels[i].dma_buf[j])
				kfree(s->channels[i].dma_buf[j]);
		}
	}
	if (s->channels)
		kfree(s->channels);
}

static void litepcie_pci_remove(struct pci_dev *dev)
{
    litepcie_device *s = pci_get_drvdata(dev);
    int i, major;
#ifdef DEBUG_KERN
	printk(KERN_INFO LITEPCIE_NAME " litepcie_pci_remove\n");
#endif
        for (i = 0; i < LITEPCIE__DEVICE_COUNT; i++) {
            if (litepcie_device_table[i] == s) {
                litepcie_device_table[i] = NULL;
                break;
            }
        }

    litepcie_end(dev, s);
    major = MAJOR(s->cdev.dev);
    for (i = 0; i  < 1+s->ch_cnt; i++)
	device_destroy(litepcie_class, MKDEV(major, i));
    cdev_del(&s->cdev);
    unregister_chrdev_region(s->cdev.dev, 1+s->ch_cnt);
    pci_iounmap(dev, s->bar0_addr);
    pci_disable_device(dev);
    pci_release_regions(dev);
    kfree(s);
};

static int litepcie_init_chrdev(litepcie_device *s, int index)
{
	int ret;
	int ch = 0, i, minor, major;
	char name[32];
    char litepcie_name[32];
	dev_t cdev;
	struct device *dev;
	if(s->dev->vendor == XILINX_FPGA_VENDOR_ID)
		snprintf(litepcie_name, sizeof(litepcie_name)-1, "%s%d", "Lime5GRadio", index);
	else if(s->dev->vendor == ALTERA_FPGA_VENDOR_ID && s->ch_cnt == 7)
		snprintf(litepcie_name, sizeof(litepcie_name)-1, "%s%d", "LimeQPCIe", index);
	else if(s->dev->vendor == ALTERA_FPGA_VENDOR_ID && s->ch_cnt == 3)
		snprintf(litepcie_name, sizeof(litepcie_name)-1, "%s%d", "LimePCIe", index);	

	ret = alloc_chrdev_region(&cdev, 0, 1+s->ch_cnt, LITEPCIE_NAME);
	if (ret)
	{
	    printk(KERN_ERR LITEPCIE_NAME " Could not allocate char device\n");
	    return ret;;
	}

	major = MAJOR(cdev);
	minor = MINOR(cdev);
	cdev_init(&s->cdev, &litepcie_fops);
	ret = cdev_add(&s->cdev, MKDEV(major, minor), 1+s->ch_cnt);
	if (ret) {
		printk(KERN_INFO LITEPCIE_NAME " Could not allocate char device\n");
		goto unreg;
	}

	dev= device_create(litepcie_class, NULL, MKDEV(major, minor), NULL,"%s_control", litepcie_name);
	if (IS_ERR(dev)) {
		printk(KERN_ERR LITEPCIE_NAME " Failed to create device.\n");
		ret = -ENODEV;
		goto destroy_ctrl;
	}

	for (i = minor+1, ch = 1; ch < s->ch_cnt; ch++, i++) {

		if (s->channels[ch].is_tx)
			snprintf(name, sizeof(name)-1, "%s_write%d", litepcie_name, s->channels[ch].dma_ep);
		else
			snprintf(name, sizeof(name)-1, "%s_read%d", litepcie_name, s->channels[ch].dma_ep);

		name[sizeof(name)-1] = 0;
		dev = device_create(litepcie_class,
				       NULL,
				       MKDEV(major, i),
				       NULL,
				       "%s", name);

		if (IS_ERR(dev)) {
			printk(KERN_ERR LITEPCIE_NAME
				 "Failed to create %s device\n",
				 name);
			ret = -ENODEV;
			goto destroy_channels;
		}
	}

	return 0; //success

destroy_channels:
	while (--ch)
		device_destroy(litepcie_class, MKDEV(major, --i));
destroy_ctrl:
	device_destroy(litepcie_class, MKDEV(major, minor));
	cdev_del(&s->cdev);
unreg:
	unregister_chrdev_region(cdev, 1+s->ch_cnt);
	return ret;
}

static const struct pci_device_id litepcie_pci_ids[] = {
	{PCI_DEVICE(XILINX_FPGA_VENDOR_ID, XILINX_FPGA_DEVICE_ID)},
    {PCI_DEVICE(ALTERA_FPGA_VENDOR_ID, ALTERA_FPGA_DEVICE_ID)},
    {}
};
MODULE_DEVICE_TABLE(pci, litepcie_pci_ids);

static struct pci_driver litepcie_pci_driver = {
    .name = LITEPCIE_NAME,
    .id_table = litepcie_pci_ids,
    .probe = litepcie_pci_probe,
    .remove = litepcie_pci_remove,
};

static int litepcie_add_channel(litepcie_device *s, uint8_t is_tx, uint8_t ep)
{
	int j;
	enum dma_data_direction dir = is_tx ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
	for (j = 0; j < DMA_BUFFER_COUNT; j++) {
            s->channels[s->ch_cnt].dma_buf[j] = kzalloc(DMA_BUFFER_SIZE,
							GFP_KERNEL | GFP_DMA32);
            if (! s->channels[s->ch_cnt].dma_buf[j])
		    return -ENOMEM;
            s->channels[s->ch_cnt].dma_bufs_addr[j] = pci_map_single(s->dev,
					s->channels[s->ch_cnt].dma_buf[j],
					DMA_BUFFER_SIZE,
					dir);
            if (!s->channels[s->ch_cnt].dma_bufs_addr[j])
		    return -ENOMEM;
        }
	s->channels[s->ch_cnt].dma_ep = ep;
	s->channels[s->ch_cnt].is_tx = is_tx;
        s->channels[s->ch_cnt].dev = s;
	s->ch_cnt++;
	return 0;
}

static int litepcie_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	litepcie_device *s = NULL;
	uint8_t rev_id, ch_cnt;
	int ret, index, i;
#ifdef DEBUG_KERN
	printk(KERN_INFO LITEPCIE_NAME " Probing device\n");
#endif

	for (index = 0; index < LITEPCIE__DEVICE_COUNT; index++)
		if (!litepcie_device_table[index])
			break;

	if (index == LITEPCIE__DEVICE_COUNT)
	{
		printk(KERN_ERR LITEPCIE_NAME " Cannot allocate a device\n");
		return -ENODEV;
	}

	s = kzalloc(sizeof(litepcie_device), GFP_KERNEL);
	if (!s)
	{
		printk(KERN_ERR LITEPCIE_NAME " Cannot allocate memory\n");
		ret = -ENOMEM;
		goto destroy_dev_state;
	}

	s->dev = dev;
	pci_set_drvdata(dev, s);

	ret = pci_enable_device(dev);
	if (ret != 0)
	{
		printk(KERN_ERR LITEPCIE_NAME " Cannot enable device\n");
		goto destroy_dev_state;
	}

	/* check device version */
	pci_read_config_byte(dev, PCI_REVISION_ID, &rev_id);
	if (rev_id != 1)
	{
		printk(KERN_ERR LITEPCIE_NAME " Unsupported device version %d\n", rev_id);
		goto disable_pcie_dev;
	}

	if (pci_request_regions(dev, LITEPCIE_NAME) < 0)
	{
		printk(KERN_ERR LITEPCIE_NAME " Could not request regions\n");
		goto disable_pcie_dev;
	}

	pci_disable_link_state(dev, PCIE_LINK_STATE_L0S);

	/* check BAR0 config */
	if (!(pci_resource_flags(dev, 0) & IORESOURCE_MEM))
	{
		printk(KERN_ERR LITEPCIE_NAME " Invalid BAR0 config\n");
		goto release_regions;
	}

	s->bar0_addr = pci_ioremap_bar(dev, 0);
	if (!s->bar0_addr)
	{
		printk(KERN_ERR LITEPCIE_NAME " Could not map BAR0\n");
		goto release_regions;
	}

	pci_set_master(dev);
	ret = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
	if (ret)
	{
		printk(KERN_ERR LITEPCIE_NAME " Failed to set DMA mask\n");
		goto unmap_io;
	};

	ch_cnt = litepcie_readl(s,CSR_CNTRL_NDMA_ADDR);
	if(ch_cnt < 0 || ch_cnt > DMA_ENDPOINT_COUNT)
	{
		printk(KERN_ERR LITEPCIE_NAME " Failed to get number of channels\n");
		goto unmap_io;
	}
#ifdef DEBUG_KERN
	printk(KERN_INFO LITEPCIE_NAME " Nchannels: %d\n",ch_cnt);
#endif
	/* allocate DMA buffers */
	s->channels = kzalloc(sizeof(litepcie_channel)*(1+ch_cnt*2), GFP_KERNEL);
	if (!s->channels)
		goto unmap_io;
        s->channels[0].dev = s;
        s->ch_cnt++;
	for (i = 0; i < ch_cnt; i++)
		if (litepcie_add_channel(s, 1, i))
			goto destory_channel_buffers;
	for (i = 0; i < ch_cnt; i++)
		if (litepcie_add_channel(s, 0, i))
			goto destory_channel_buffers;

	litepcie_device_table[index] = s;
#ifdef DEBUG_KERN
	printk(KERN_INFO LITEPCIE_NAME " Assigned to %d\n", index);
#endif
	ret = litepcie_init_chrdev(s, index);
	if (ret)
	    pci_unregister_driver(&litepcie_pci_driver);
	else
	    return 0;

destory_channel_buffers:
	litepcie_end(dev, s);
unmap_io:
	pci_iounmap(dev, s->bar0_addr);
release_regions:
	pci_release_regions(dev);
disable_pcie_dev:
	pci_disable_device(dev);
destroy_dev_state:
	kfree(s);
	printk(KERN_ERR LITEPCIE_NAME " Error while probing device\n");
    return ret;
}

static int __init litepcie_module_init(void)
{
    int ret;

    litepcie_class = class_create(THIS_MODULE, LITEPCIE_NAME);
    if (IS_ERR(litepcie_class))
        return PTR_ERR(litepcie_class);
#ifdef DEBUG_KERN
	printk(KERN_INFO LITEPCIE_NAME " Init litepcie module\n");
#endif
    
    ret = pci_register_driver(&litepcie_pci_driver);
    if (ret < 0)
    {
        printk(KERN_ERR LITEPCIE_NAME " Error while registering PCI driver\n");
        return ret;
    }

    return 0;
}

static void __exit litepcie_module_exit(void)
{
    pci_unregister_driver(&litepcie_pci_driver);
#ifdef DEBUG_KERN
	printk(KERN_INFO LITEPCIE_NAME " litepcie_module_exit\n");
#endif
    class_destroy(litepcie_class);
}

module_init(litepcie_module_init);
module_exit(litepcie_module_exit);

MODULE_LICENSE("GPL");
