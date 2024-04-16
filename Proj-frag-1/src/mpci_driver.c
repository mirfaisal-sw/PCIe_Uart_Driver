
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/version.h>

#include "mpci_driver.h"

#define RTL8111C_VENDOR_ID 0x10ec
#define RTL8111C_DEVICE_ID 0x8168

static ssize_t pcie_sysfs_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
        int ret = 0;

        ret += snprintf(buf + ret, PAGE_SIZE - ret, ">>>> PCIe Test <<<<\n");
        ret += snprintf(buf + ret, PAGE_SIZE - ret, "0 : PCIe Unit Test\n");
        ret += snprintf(buf + ret, PAGE_SIZE - ret, "1 : Link Test\n");
        ret += snprintf(buf + ret, PAGE_SIZE - ret, "2 : DisLink Test\n");
        ret += snprintf(buf + ret, PAGE_SIZE - ret,
                                "3 : Runtime L1.2 En/Disable Test\n");
        ret += snprintf(buf + ret, PAGE_SIZE - ret,
                                "4 : Stop Runtime L1.2 En/Disable Test\n");
        ret += snprintf(buf + ret, PAGE_SIZE - ret,
                                "5 : SysMMU Enable\n");
        ret += snprintf(buf + ret, PAGE_SIZE - ret,
                                "6 : SysMMU Disable\n");

        return ret;
}


static ssize_t pcie_sysfs_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{

#if 0
        int op_num, domain_nr;
        struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);
        u64 iommu_addr, iommu_size;
        int wifi_reg_on = of_get_named_gpio(dev->of_node,
                "gpio_wifi_reg_on", 0);
        struct pci_bus *ep_pci_bus;
        struct dw_pcie *pci = exynos_pcie->pci;
        struct pcie_port *pp = &pci->pp;
        u32 val;

		....
#endif 
        return count;
}

static DEVICE_ATTR_RW(pcie_sysfs);

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_rc_internal}
 * @purpose "Create PCIe sys file"
 * @logic "Create PCIe RC device sys file"
 * @params
 * @param{in/out, dev, struct ::device *, not NULL}
 * @endparam
 * @retval{-, int, 0, >=-EINVAL, not 0}
 */
static inline int rtl_create_pcie_sys_file(struct device *dev)
{
        return device_create_file(dev, &dev_attr_pcie_sysfs);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_rc_internal}
 * @purpose "Remove PCIe sys file"
 * @logic "Remove PCIe sysfs file"
 * @params
 * @param{in/out, dev, struct ::device *, not NULL}
 * @endparam
 * @noret
 */
static inline void rtl_remove_pcie_sys_file(struct device *dev)
{
        device_remove_file(dev, &dev_attr_pcie_sysfs);
}

static int rtl_read_and_print_config_space(struct pci_dev *dev)
{
	u16 vid, did;
	u8 capability_ptr;
	u32 bar0, saved_bar0;
	int region;

	pr_alert("RTL8111C: In function - %s\n", __func__);

	/* Let's read the PCIe VID and DID */
	if(0 != pci_read_config_word(dev, 0x0, &vid)) {
		printk("RTL8111C: Error reading from config space\n");
		return -1;
	}
	
	if(0 != pci_read_config_word(dev, 0x2, &did)) {
		printk("RTL8111C: Error reading from config space\n");
		return -1;
	}
	
	pr_alert("RTL8111C: Device ID: 0x%x : 0x%x\n", vid, did);
	
	/* Read the pci capability pointer */	
	if(0 != pci_read_config_byte(dev, 0x34, &capability_ptr)) {
		printk("RTL8111C: Error reading from config space\n");
		return -1;
	}
	if(capability_ptr) 
		pr_alert("RTL8111C: PCI card has capabilities!\n");
	else
		pr_alert("RTL8111C: PCI card doesn't have capabilities!\n");

	/* Determine BAR's fields and its size
	 * To determine the amount of address space needed by a PCI 
	 * device, we must save the original value of the BAR, write a 
	 * value of all 1's to the register, then read it back. The amount 
	 * of memory can then be determined by masking the information bits,
         * performing a bitwise NOT ('~' in C), and incrementing the value by 1.
	 * Then restore the original value of BAR.
	 * Note - See section - 6.2.5.1 in PCI specification Rev 3.0.
	 */
	if(0 != pci_read_config_dword(dev, 0x10, &bar0)) {
		pr_alert("RTL8111C: Error reading from config space\n");
		return -1;
	}

	saved_bar0 = bar0;

	/*I. Write all 1s to the BAR register*/
	if(0 != pci_write_config_dword(dev, 0x10, 0xffffffff)) {
		pr_alert("RTL8111C: Error writing to config space\n");
		return -1;
	}

	/*II. Read back*/
	if(0 != pci_read_config_dword(dev, 0x10, &bar0)) {
		pr_alert("RTL8111C: Error reading from config space\n");
		return -1;
	}

	/*III. Read BAR information, such as it holds I/O space or 
	 * memory space*/
	/* Read BAR info using lower 4 bits*/ 
	if((bar0 & 0x1) == 1) 
		pr_alert("RTL8111C: BAR0 is IO space\n");
	else
		pr_alert("RTL8111C: BAR0 is memory space\n");
	
	/*Decode Bits[2:1]*/
	if((bar0 & 0x06) == 0x02)
			pr_alert("RTL8111C: BAR0 is 16 bits wide\n");
	else if((bar0 & 0x06) == 0x00)
			pr_alert("RTL8111C: BAR0 is 32 bits wide\n");
	else if((bar0 & 0x06) == 0x04)
			pr_alert("RTL8111C: BAR0 is 64 bits wide\n");
	else {
		
	}
	
	/*Decode Bit 3*/
	if(bar0 & 0x08)
		pr_alert("RTL8111C: Pre-fetchable memory\n");
	else
		pr_alert("RTL8111C: Non-Pre-fetchable memory\n");
	
	/* IV. Read BAR size as below.*/
	/*If I/O Space*/
	if(bar0 & 0x1) {
		/* Clear bit 0 for I/O BAR, as they are reserved bits*/
		bar0 &= 0xFFFFFFFE;
		bar0 = ~bar0;
		bar0++;

	} else { /*If Memory space*/
		/* Clear bit 0 -3 memory BAR, as they are reserved bits*/
		bar0 &= 0xFFFFFFF0;
		bar0 = ~bar0;
		bar0++;
	}

	pr_alert("RTL8111C: Size of BAR0 - %d bytes\n", bar0);

	/*V. Restore the original value of BAR*/
	if(0 != pci_write_config_dword(dev, 0x10, saved_bar0)) {
		pr_alert("RTL8111C: Error writing to config space\n");
		return -1;
	}

	return 0;
}

void rtl_check_bar_resource(struct pci_dev *dev)
{
	int bar; /*It's value ranges from 0 to 5*/
	int i, size_of_bar;
	
	/* Expose the PCI resources from this device as files */
	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		bar = i;
		
		size_of_bar = pci_resource_len(dev, bar);
		
		/* skip empty resources */
		if(!size_of_bar)
			continue;
		
		pr_alert("RTL8111C: Size of Bar - %d: %d bytes\n", bar, size_of_bar);

		pr_alert("RTL8111C: BAR - %d starts at address 0x%llx\n",
			bar, pci_resource_start(dev, bar));
			
		pr_alert("RTL8111C: BAR - %d ends at address 0x%llx\n", 
			bar, pci_resource_end(dev, bar));
		
		if (pci_resource_flags(dev, bar) & IORESOURCE_IO) {
		
			pr_alert("RTL8111C: Bar - %d has IO mapped addresses\n", bar);
		
		} else if (pci_resource_flags(dev, bar) & IORESOURCE_IO) {
		
			pr_alert("RTL8111C: Bar - %d has Memory mapped addresses\n", bar);
		}
	
		if (pci_resource_flags(dev, bar) & IORESOURCE_PREFETCH) {
		
			pr_alert("RTL8111C: Bar - %d is Pre-fetchable\n", bar);		
		}
	
		if (pci_resource_flags(dev, bar) & IORESOURCE_READONLY) {
	
			pr_alert("RTL8111C: Bar - %d is read only\n", bar);		
		}
	}
	
}

struct rtl8111_private *rtl8111_device_init(struct pci_dev *pdev, struct device *dev)
{
	struct rtl8111_private *ep;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return NULL;

	ep->pdev = pdev;
	//ep->ndev = dev;

	mutex_init(&ep->register_mutex);
	return ep;
}

/** Finds out BARs available from BAR mask and choose
 *  first, second or third bar using 2nd argument.
 *  @bar_mask: BAR mask with bit set to 1 shows BAR number
 *  @bar_index_to_choose: 1 means first bar and so on
 */
int find_bars_from_bitmask(int bar_mask, int bar_index_to_choose)
{
        int i, bar_num, bar_index = 0;

        for(i = 0; i< PCI_NUM_RESOURCES; i++) {
                if((bar_mask >> i) & 0x1) {
                        bar_num = i;
                        bar_index++;

                        pr_alert("RTL8111C: BAR %d is available\n", bar_num);
                }

                if(bar_index == bar_index_to_choose)
                        break;
        }

        pr_alert("RTL8111C: Selcted BAR %d having BAR index - %d\n", bar_num, bar_index);
        return bar_num;
}

int rtl_get_mac_version(struct rtl8111_private *tp)
{
	u32 read_reg;
	u16 xid;

	read_reg = RTL_R32(tp, TxConfig);

	xid = (read_reg >> 20);
	pr_alert("RTL8111F: xid bits- %x\n", xid);

	if((xid & 0x7c8) == 0x2c8) {
		tp->mac_version = RTL_GIGA_MAC_VER_34; /*8168E family.*/
		pr_alert("RTL8111F: Chip MAC version - 34, 8168E family");

	} else {
		pr_alert("RTL8111F: Unknowd chip XID: %x\n", xid);
		return -ENODEV;
	}
	
	return 0;
}
										
/**
 * @brief Function is called, when a PCI device is registered
 *
 * @param dev   pointer to the PCI device
 * @param id    pointer to the corresponding id table's entry
 *
 * @return      0 on success
 *              negative error code on failure
 */
static int rtl8111c_probe(struct pci_dev *pdev, const struct pci_device_id *id) {
	
	void __iomem *ptr_bar;
	int ret, status;
	int region, bar_mask;
	struct rtl8111_private *ep;
	u16 xid;
	
	pr_alert("RTL8111F: In func - %s\n", __func__);
	ep = rtl8111_device_init(pdev, &pdev->dev);
    	if (!ep)
        	return -ENOMEM;

	ret = rtl_create_pcie_sys_file(&pdev->dev);
	if(ret == -1)
		pr_alert("RTL8111F: pcie sysfs create failed.\n");

	if(0 != rtl_read_and_print_config_space(pdev)) {
		pr_alert("RTL8111C: Config space read failed\n");
	}
	
	rtl_check_bar_resource(pdev);
	
	ret = pcim_enable_device(pdev);
	if(ret < 0) {
		pr_alert("RTL8111C: Could not enable device\n");
		return ret;
	}

	/*Get BAR mask from the type of MMIO region*/
	bar_mask = pci_select_bars(pdev, IORESOURCE_MEM);
	if (region < 0) { 
                dev_err(&pdev->dev, "no MMIO resource found\n");
                return -ENODEV;
        }
	pr_alert("RTL8111C: BAR mask - %x\n", bar_mask);

	/*Find available MMIO bars and choose first bar*/
	region = find_bars_from_bitmask(bar_mask, 2);

	ret = pcim_iomap_regions(pdev, BIT(region), "mpci_driver");
	if(ret < 0) {
		pr_alert("RTL8111C: pcim_iomap_regions() failed, ret - %d\n", ret);
		return ret;
	}
	
	ep->mmio_addr = pcim_iomap_table(pdev)[region];
	if(ep->mmio_addr == NULL) {
		pr_alert("RTL8111C: BAR pointer is invalid\n");
		return -1;
	}

	/*ptr_bar4 = pcim_iomap_table(pdev)[4];
        if(ptr_bar4 == NULL) {
                pr_alert("RTL8111C: BAR4 pointer is invalid\n");
                return -1;
        }*/

	rtl_get_mac_version(ep);

	ep->cp_cmd = RTL_R16(ep, CPlusCmd) & CPCMD_MASK;

	pci_set_drvdata(pdev, ep);

	/*Note : - Take reference of below: -
	1. char/xillybus/xillybus_pcie.c -> xilly_probe()
	2. https://github.com/Johannes4Linux/gpio_card_pcittl32io/blob/75682103d3821727cbb3b75629e2e4c3c16ff159/pcittl32io.c
	*/
	
	//printk("pcittl32io - GPIO State DWord 0x%x\n", ioread32(ptr_bar0 + PCITTL32IO_GPIO_STATE));
	//iowrite8(0x1, ptr_bar0 + PCITTL32IO_DIRECTION);
	
#if 0
	/*Enable DMA by setting the bus master bit in the PCI_COMMAND register.
          The device will then be able to act as a master on the address bus.*/
	pci_set_master(pdev);
	
	/* Set up a single MSI interrupt */
	if (pci_enable_msi(pdev)) {
			pr_alert("RTL8111C: Failed to enable MSI interrupts. Aborting.\n");
			return -ENODEV;
	}
	rc = devm_request_irq(&pdev->dev, pdev->irq, xillybus_isr, 0,
						  xillyname, ep);
	if (rc) {
			pr_alert("RTL8111C: Failed to register MSI handler. Aborting.\n");
			return -ENODEV;
	}

#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32))
	if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
		ep->dma_using_dac = 1;
	} else if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
		ep->dma_using_dac = 0;
	} else {
		pr_alert("RTL8111F: Failed to set DMA mask. Aborting.\n");
		return -ENODEV;
	}
#else
	if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(64))) {
		ep->dma_using_dac = 1;
	} else if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))) {
		ep->dma_using_dac = 0;
	} else {
		pr_alert("RTL8111F: Failed to set DMA mask. Aborting.\n");
		return -ENODEV;
	}
#endif
	pr_alert("RTL8111F: Func - %s success\n", __func__);
	return 0;
}

/**
 * @brief Function is called, when a PCI device is unregistered
 *
 * @param dev   pointer to the PCI device
 */
static void rtl8111c_remove(struct pci_dev *pdev) {
	
	pr_alert("RTL8111F: In func - %s\n", __func__);

	pci_disable_device(pdev);
	/*
	 * Release regions should be called after the disable. OK to
	 * call if request regions has not been called or failed.
	 */
	pci_release_regions(pdev);	

	rtl_remove_pcie_sys_file(&pdev->dev);
}

static struct pci_device_id rtl8111c_ids[] = {
	{ PCI_DEVICE(RTL8111C_VENDOR_ID, RTL8111C_DEVICE_ID) },
	{ }
};

MODULE_DEVICE_TABLE(pci, rtl8111c_ids);

/* PCI driver struct */
static struct pci_driver rtl8111c_driver = {
	.name = "rtl8111c",
	.id_table = rtl8111c_ids,
	.probe = rtl8111c_probe,
	.remove = rtl8111c_remove,
#if 0
	.suspend = rtl8111c_suspend,
	.resume = rtl8111c_resume,
#endif
};

/**
 * @brief This function is called, when the module is loaded into the kernel
 */
static int __init my_init(void) {
	pr_alert("RTL8111C: Registering the PCI device\n");
	return pci_register_driver(&rtl8111c_driver);
}

/**
 * @brief This function is called, when the module is removed from the kernel
 */
static void __exit my_exit(void) {
	pr_alert("RTL8111C: Unregistering the PCI device\n");
	pci_unregister_driver(&rtl8111c_driver);
}

module_init(my_init);
module_exit(my_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mir.Faisal@harman.com");
