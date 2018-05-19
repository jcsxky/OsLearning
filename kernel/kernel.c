#include "kernel.h"
void register_rtl8139_driver(void);
void main(){
	INIT_LIST_HEAD(&pcidevs_root);
	INIT_LIST_HEAD(&pcidrvs_root);
	int bus,dev,func;
	for(bus = 0; bus < PCI_BUS_MAX; bus++){
		for(dev = 0; dev < PCI_DEV_MAX; dev++){
			for(func = 0; func < PCI_FUNC_MAX; func++){
				unsigned port0xcfc = get_pci_cfg_reg(bus, dev, func, 0);
				u16 vendor = (u16)port0xcfc;
				//if(vendor == 0x8086) continue;
				if(vendor == 0xffff) break; 
				struct pci_dev *pcidev = pcidevs[bus][dev][func];
				int i;
				for(i = 0; i < 64; i++)
					((unsigned *)pcidev)[i] = get_pci_cfg_reg(bus, dev, func, i);
				pcidev->bus = bus;
				pcidev->dev = dev;
				pcidev->func = func;
				list_add(&pcidev->node, &pcidevs_root);
				if(func == 0 && !(pcidev->headtype & 0x80)) break;
			}
		}
	}

	register_rtl8139_driver();
}
