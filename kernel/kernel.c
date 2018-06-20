#include "kernel.h"
extern void register_ne2000_driver(void);
extern void list_add(list_head_t *new, list_head_t *head);
extern void print(void *src,unsigned int n);
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned int DWORD;
int pci_dev_index=0;
void main(){
	//INIT_LIST_HEAD(&pcidevs_root);
	//INIT_LIST_HEAD(&pcidrvs_root);
	int bus,dev,func,x;
	for(bus = 0; bus < PCI_BUS_MAX; bus++){
		for(dev = 0; dev < PCI_DEV_MAX; dev++){
			for(func = 0; func < PCI_FUNC_MAX; func++){
				unsigned port0xcfc = get_pci_cfg_reg(bus, dev, func, 0);
				u16 vendor = (u16)port0xcfc;
				//if(vendor == 0x8086) continue;
				unsigned port0xcfc2;
				if(vendor == 0xffff) {
					//list_add(0x0, &pcidevs_root);
					//port0xcfc = get_pci_cfg_reg(bus, dev, func, 0);
					//port0xcfc1 = get_pci_cfg_reg(bus, dev, func, 1);
					//break;
				}
				else{
					struct pci_dev *pcidev = &pcidevs[pci_dev_index++];
					int i;
					//printInt(port0xcfc,16);
					for(i = 0; i < 0x40; i+=4){
						if(bus==0 && dev==3 && func==0){
							switch(i){
								/*case 0x10:
									set_pci_cfg_reg(bus, dev, func, i,0xfeb90001);
									break;
								case 0x14:
									set_pci_cfg_reg(bus, dev, func, i,0xc001);
									break;*/
							}
						}
						x=get_pci_cfg_reg(bus, dev, func, i);
						((unsigned *)pcidev)[i/4] = x;
						//if(bus==0 && dev==3 && func==0){
							if(i>=0x10 && i<=0x24){
								print("ADDR:",5);
								printInt(x,16);
								//print("",3);
							}else{
								print("ELSE:",5);
								printInt(x,16);
							}
							//printInt(x,16);
							/*if(i==0x10){
								set_pci_cfg_reg(bus,dev,func,i,0x00400001);
								((unsigned *)pcidev)[i/4] = 0x00400001;
							}
							else if(i==0x14){
								((unsigned *)pcidev)[i/4] = 0x00500000;
								set_pci_cfg_reg(bus,dev,func,i,0x00500000);
							}*/
						//}
					}
					print("\n",1);
					pcidev->bus = bus;
					pcidev->dev = dev;
					pcidev->func = func;
					//list_add(&pcidev->node, &pcidevs_root);
					//if(func == 0 && !(pcidev->headtype & 0x80))
					//	break;
				}
			}
		}
	}
	//while(1);
	//printInt(pci_dev_index,16);
	//printInt(21,10);
	//printInt(21,16);
	register_ne2000_driver();
}
