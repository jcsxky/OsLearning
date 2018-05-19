#include "kernel.h"
void __list_add(list_head_t *new, list_head_t *prev,
												list_head_t *next){
		new->next = next;
		next->prev = new;
		new->prev = prev;
		prev->next = new;
}
void list_add(list_head_t *new, list_head_t *head){
	__list_add(new, head, head->next);
}

void list_add_tail(list_head_t *new, list_head_t *head){
	__list_add(new, head->prev, head);
}
struct pci_device_id *id_table_matched(struct pci_device_id *id_table, struct pci_dev *dev){
	struct pci_device_id * id = id_table;
	while(id->vendor){
		if(id->vendor == dev->vendor && id->device == dev->device) return id;
		id++;
	}
	return (void*)0;
}
int pci_register_driver(struct pci_driver *driver){
	list_add(&driver->node, &pcidrvs_root);	
	struct list_head *curr_node = pcidevs_root.next;
	//struct pci_dev *matched;
	while(curr_node != &pcidevs_root){
		struct pci_dev *curr_dev = MB2STRU(struct pci_dev, curr_node, node);
		struct pci_device_id * id = id_table_matched( driver->id_table, curr_dev);
		if(id){
			curr_dev->driver = driver;
			driver->probe( curr_dev, id);
		};
		curr_node = curr_node->next;
	}
	return 0;
}


void writeb(unsigned addr, u8 value){
	__asm__ __volatile__("mov %%al, (%%ebx)\n\t"
						 :
						 :"b"(addr), "a"(value));
}

void writew(unsigned addr, u16 value){
	__asm__ __volatile__("mov %%ax, (%%ebx)\n\t"
						 :
						 :"b"(addr), "a"(value));
}

void writel(unsigned addr, unsigned value){
	__asm__ __volatile__("mov %%eax, (%%ebx)\n\t"
						 :
						 :"b"(addr), "a"(value));
}

unsigned readb(unsigned addr){
	unsigned value;
	__asm__ __volatile__("xor %0, %0 \n\t"
						 "mov (%%ebx), %%al\n\t"
						 :"=r"(value)
						 :"b"(addr)
						 );
	return value;
}


unsigned readw(unsigned addr){
	unsigned value;
	__asm__ __volatile__("xor %0, %0 \n\t"
						 "mov (%%ebx), %%ax\n\t"
						 :"=r"(value)
						 :"b"(addr)
						 );
	return value;
}


unsigned readl(unsigned addr){
	unsigned value;
	__asm__ __volatile__("xor %0, %0 \n\t"
						 "mov (%%ebx), %%eax\n\t"
						 :"=r"(value)
						 :"b"(addr)
						 );
	return value;
}

void maskb(unsigned addr, u8 mask){
	unsigned value = readb(addr);
	value |= mask;
	writeb(addr, value);
}

void unmaskb(unsigned addr, u8 mask){
	unsigned value = readb(addr);
	value &= ~mask;
	writeb(addr, value);
}


void maskw(unsigned addr, u16 mask){
	unsigned value = readw(addr);
	value |= mask;
	writew(addr, value);
}

void unmaskw(unsigned addr, u16 mask){
	unsigned value = readw(addr);
	value &= ~mask;
	writew(addr, value);
}

void maskl(unsigned addr, unsigned mask){
	unsigned value = readl(addr);
	value |= mask;
	writel(addr, value);
}
void set_pci_cfg_reg(int bus, int dev, int func, int reg, unsigned value){
	unsigned addr = MK_PCI_CFG_ADDR(bus, dev, func, reg);
	__asm__ __volatile__ ("out %%eax, %%dx\n\t"
						  "mov $0xCFC, %%dx\n\t"
						  "mov %%ebx, %%eax\n\t"
						  "out %%eax, %%dx\n\t"
						  :
						  :"a"(addr), "d"(PCI_CONFIG_ADDR), "b"(value)
						  );	
}
void unmaskl(unsigned addr, unsigned mask){
	unsigned value = readl(addr);
	value &= ~mask;
	writel(addr, value);
}
unsigned pci_read_config_dword(struct pci_dev *pcidev, int offset){
	return 	get_pci_cfg_reg(pcidev->bus, pcidev->dev, pcidev->func, offset>>2);
}

void pci_write_config_dword(struct pci_dev *pcidev, int offset, unsigned value){
	set_pci_cfg_reg(pcidev->bus, pcidev->dev, pcidev->dev, offset>>2, value);
}
void pci_fix_config_dword(struct pci_dev *pcidev, int offset, unsigned value){
	unsigned origin = pci_read_config_dword(pcidev, offset);
	origin |= value;
	pci_write_config_dword(pcidev, offset, origin);
}

int pci_enable_device(struct pci_dev *pcidev){
	pci_fix_config_dword(pcidev, PCI_COMMAND, PCI_COMMAND_IO | PCI_COMMAND_MEMORY);
	return 0;
}

int pci_set_master(struct pci_dev *pcidev){
	pci_fix_config_dword(pcidev, PCI_COMMAND, PCI_COMMAND_MASTER);
	return 0;
}


