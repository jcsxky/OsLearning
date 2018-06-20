#include "kernel.h"
extern int pci_dev_index;
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
		if(id->vendor == dev->vendor && id->device == dev->device)
			return id;
		id++;
	}
	return (void*)0;
}
int pci_register_driver(struct pci_driver *driver){
	//list_add(&driver->node, &pcidrvs_root);	
	//struct list_head *curr_node = pcidevs_root.next;
	//struct pci_dev *matched;
	int i;
	for(i=0;i<pci_dev_index;i++){
		struct pci_dev *curr_dev = pcidevs+i;
		struct pci_device_id * id = id_table_matched( driver->id_table, curr_dev);
		if(id){
			curr_dev->driver = driver;
			if(driver->probe!=0){
				//print("qqqq\n",5);
				//printInt(curr_dev->address[0],16);
				//printInt(curr_dev->command,16);
				//while(1);
				driver->probe( curr_dev, id);
			}
		}
	}
	return 0;
}
void *
memmove(void *dst, const void *src, unsigned int n)
{
	const char *s;
	char *d;

	s = src;
	d = dst;
	if (s < d && s + n > d) {
		s += n;
		d += n;
		if ((int)s%4 == 0 && (int)d%4 == 0 && n%4 == 0)
			asm volatile("std; rep movsl\n"
				:: "D" (d-4), "S" (s-4), "c" (n/4) : "cc", "memory");
		else
			asm volatile("std; rep movsb\n"
				:: "D" (d-1), "S" (s-1), "c" (n) : "cc", "memory");
		// Some versions of GCC rely on DF being clear
		asm volatile("cld" ::: "cc");
	} else {
		if ((int)s%4 == 0 && (int)d%4 == 0 && n%4 == 0)
			asm volatile("cld; rep movsl\n"
				:: "D" (d), "S" (s), "c" (n/4) : "cc", "memory");
		else
			asm volatile("cld; rep movsb\n"
				:: "D" (d), "S" (s), "c" (n) : "cc", "memory");
	}
	return dst;
}
int
memcmp(const void *v1, const void *v2, unsigned int n)
{
	const u8 *s1 = (const u8 *) v1;
	const u8 *s2 = (const u8 *) v2;

	while (n-- > 0) {
		if (*s1 != *s2)
			return (int) *s1 - (int) *s2;
		s1++, s2++;
	}

	return 0;
}
void *
memset(void *v, int c, unsigned int n)
{
	char *p;
	int m;

	p = v;
	m = n;
	while (--m >= 0)
		*p++ = c;

	return v;
}
void *
memcpy(void *dst, const void *src)
{
	char* dst1=(char*)dst,* src1=(char*)src;
	while ((*dst1++ = *src1++) != '\0')
		/* do nothing */;
	return dst;
}
void writeb(unsigned short addr, u8 value){
	__asm__ __volatile__("out %%al, %%dx\n\t"
						 :
						 :"d"(addr), "a"(value));
}

void writew(unsigned short addr, u16 value){
	__asm__ __volatile__("out %%ax, %%dx\n\t"
						 :
						 :"d"(addr), "a"(value));
}

void writel(unsigned short addr, unsigned value){
	__asm__ __volatile__("out %%eax, %%dx\n\t"
						 :
						 :"d"(addr), "a"(value));
}

u8 readb(unsigned short port){
	u8 value;
	__asm__ __volatile__("in %%dx, %%al\n\t"
						 :"=r"(value)
						 :"d"(port)
						 );
	return value;
}


u16 readw(unsigned short addr){
	u16 value;
	__asm__ __volatile__(
						 "in %%dx, %%ax\n\t"
						 :"=r"(value)
						 :"d"(addr)
						 );
	return value;
}


u32 readl(unsigned short addr){
	u32 value;
	__asm__ __volatile__(
						 "in %%dx, %%eax\n\t"
						 :"=r"(value)
						 :"d"(addr)
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

void unmaskl(unsigned addr, unsigned mask){
	unsigned value = readl(addr);
	value &= ~mask;
	writel(addr, value);
}
unsigned pci_read_config_dword(struct pci_dev *pcidev, int offset){
	return 	get_pci_cfg_reg(pcidev->bus, pcidev->dev, pcidev->func, offset);
}

void pci_write_config_dword(struct pci_dev *pcidev, int offset, unsigned value){
	set_pci_cfg_reg(pcidev->bus, pcidev->dev, pcidev->func, offset, value);
}
void pci_fix_config_dword(struct pci_dev *pcidev, int offset, unsigned value){
	unsigned origin = pci_read_config_dword(pcidev, offset);
	origin |= value;
	pci_write_config_dword(pcidev, offset, origin);
}

int pci_enable_device(struct pci_dev *pcidev){
	pci_fix_config_dword(pcidev, PCI_COMMAND, PCI_COMMAND_IO/* | PCI_COMMAND_MEMORY*/);
	return 0;
}

int pci_set_master(struct pci_dev *pcidev){
	pci_fix_config_dword(pcidev, PCI_COMMAND, PCI_COMMAND_MASTER);
	return 0;
}


