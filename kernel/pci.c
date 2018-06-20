#define PCI_CONFIG_ADDR 0xCF8
#define PCI_CONFIG_DATA 0xCFC
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned int DWORD;
/* PCI设备索引。bus/dev/func 共16位，为了方便处理可放在一个WORD中 */
#define PDI_BUS_SHIFT   8
#define PDI_BUS_SIZE    8
#define PDI_BUS_MAX     0xFF
#define PDI_BUS_MASK    0xFF00

#define PDI_DEVICE_SHIFT   3
#define PDI_DEVICE_SIZE    5
#define PDI_DEVICE_MAX     0x1F
#define PDI_DEVICE_MASK    0x00F8

#define PDI_FUNCTION_SHIFT   0
#define PDI_FUNCTION_SIZE    3
#define PDI_FUNCTION_MAX     0x7
#define PDI_FUNCTION_MASK    0x0007

#define MK_PDI(bus,dev,func) (WORD)((bus&PDI_BUS_MAX)<<PDI_BUS_SHIFT | (dev&PDI_DEVICE_MAX)<<PDI_DEVICE_SHIFT | (func&PDI_FUNCTION_MAX) )
#define MK_PCICFGADDR(bus,dev,func) (DWORD)(0x80000000L | (DWORD)MK_PDI(bus,dev,func)<<8)

struct pci_config_addr{
	int always:2 ;		/* 0 ~ 1 */
	unsigned reg: 6;	/* 2 ~ 7 */
	union{
		struct{
			unsigned func: 3;	/* 8 ~ 10 */
			unsigned dev: 5;	/* 11 ~ 15 */
			unsigned bus: 8;	/* 16 ~ 23 */
		};
		u16 value;
	};
	unsigned reserved:	7;
	int enabled:	1;
};
unsigned get_pci_cfg_reg(int bus, int dev, int func, int reg){//reg后两为必须为0,即为4的倍数
	unsigned addr = (MK_PCICFGADDR(bus, dev, func))|(reg&0xfffffffc);
	unsigned port0xcfc = 0;
	__asm__ __volatile__ ("out %%eax, %%dx\n\t"
						  "mov $0xCFC, %%dx\n\t"
						  "in %%dx, %%eax\n\t"
						  :"=a"(port0xcfc)
						  :"a"(addr), "d"(PCI_CONFIG_ADDR)
						  );	
	return port0xcfc;	
}

void set_pci_cfg_reg(int bus, int dev, int func, int reg, unsigned value){
	unsigned addr = (MK_PCICFGADDR(bus, dev, func))|(reg&0xfffffffc);
	__asm__ __volatile__ ("out %%eax, %%dx\n\t"
						  "mov $0xCFC, %%dx\n\t"
						  "mov %%ebx, %%eax\n\t"
						  "out %%eax, %%dx\n\t"
						  :
						  :"a"(addr), "d"(PCI_CONFIG_ADDR), "b"(value)
						  );	
}


/*
汇编语言中，CPU对外设的操作通过专门的端口读写指令来完成；
　　读端口用IN指令，写端口用OUT指令。
　　例子如下：
　　IN AL,21H；表示从21H端口读取一字节数据到AL
　　IN AX,21H；表示从端口地址21H读取1字节数据到AL，从端口地址22H读取1字节到AH
　　MOV DX,379H
　　IN AL,DX ；从端口379H读取1字节到AL
　　OUT 21H,AL；将AL的值写入21H端口
　　OUT 21H,AX；将AX的值写入端口地址21H开始的连续两个字节。（port[21H]=AL,port[22h]=AH）
　　MOV DX,378H
　　OUT DX,AX ；将AH和AL分别写入端口379H和378H
*/
u8 readportb(unsigned short port){
	u8 value = 0;
	__asm__ __volatile__ ("in %%dx, %%al\n\t"
						  :"=a"(value)
						  :"d"(port)
						  );
	/*print("readport:",9);
	printInt(port,16);
	print("\n",1);*/
	return value;	
}

