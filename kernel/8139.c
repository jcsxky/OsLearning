//1, TODO 4G DESC overflow bug
//2, 注意mmio时的gcc的寄存器缓冲和eliminate优化和硬件的cache缓冲。
//3, TODO 清除IMR的不需要的位（似乎需要的多）
//4, TODO 通过写bit，清除status register的一些位。
//5, TODO ACK.
//6, 怎么实现nic_wake_queue(),我没有软中断。拿就要在收包中断里调用发包函数，这样会引入cli-sti的复杂性，我不想在网卡驱动层引入irq_push/pop。那样容易ｇｅｔ_illcurrent(). 看来要实现soft-irq了。
//7, ack 8139(clear ISR bit x)
//8, EOI (ack 8259A)
#include "kernel.h"
#define PRIV(netdev) ((struct rtl8139_private*)(netdev->private))
#define __priv(netdev) ((struct rtl8139_private *)netdev->private)
#define ETH_MIN_LEN 60
#define NUM_TX_DESC 4
#define TX_BUF_SIZE (1762 & ~3) /* MTU of 8139 should be 1762, but 4 bytes 
								 * aligned is required for this chip */
//static struct net_device *testnd;
struct rtl8139_private{
	struct pci_dev *pcidev;
	void *mmio_addr;			/* memory mapped I/O addr */
	unsigned long regs_len;		/*length of I/O or MMI/O region */
	unsigned tx_flags;
	unsigned touse_desc;
	unsigned bsy_desc;
	unsigned char *tx_bufs[NUM_TX_DESC];		/* 跟linux正好相反*/
	unsigned char *tx_buf;
	unsigned char *tx_buf_dma;
	u16 txbuf_size;
	u8 *rx_ring;
	void *rx_ring_dma;
	int rx_ring_len;
	unsigned cursor_r;		/* points to the first byte behind the package we 
							 * read just now, in another word, it always points
							 * to the package we are going to read */
};

static inline void RTL_maskb(struct net_device *netdev, int offset, unsigned mask){
	maskb(netdev->base_addr + offset, mask);
}
static inline void RTL_maskw(struct net_device *netdev, int offset, unsigned mask){
	maskw(netdev->base_addr + offset, mask);
}
static inline void RTL_maskl(struct net_device *netdev, int offset, unsigned mask){
	maskl(netdev->base_addr + offset, mask);
}


static inline unsigned RTL_readb(struct net_device *netdev, int offset){
	return readb(netdev->base_addr + offset);
}
static inline unsigned RTL_readw(struct net_device *netdev, int offset){
	return readw(netdev->base_addr + offset);
}
static inline unsigned RTL_readl(struct net_device *netdev, int offset){
	return readl(netdev->base_addr + offset);
}



static inline void RTL_writeb(struct net_device *netdev, int offset, unsigned value){
	writeb(netdev->base_addr + offset, value);
}
static inline void RTL_writew(struct net_device *netdev, int offset, unsigned value){
	writew(netdev->base_addr + offset, value);
}
static inline void RTL_writel(struct net_device *netdev, int offset, unsigned value){
	writel(netdev->base_addr + offset, value);
}
#define MAKE_IP(a, b, c, d) (((a)<<24) + ((b)<<16) + ((c)<<8) + d)
static struct ipconfig{
	u8 mac[7];
	u32 ip;
	u32 gateway_ip;
	u32 ipmask;
} ipconfigs[] ={
	{
		{0x00, 0xe0, 0x4c, 0x4b, 0x79, 0x9e, 0},	//base addr: 0x3bafe000  高挡板网卡，连开发机
		MAKE_IP(192, 168, 0, 9),
		MAKE_IP(192, 168, 0, 1),
		~0xff
	},
	{
		{0x00, 0x19, 0xe0, 0x00, 0x4f, 0xaa, 0},	//base addr: 0x3baff000		低板网卡，连网关出口
		MAKE_IP(192, 168, 1, 9),
		MAKE_IP(192, 168, 1, 1),
		~0xff
	},
};

/* vendor 和 device就能唯一的指定设备类型*/
static struct pci_device_id rtl8139_id_tbl[] = {
	{0x10ec, 0x8139,  },
	{0x10ec, 0x8138,  },
	{0x1113, 0x1211,  },
	{0x1500, 0x1360,  },
	{0x4033, 0x1360,  },
	{0x1186, 0x1300,  },
	{0x1186, 0x1340,  },
	{0x13d1, 0xab06,  },
	{0x1259, 0xa117,  },
	{0x1259, 0xa11e,  },
	{0x14ea, 0xab06,  },
	{0x14ea, 0xab07,  },
	{0x11db, 0x1234,  },
	{0x1432, 0x9130,  },
	{0x02ac, 0x1012,  },
	{0x018a, 0x0106,  },
	{0x126c, 0x1211,  },
	{0x1743, 0x8139,  },
	{0x021b, 0x8139,  },
	{0, 0},
};


/* Symbolic offsets to registers. */
enum RTL8139_registers {
	MAC0		= 0,	 /* Ethernet hardware address. */
	MAR0		= 8,	 /* Multicast filter. */
	TxStatus0	= 0x10,	 /* Transmit status (Four 32bit registers). */
	TxAddr0		= 0x20,	 /* Tx descriptors (also four 32bit). */
	RBSTART		= 0x30,
	ChipCmd		= 0x37,
	CursorToRead	= 0x38,
	CursorToRecv	= 0x3A,
	IMR	= 0x3C,
	ISR	= 0x3E,
	TxConfig	= 0x40,
	RxConfig	= 0x44,
	Timer		= 0x48,	 /* A general-purpose counter. */
	RxMissed	= 0x4C,  /* 24 bits valid, write clears. */
	Cfg9346		= 0x50,
	Config0		= 0x51,
	Config1		= 0x52,
	TimerInt	= 0x54,
	MediaStatus	= 0x58,
	Config3		= 0x59,
	Config4		= 0x5A,	 /* absent on RTL-8139A */
	HltClk		= 0x5B,
	MultiIntr	= 0x5C,
	TxSummary	= 0x60,
	BasicModeCtrl	= 0x62,
	BasicModeStatus	= 0x64,
	NWayAdvert	= 0x66,
	NWayLPAR	= 0x68,
	NWayExpansion	= 0x6A,
	/* Undocumented registers, but required for proper operation. */
	FIFOTMS		= 0x70,	 /* FIFO Control and test. */
	CSCR		= 0x74,	 /* Chip Status and Configuration Register. */
	PARA78		= 0x78,
	FlashReg	= 0xD4,	/* Communication with Flash ROM, four bytes. */
	PARA7c		= 0x7c,	 /* Magic transceiver parameter register. */
	Config5		= 0xD8,	 /* absent on RTL-8139A */
};

enum ChipCmdBits {
	CmdReset	= 0x10,
	CmdRxEnb	= 0x08,
	CmdTxEnb	= 0x04,
	RxBufEmpty	= 0x01,
};


/* Interrupt register bits, using my own meaningful names. */
enum IntrStatusBits {
	PCIErr		= 0x8000,
	PCSTimeout	= 0x4000,
	CabLenChg = 0x2000,
	RxFIFOOver	= 0x40,
	RxUnderrun	= 0x20,
	RxOverflow	= 0x10,
	TxErr		= 0x08,
	TxOK		= 0x04,
	RxErr		= 0x02,
	RxOK		= 0x01,

	RxAckBits	= RxFIFOOver | RxOverflow | RxOK,
};

enum TxStatusBits {
	TxHostOwns	= 0x2000,
	TxUnderrun	= 0x4000,
	TxStatOK	= 0x8000,
	TxOutOfWindow	= 0x20000000,
	TxAborted	= 0x40000000,
	TxCarrierLost	= 0x80000000,
};
enum RxStatusBits {
	RxMulticast	= 0x8000,
	RxPhysical	= 0x4000,
	RxBroadcast	= 0x2000,
	RxBadSymbol	= 0x0020,
	RxRunt		= 0x0010,
	RxTooLong	= 0x0008,
	RxCRCErr	= 0x0004,
	RxBadAlign	= 0x0002,
	RxStatusOK	= 0x0001,
};

/* Bits in RxConfig. */
enum rx_mode_bits {
	AcceptErr	= 0x20,
	AcceptRunt	= 0x10,
	AcceptBroadcast	= 0x08,
	AcceptMulticast	= 0x04,
	AcceptMyPhys	= 0x02,
	AcceptAllPhys	= 0x01,
};

/* Bits in TxConfig. */
enum tx_config_bits {
        /* Interframe Gap Time. Only TxIFG96 doesn't violate IEEE 802.3 */
        TxIFGShift	= 24,
        TxIFG84		= (0 << TxIFGShift), /* 8.4us / 840ns (10 / 100Mbps) */
        TxIFG88		= (1 << TxIFGShift), /* 8.8us / 880ns (10 / 100Mbps) */
        TxIFG92		= (2 << TxIFGShift), /* 9.2us / 920ns (10 / 100Mbps) */
        TxIFG96		= (3 << TxIFGShift), /* 9.6us / 960ns (10 / 100Mbps) */

	TxLoopBack	= (1 << 18) | (1 << 17), /* enable loopback test mode */
	TxCRC		= (1 << 16),	/* DISABLE Tx pkt CRC append */
	TxClearAbt	= (1 << 0),	/* Clear abort (WO) */
	TxDMAShift	= 8, /* DMA burst value (0-7) is shifted X many bits */
	TxRetryShift	= 4, /* TXRR value (0-15) is shifted X many bits */

	TxVersionMask	= 0x7C800000, /* mask out version bits 30-26, 23 */
};

enum RxConfigBits {
	/* rx fifo threshold */
	RxCfgFIFOShift	= 13,
	RxCfgFIFONone	= (7 << RxCfgFIFOShift),

	/* Max DMA burst */
	RxCfgDMAShift	= 8,
	RxCfgDMAUnlimited = (7 << RxCfgDMAShift),

	/* rx ring buffer length */
	RxCfgRcv8K	= 0,
	RxCfgRcv16K	= (1 << 11),
	RxCfgRcv32K	= (1 << 12),
	RxCfgRcv64K	= (1 << 11) | (1 << 12),

	/* Disable packet wrap at end of Rx buffer. (not possible with 64k) */
	RxNoWrap	= (1 << 7),
};
static inline void rtl8139_reset(struct net_device *netdev){
	RTL_writeb(netdev, ChipCmd, CmdReset);
	int i;
	for(i = 0; i < 1000; i++){
		if((RTL_readb(netdev, ChipCmd) & CmdReset) == 0) return;
		//udelay(10);
	}
	//spin("8139 reset failed");
	return;
}



/* 不需要，4个tx_buf的长度不需要指定，　只是在发送数据时，用来指定本次含数据的长度。
static inline void RTL8139_config_txbuf_len(struct net_device *netdev, int bitfield){
	int size = bitfield == 0 ? 8 : 32*bitfield ;
	assert(size < 1792);
	netdev->txbuf_size;
}
*/
int rtl8139_stop(struct net_device *netdev){
	return 0;
}

#pragma pack(push)
#pragma pack(1)
struct TxStatusReg{
	union{
		unsigned value;
		struct {
			int size: 13;
			int OWN: 1;
			int TUN: 1;
			int TOK: 1;
			int threshold: 6;
			int reserved: 2;
			int ncc: 4;
			int CDH: 1;
			int OWC: 1;
			int TABT: 1;
			int CRS: 1;
		};
	};
};	

struct raw_package{
	u16 rx_status;
	u16 size;
	char data[0];
};
#pragma pack(pop)

void info_regs(struct net_device *netdev){
	u16 intr_status = RTL_readw(netdev, ISR);
	u32 tx_status = RTL_readl(netdev, TxStatus0);
}

static int tx_bottomhalf( void *_netdev){
	struct net_device *netdev = _netdev;
	//nic_wake_queue(netdev);
	//oprintf(" TX bottom half ");
	//mdelay(500);
	return 0;
}

/* 1, 一旦有“发包中断”，就扫描４个DESC,从而forward bsy_desc
 * 2, 必须要在发包中断里free sk_buff,因为这时才能确定其相应的数据已经送上网线了
 * 3, by default, the transmitter will re-transmit a number of 16 times before aborting
 *	  due to excessive collisions. If failed finally, bit TER in ISR will be set.
 */
static void on_tx(struct net_device *netdev){
	struct rtl8139_private *private = netdev->private;
    int frozen_desc = private->touse_desc + 4;
	//assert( private->bsy_desc < frozen_desc);

	struct TxStatusReg TSR;
	int vdesc = private->bsy_desc;
	while(private->bsy_desc != frozen_desc){
		int realdesc = private->bsy_desc % NUM_TX_DESC;
		TSR.value = RTL_readl(netdev, TxStatus0 + realdesc * 4);
		if(!TSR.TOK && !TSR.TUN && !TSR.TABT){
			if(vdesc == private->bsy_desc){
					netdev->debug.count_drop_tok++;
					return;
			}
			break;
		}
		if(TSR.TOK){
			//RTL_writew(netdev, ISR, TxOK);
		}
		else{
			;
		}

		private->bsy_desc ++;
	}

	/* all the four decriptors free again, we wake tx-queue later */
	if(private->bsy_desc - private->touse_desc == 4){
		//oprintf(" wake tx_queue in bh");
		//mdelay(2000);
		//mark_bh(netdev->on_tx_bh);
	}
}


extern void process_rx_queue( struct net_device *netdev);
static int rx_bottomhalf( void *_netdev){
	//process_rx_queue(_netdev);
	return 0;
}
static void on_rx(struct net_device *netdev){
	//RTL_writew(netdev, ISR, RxOK);
	
	/*oprintf(" !R! ");

	struct rtl8139_private *private = netdev->private;
	while(!( RTL_readb(netdev, ChipCmd) & RxBufEmpty )){
		struct raw_package *raw = (void *)(private->rx_ring + private->cursor_r);
		int data_size = raw->size - 4;
		struct sk_buff *skb = dev_alloc_skb(data_size );
		skb->dev = netdev;
		memcpy(skb->ethhdr, raw->data, data_size);
		LL2_A(&netdev->rx_queue, skb);
		mark_bh(netdev->on_rx_bh);

		private->cursor_r = (private->cursor_r + raw->size + 4 + 3) & ~3;
		if(private->cursor_r > private->rx_ring_len) 
					private->cursor_r %= private->rx_ring_len;
		RTL_writew(netdev, CursorToRead, private->cursor_r - 0x10);
	}*/
}
/*
 * 中断例程里，假如又发生了网卡中断，那ISR上相应的bit会被置位。我们处理一个bit
 * 之后就返回。 如果ISR里还有置位，它自然还会触发新的中断。 FIXME  对吗？
 * 不，不该这样做，应该尽最快把ISR空闲出来，这样新的网卡中断能表征出来。
 */
void on_intr(int irq, void *dev, struct pt_regs *regs){
	struct net_device *netdev = dev;
	/* the interupt Status Register reflects all current pending interrupts, regardless of
	 * the state of the corresponding mask bit in the IMR.
	 */
	 //如果有一个中断发生在下面两句话中间怎么办？
	unsigned isr = RTL_readw(netdev, ISR);
	RTL_writew(netdev, ISR, 0xffff);		//clear all interrupts
	//oprintf("@isr: %x  \n", isr);

	if(isr & TxOK){
	    on_tx(netdev);	
	}
	if(isr & TxErr){
		//spin("TxErr");
	}
	if(isr & RxOK){
       	on_rx( netdev ); 
	}
	if(isr & RxErr){
        //spin("RxErr");
	}
	if(isr & RxOverflow){
		//spin( "RxOverflow");
	}
	if(isr & RxUnderrun){
		//spin( "CARP is written but Rx buffer is empty, or link status changed");
	}
	if(isr &RxOverflow){
		;
	}
	if(isr & CabLenChg){
		//spin("Cable Length Change");
	}
	if(isr & PCSTimeout){
		//spin("PCSTimeout");
	}
	if(isr & PCIErr){
		//spin("PCIErr");
	}
	if(isr == 0)
	{
		//oprintf("warning, an irq of 8139 with zero ISR. see ISR:%x \n", isr);
		return;
	}
}

/* 1, 无穷数组相对circle ring的好处，就看出来了：我们一下子就知道还有几个desc可用
 * 2,　这个函数有问题，一次是只该发送一个包的。这儿却是循环用了几个。
 * 1, 只能由net_wake_queue调用，它保证了这个函数在关中断模式下执行。
 * @return 0, if any DESC avaliable;	-1, no avaliable DESC
 * net_wake_queue() relies on it to stop Tx queue.
 */
static int rtl8139_start_xmit(struct sk_buff *skb, struct net_device *netdev){
	/*struct rtl8139_private * private  = netdev->private;
    assert(private->bsy_desc > private->touse_desc);
	assert( cli_already() );
	int realdesc = private->touse_desc % NUM_TX_DESC;
	if(skb->pkgsize < TX_BUF_SIZE){
		int size = skb->pkgsize < 60 ? 60 : skb->pkgsize;
		memset( PRIV(netdev)->tx_bufs[realdesc], 0xbb, size );
		memcpy( PRIV(netdev)->tx_bufs[realdesc], skb->ethhdr, skb->pkgsize );
		dev_free_skb(skb);
		struct TxStatusReg tsr = {value: 0};
		tsr.size = size;
		tsr.OWN = 0;
		tsr.threshold = 0;
		RTL_writel(netdev, TxStatus0 + realdesc * 4, tsr.value);
	}
	else	assert(" too large package size ");
	private->touse_desc ++;
	if(private->bsy_desc == private->touse_desc) return -1;*/
	  return 0;
}

int rtl8139_tx_busy(struct net_device *netdev){
	return PRIV(netdev)->touse_desc == PRIV(netdev)->bsy_desc;
}
#define SA_INTERRUPT (1<<1)
#define IRQ_DISABLED (1<<2)
int rtl8139_open(struct net_device *netdev){
	struct rtl8139_private *private = netdev->private;
	//private->rx_ring = kmalloc2((32 + 2) * 1024, __GFP_DMA);	/* 32K + 2K */	
	private->rx_ring_dma = private->rx_ring - PAGE_OFFSET;
	//private->tx_buf = kmalloc2( TX_BUF_SIZE * NUM_TX_DESC, __GFP_DMA);
	private->tx_buf_dma = private->tx_buf - PAGE_OFFSET;
	int i;
	for(i = 0; i < NUM_TX_DESC; i++){
		private->tx_bufs[i] = private->tx_buf + i * TX_BUF_SIZE;
	}
	//spin("spin1");	
	private->bsy_desc = 4;
	private->touse_desc = 0;
	//netdev->flags |= ND_QUEUE_STOOPED;
	netdev->flags = 0;
	netdev->tx_count = netdev->rx_count  = 0;
	//memset(&netdev->debug,  0, sizeof(netdev->debug));
	private->cursor_r = 0;
	private->rx_ring_len = 32 * 1024;

	rtl8139_reset(netdev);
	RTL_writeb(netdev, ChipCmd, CmdRxEnb | CmdTxEnb);
	RTL_writel(netdev, RBSTART, (unsigned)__priv(netdev)->rx_ring_dma);
	RTL_writel(netdev, RxConfig, AcceptAllPhys | AcceptMyPhys | AcceptBroadcast | AcceptMulticast |RxNoWrap | RxCfgRcv32K);
	/*register IRQ handler, open IMR */
	request_irq(netdev->irq, on_intr, SA_INTERRUPT, netdev);	
	irq_desc[(int)netdev->irq].status &= ~IRQ_DISABLED;
	RTL_writew(netdev, IMR, 0xffff);
	for(i = 0; i < NUM_TX_DESC; i++){
		RTL_writel(netdev, TxAddr0 + i*4, (unsigned)(private->tx_buf_dma + TX_BUF_SIZE*i));
	}
	//netdev->on_tx_bh = alloc_bh(tx_bottomhalf, netdev);
	//netdev->on_rx_bh = alloc_bh(rx_bottomhalf, netdev);
	netdev->tx_queue.root = netdev->tx_queue.tail = 0;
	netdev->rx_queue.root = netdev->rx_queue.tail = 0;
	return 0;
}


int rtl8139_init_one(struct pci_dev *pcidev, const struct pci_device_id *id){
	pci_enable_device(pcidev);
	pci_set_master(pcidev);	/*多数BIOS会清除PCI网卡的master位，导致板卡不能往主存拷数据*/
	struct net_device *netdev;// = kmalloc2( sizeof(struct net_device), 0 );
	//oprintf(" INIT ONE, netdev: %x ", netdev);
	pcidev->core = netdev;
	netdev->pcidev = pcidev;
	//register_nic(netdev);
	netdev->open = rtl8139_open;
	netdev->stop = rtl8139_stop;
	netdev->tx_busy = rtl8139_tx_busy;
	netdev->start_xmit = rtl8139_start_xmit;

	//oprintf("irq pin: %u , line: %u\n", pcidev->irqpin, pcidev->irqline);
	/* TODO ioremap, remove hard code */
	netdev->base_addr = (unsigned)( pcidev->address[1] & ~0xf) - PAGE_OFFSET;
	//oprintf("base_addr:%x\n", netdev->base_addr);
	//assert(( (netdev->base_addr + PAGE_OFFSET) >> 20) == 0xfeb);
	netdev->irq = pcidev->irqline;
	int mac[2] = {0};
	mac[0] = RTL_readl(netdev, 0);
	mac[1] = RTL_readl(netdev, 4);
	//memcpy(netdev->mac, (char *)mac, 6);
	netdev->mac[6] = 0; 
	//oprintf("MAC:%x %x\n", mac[0], mac[1]);

	struct rtl8139_private *private;//= kmalloc2( sizeof(struct rtl8139_private), 0);
	private->pcidev = pcidev;
	netdev->private = private;
	/*int i;
	for(i = 0; i <= 2; i ++){
		if(memcmp((char *)netdev->mac, ipconfigs[i].mac, 6) == 0){
			netdev->ipmask = ipconfigs[i].ipmask;
			netdev->gateway_ip = ipconfigs[i].gateway_ip;
			netdev->ip = ipconfigs[i].ip;
			break;
		}
	}*/
	/*TODO how we pick out the right bar */
	//assert(!(pcidev->address[0] & 1));	/* port mapped address */
	//assert( pcidev->address[0] & 1);	/* memory mapped address */
	//testnd = netdev;
	return 0;
}

static struct pci_driver rtl8139_driver = {
	id_table: rtl8139_id_tbl,
	probe:rtl8139_init_one,
};

void register_rtl8139_driver(void){
	pci_register_driver(&rtl8139_driver);
}
