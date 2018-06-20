#include "kernel.h"
#include "ne2000.h"
/* vendor 和 device就能唯一的指定设备类型*/
static struct pci_device_id ne2000_id_tbl[] = {
	{0x10ec,0x8029,0,0},
	{0,0,0,0},
};
static struct net_device netdev_obj;
static inline void RTL_maskb(struct net_device *netdev, int offset, unsigned mask){
	maskb(netdev->base_addr + offset, mask);
}
static inline void RTL_maskw(struct net_device *netdev, int offset, unsigned mask){
	maskw(netdev->base_addr + offset, mask);
}
static inline void RTL_maskl(struct net_device *netdev, int offset, unsigned mask){
	maskl(netdev->base_addr + offset, mask);
}


static inline u8 RTL_readb(struct net_device *netdev, int offset){
	return readb((netdev->base_addr & 0xffff) + offset);
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

int ne2000_read(struct net_device*	dev,
		    void*		buf,
		    unsigned short	count,
		    unsigned short	offset)
{
  char* p = 0;
  int ret = count;

  // Sets RD2 (abort/complete remote DMA)
  RTL_writeb(dev,0x0,(RTL_readb(dev,NE2000_CR) & ~(NE2000_CR_RD0 | NE2000_CR_RD1)) |
       NE2000_CR_RD2);

  /* These two registers set the start address of remote DMA. */
  RTL_writeb( dev, NE2000_RSAR0,offset);
  RTL_writeb(dev,NE2000_RSAR1,offset >> 8);

  /* These two registers set the data byte counts of remote DMA. */
  RTL_writeb(dev,NE2000_RBCR0,count);
  RTL_writeb( dev,NE2000_RBCR1,count >> 8);

  // Sets RD0 (remote read)
  RTL_writeb(dev,0x0,(RTL_readb(dev,NE2000_CR) & ~(NE2000_CR_RD1 | NE2000_CR_RD2)) |
       NE2000_CR_RD0);

  for (p = buf;  count > 0;  count--, p++)
  {
    *p = RTL_readb(dev,NE2000_DMA_PORT);
  }

  return (ret);
}

int ne2000_write(struct net_device* dev,
		     void*	 buf,
		     unsigned short	 count,
		     unsigned short	 offset)
{
  char* p = 0;
  int ret = count;

  // Sets RD2 (abort/complete remote DMA)
  RTL_writeb(dev,0x0,(RTL_readb(dev, NE2000_CR) & ~(NE2000_CR_RD0 | NE2000_CR_RD1)) |
       NE2000_CR_RD2);

  /* These two registers set the start address of remote DMA. */
  RTL_writeb( dev, NE2000_RSAR0,offset);
  RTL_writeb( dev, NE2000_RSAR1,offset >> 8);

  /* These two registers set the data byte counts of remote DMA. */
  RTL_writeb( dev, NE2000_RBCR0,count);
  RTL_writeb( dev, NE2000_RBCR1,count >> 8);

  // Sets RD1 (remote write)
  RTL_writeb(dev,0x0,(RTL_readb(dev, NE2000_CR) & ~(NE2000_CR_RD0 | NE2000_CR_RD2)) |
       NE2000_CR_RD1);

  for (p = buf; count > 0; count--, p++)
  {
    RTL_writeb(dev, NE2000_DMA_PORT,*p);
  }

  return (ret);
}



/**
 *  @brief Enqueues a packet in the appropriate queue
 *
 */
static inline
void rtl8029_enqueue (pok_packet_t *packet)
{
  pok_queue_t*	queue;// = dev.recv_buf + packet->udp.dst;
  unsigned int	off = 0;
  unsigned int	i = 0;

  /* overflow? */
  if (queue->len + packet->udp.len > RECV_BUF_SZ)
  {
    print("rtl8029_read: error: overflow!\n", 31);
    return;
  }

  /* at which offset should we start writing? */
  off = (queue->off + queue->len) % RECV_BUF_SZ;

  /* copying data from the packet to the circular buffer in the queue */
  for (i = 0; i < packet->udp.len; i++)
  {
    queue->data[off] = packet->data[i];
    off = (off + 1) % RECV_BUF_SZ;
  }

  /* updating data length in this queue */
  queue->len += packet->udp.len;
}

/**
 *  @brief Reads data from the corresponding network stack
 *
 *  Reads enqueued data in the stack partition.
 */
void rtl8029_read (int /*pok_port_id_t*/ port_id, void* data, unsigned int len)
{
  int/*pok_port_id_t*/ global;
  int/*pok_port_id_t*/     ret;
	#define POK_ERRNO_OK 0
  ret = POK_ERRNO_OK;//pok_port_virtual_get_global (port_id, &global);

  if (ret == POK_ERRNO_OK)
  {
    char	*dest = data;
    pok_queue_t* queue;// = dev.recv_buf + global;
    unsigned int	size = len < queue->len ? len : queue->len;
    unsigned int	copied = 0;

    /* is there something to read ? */
    if (queue->len == 0)
    {
      print("rtl8029_read: error: empty read ring buffer\n",44);
      return;
    }

    /* copy from the queue to the buffer */
    for (copied = 0; copied < size; copied++)
    {
      dest[copied % RECV_BUF_SZ] = queue->data[queue->off];
      queue->off = (queue->off + 1) % RECV_BUF_SZ;
    }

    /* updating data length in this queue */
    queue->len -= size;
  }
}


/**
 *  @brief Polls rtl8029 device.
 *
 *  Watches for events, typically for receiving queued packets.
 */
void rtl8029_polling ()
{
  unsigned char	state; // ISR state
	struct net_device *netdev=&netdev_obj;
  NE2000_SELECT_PAGE(netdev, 0);
	
  while (1)
  {
    // do we have an interrupt flag set?
    if ((state = RTL_readb(netdev, NE2000_ISR)) == 0)
      continue;

    if (state & NE2000_ISR_PRX)
    {
      if ((RTL_readb(netdev, NE2000_RSR) & NE2000_RSR_PRX) == 0)
      {
				// error
				print("error occur\n",12);
      }

      //print("[*]\n",4);

      /* no errors */
      s_ne2000_header	ne2000_hdr;	// ne2000 packet header
      unsigned short	offset;		// dma offset
      unsigned char	start, end;	// pointers for the ring buffer
      pok_packet_t	recv_packet;

      while (1)
      {
				//print("1\n",2);
				/* This register is used to prevent overwrite of the receive buffer ring.
					 It is typically used as a pointer indicating the last receive buffer
					 page the host has read.*/
				start = RTL_readb(netdev , NE2000_BNRY) + 1;

				/* This register points to the page address of the first receive
					 buffer page to be used for a packet reception. */
				NE2000_SELECT_PAGE(netdev, 1);
				end = RTL_readb(netdev, NE2000_CURR);
				NE2000_SELECT_PAGE(netdev, 0);

				if ((end % NE2000_MEMSZ) == (start % NE2000_MEMSZ) + 1)
				{
					break;
				}
				//print("2\n",2);
				/* et on decapsule! */
				offset = start << 8;
				// ne2000 header
				offset += ne2000_read(netdev, &ne2000_hdr, sizeof(s_ne2000_header),
									offset);
				//print("3\n",2);
				//printInt(ne2000_hdr.size,16);
				if(ne2000_hdr.size>1){
					printInt(ne2000_hdr.size,16);
					print("\n",1);
				}
				if(NET_DATA_MAXLEN>ne2000_hdr.size - sizeof(s_ne2000_header)){
					ne2000_read(netdev, &recv_packet,
								ne2000_hdr.size - sizeof(s_ne2000_header), offset);
					//rtl8029_enqueue(&recv_packet);处理收到的数据包
					print("4\n",2);
					// update the BNRY register... almost forgot that
					RTL_writeb( netdev, NE2000_BNRY, ne2000_hdr.next > NE2000_MEMSZ ?
							 NE2000_RXBUF - 1 : ne2000_hdr.next - 1);
				}else
					;//print("5\n",2);
      }

      RTL_writeb(netdev,NE2000_ISR,NE2000_ISR_PRX); // Clear PRX flag
    }

    if (state & NE2000_ISR_PTX)
    {
      RTL_writeb( netdev, NE2000_ISR,NE2000_ISR_PTX); // Clear PTX flag
    }

    if (state & NE2000_ISR_RXE)
    {
      RTL_writeb( netdev, NE2000_ISR,NE2000_ISR_RXE); // Clear RXE flag
    }

    if (state & NE2000_ISR_TXE)
    {
      RTL_writeb( netdev, NE2000_ISR,NE2000_ISR_TXE); // Clear TXE flag
    }

    if (state & NE2000_ISR_OVW)
    {
      RTL_writeb( netdev, NE2000_ISR,NE2000_ISR_OVW); // Clear OVW flag
    }

    if (state & NE2000_ISR_CNT)
    {
      RTL_writeb(netdev, NE2000_ISR,NE2000_ISR_CNT); // Clear CNT flag
    }

    if (state & NE2000_ISR_RST)
    {
      RTL_writeb(netdev, NE2000_ISR,NE2000_ISR_RST); // Clear RST bit
    }
  }
}

int ne2000_init_one(struct pci_dev *pcidev, const struct pci_device_id *id){
	int i;
	pci_enable_device(pcidev);
	pci_set_master(pcidev);
	
	struct net_device *netdev=&netdev_obj;
	print("io base:",8);
	//printInt(pcidev->address[0],16);
	//while(1){}
	netdev->base_addr = (unsigned)( pcidev->address[0] & 0xffffffe0)+0xc0000000;
	
	//while(1){}
 NE2000_SELECT_PAGE(netdev, 0);

  /* This bit is the STOP command. When it is set, no packets will be
     received or transmitted. POWER UP=1. */
  RTL_writeb(netdev,NE2000_CR,NE2000_CR_STP);

  // Sets several options... Read the datasheet!
  RTL_writeb(netdev,NE2000_TCR,0);
  RTL_writeb(netdev,NE2000_RCR,NE2000_RCR_AB);
  RTL_writeb(netdev,NE2000_DCR,NE2000_DCR_LS | NE2000_DCR_FT1);

  /* The Page Start register sets the start page address
     of the receive buffer ring. */
  RTL_writeb(netdev,NE2000_PSTART,NE2000_RXBUF);
  /* The Page Stop register sets the stop page address
     of the receive buffer ring. */
  RTL_writeb(netdev,NE2000_PSTOP,NE2000_MEMSZ);
  /* This register is used to prevent overwrite of the receive buffer ring.
     It is typically used as a pointer indicating the last receive buffer
     page the host has read. */
  RTL_writeb(netdev,NE2000_BNRY,NE2000_RXBUF);

  /* These two registers set the data byte counts of remote DMA. */
  RTL_writeb(netdev,NE2000_RBCR0,0);
  RTL_writeb(netdev,NE2000_RBCR1,0);

  NE2000_SELECT_PAGE(netdev, 1);

  /* This register points to the page address of the first receive buffer
     page to be used for a packet reception. */
  RTL_writeb(netdev,NE2000_CURR,NE2000_RXBUF + 1);
	//printInt(RTL_readb(netdev,0x0),16);
	u8 mac[6*2];
	ne2000_read(netdev,mac,6*2,0);
	for(i=0;i<6;i++){
		//mac[i] = RTL_readb(netdev,0x8+i);
		print("MAC:",4);
		int u=i*2;
		printInt(mac[u],16);
	}
	//while(1);
  RTL_writeb(netdev, NE2000_PAR0,mac[0]);
  RTL_writeb(netdev, NE2000_PAR1,mac[2]);
  RTL_writeb(netdev, NE2000_PAR2,mac[4]);
  RTL_writeb(netdev, NE2000_PAR3,mac[6]);
  RTL_writeb(netdev, NE2000_PAR4,mac[8]);
  RTL_writeb(netdev, NE2000_PAR5,mac[10]);

	NE2000_SELECT_PAGE(netdev, 0);
	RTL_writeb( netdev, NE2000_CR,NE2000_CR_STA);
	
  // Reactivating interrupts
  /* ISR register must be cleared after power up. */
  RTL_writeb(netdev, NE2000_ISR,0xFF);

  /* All bits correspond to the bits in the ISR register. POWER UP=all 0s.
     Setting individual bits will enable the corresponding interrupts. */
  /* Since POK use polling, ALL interrupts are disabled */
  RTL_writeb(netdev, NE2000_IMR,0);
	return 0;

}
static struct pci_driver ne2000_driver = {
	id_table: ne2000_id_tbl,
	probe:ne2000_init_one,
};
void register_ne2000_driver(){
	pci_register_driver(&ne2000_driver);
	rtl8029_polling();
}
