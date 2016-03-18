#include "cna.h"
#include "cna_dcb.h"
static int cna_probe(struct pci_dev *pdev, const struct pci_device_id *id);
static void cna_remove(struct pci_dev *pdev);
static int cna_open(struct net_device *dev);
static int cna_release(struct net_device *dev);
int cna_init(struct net_device *dev);
int private_init(struct cna_private *priv);
void hw_init(struct net_device *dev);
static irqreturn_t netdev_interrupt(int irq,void *dev_id);
static int cna_tx(struct sk_buff *skb, struct net_device *dev);
static void tx_error(struct net_device *dev);
static void cna_rx(struct net_device *dev);
static void skb_copy_w(struct net_device *dev, struct sk_buff *skb, u32 skb_size, int i) ;
static int cna_set_mac(struct net_device *netdev, void *p);
static int cna_change_mtu(struct net_device *netdev, int new_mtu);
static void cna_tx_timeout(struct net_device *netdev);
//static int cna_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd);
static struct net_device_stats *cna_stats(struct net_device *dev);
static int cna_napi_poll(struct napi_struct *napi, int budget);

char cna_driver_name[] = "cna_driver";
static const struct pci_device_id cna_ids[] =		//定义pci_device_id数组
{
	{ PCI_DEVICE(0x10EE, 0x0505)},
	{ 0,},
};

MODULE_DEVICE_TABLE(pci, cna_ids);			//将pci_device_id数组导出到用户空间

int napi_en = 1; 
long int totals=0, rxover=0;
int rx_cnt=0, tx_cnt=0, rer=0, ter=0, fer=0, ovw=0, fovw=0;
unsigned long ttime, rtime;
int tx_desc_cnt = 0;	//用于记录当前占用的发送描述符数目，从而控制发送队列的开关

/********************************probe begin******************************************/
int private_init(struct cna_private *priv)	//私有数据初始化
{
	void *membase = priv->membase;
	int i;
	priv->tx=0;
	priv->rx=0;
	priv->cur = 0;

	spin_lock_init( &(priv->lock));							//初始化自旋锁
	spin_lock_init( &(priv->tx_lock));

	//printk( "the netdev private data init success you can use it now\n");

	return 0;
}

void hw_init(struct net_device *dev)		//硬件初始化
{
	struct cna_private *priv= netdev_priv(dev);
	void *membase = priv->membase;

	int i;
	unsigned int val;

	iowrite8(chip_rst_cmd, membase + chipcmd);		//复位网卡
	iowrite8(fifo_rst_cmd, membase + chipcmd );

/*暂时略去
	for(i=0;i<1000;i++)	//等待网卡可用
	{
		if(ioread8(membase + chipstat) & ready_stat)
			break;
		else
			udelay(100);
	}
*/
	barrier();
	iowrite8( mac_rst_cmd, membase + chipcmd);
	iowrite32(cpu_to_le32(~intr_enable), membase + intr_mask);	//中断禁止
	
	iowrite32( cpu_to_le32(intr_enable), membase + intr_mask);	//中断使能
	barrier();
	//printk( "the cna  hardware init success you can use it now\n");
	return ;
}

int cna_init(struct net_device *dev)		//网络设备的初始化
{
	int err, i;
	unsigned int tst_read1, tst_read2;	//test
	struct cna_private *priv = netdev_priv(dev);
	void *membase = priv->membase;
	
	err = private_init(priv);		//初始化私有数据

	if( err )
		return err;
	
//	iowrite8(0x01,membase + 0x0);	//test

	iowrite32(cpu_to_le32(0x12345678), membase + tst_addr1);	//test
	iowrite32(cpu_to_le32(0x01020304), membase + tst_addr2);	//test
		
	hw_init(dev);			//初始化硬件设备s
	//printk( "read the mac from the hardware\n");
	for(i=0; i<6; i++)
	{
		dev->dev_addr[i] = ioread8(membase + mac_addr +i);
		dev->broadcast[i]= 0xFF;
	}
	printk( "MAC ADDR: %x %x %x %x %x %x", dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2], dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);
	
	return 0;
}

static const struct net_device_ops cna_netdev_ops = {
	.ndo_open = cna_open,
	.ndo_stop = cna_release,
	.ndo_start_xmit = cna_tx,

	.ndo_set_mac_address = cna_set_mac,
	.ndo_change_mtu = cna_change_mtu,
	.ndo_tx_timeout = cna_tx_timeout,
	.ndo_get_stats = cna_stats,
};

void cna_assign_netddev_ops(struct net_device *dev)
{
	dev->netdev_ops = &cna_netdev_ops;

	//cna_dcb_init(dev);
/*
	dev->open = &cna_open;
	dev->stop = &cna_release;
	dev->hard_start_xmit = &cna_tx;
	dev->set_mac_address = &cna_set_mac;
	dev->do_ioctl = &cna_ioctl;
	dev->change_mtu = &cna_&change_mtu;
	dev->get_stats = &cna_stats;
*/
}

static int cna_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int err, memtag=0;
	unsigned long memstart, memlen, memflag;
	unsigned int memlow, memhigh;
	unsigned int membasel, membaseh;

	void *membase = NULL;

	struct net_device *dev=NULL;
	struct cna_private *priv;

	//printk( "begin probe ***************\n");

	err = pci_enable_device(pdev);
	if( err )
	{
		printk( "enable pci device error\n");
		return -EIO;	
	}
	
	//查看pci设备资源
	memstart = pci_resource_start(pdev, 0);	//BAR 0空间
	memlen = pci_resource_len(pdev, 0);
	memflag = pci_resource_flags(pdev, 0);

	memlow = memstart & (0xffff);
	memhigh = (memstart >> 16) & (0xffff);

	err = pci_request_regions(pdev, cna_driver_name);		
	if( err )
	{
		printk( "can't request enough region*\n");
		goto out;
	}
	err = -ENOMEM;

	if(memlen < cna_pci_region)
	{
		printk( "cna region is too small*\n");
		goto err;
	}
	else
	{
		if(memflag & IORESOURCE_MEM)		//
			memtag =  1;
		//printk( "this is ioremap memtype=%d begin=%04x%04x len=%lu", memtag, memhigh, memlow,  memlen);
		if(memtag)
			membase = ioremap(memstart , memlen);		//io内存映射
		else
			membase = ioport_map(memstart,memlen);		//io端口映射

		if(membase == NULL)
			goto err;
	}

	pci_set_master(pdev);					//***使能pci设备，并将pci设为主DMA
	
	dev = alloc_etherdev(sizeof(struct cna_private));	
	if(dev == NULL)
		goto net_err;

	SET_MODULE_OWNER(dev);
	SET_NETDEV_DEV(dev,&pdev->dev);
	
	dev->irq = pdev->irq;
	
	pci_set_drvdata(pdev, dev);
	priv = netdev_priv(dev);
	priv->membase = membase;
	priv->memtag = memtag;
	priv->pdev = pdev;
	
	err = cna_init(dev);
	if( err )
	{
		free_netdev(dev);
		goto net_err;
	}

	cna_assign_netddev_ops(dev);

	if(register_netdev(dev) !=0 )
	{
		printk( "register netdev error*\n");
		free_netdev(dev);
		goto net_err;
	}
		

	return 0;
	
net_err:
	if(memtag)
		iounmap(membase);
	else
		ioport_unmap(membase);

err:	
	pci_release_regions(pdev);

out:	
	pci_disable_device(pdev);
	return err;	
}


static void cna_remove(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct cna_private *priv = netdev_priv(dev);
	void *membase = priv->membase;

	printk( "this is happend in remove the cna*\n");

	iowrite8( chip_stop_cmd, membase + chipcmd);	//################################set chipcmd 0x01, notify the hardware to stop device

	unregister_netdev(dev);						//注销net_dev
		
	if(priv->membase)
	{
		if(priv->memtag)
			iounmap(priv->membase);
		else 
			ioport_unmap(priv->membase);
	}

	barrier();
	free_netdev(dev);
							//释放申请的网络设备
	pci_release_regions(pdev);					//释放申请的内存区域	
	pci_set_drvdata(pdev, NULL);
	pci_disable_device(pdev);					//禁止pci设备

	printk( "the driver is removed success*\n");
}
/********************************probe end******************************************/

/********************************open begin*********************************************/
static irqreturn_t netdev_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct cna_private *priv = netdev_priv(dev);
	void *membase = priv->membase;
	
	totals++;
	//if(printk_ratelimit())
	//printk("recieve a interrupt total recieved=%ld\n",totals);
		
	spin_lock(&(priv->lock));
	priv->status = ioread8(membase + intr_desc);		//读中断状态
	printk("intr_desc: %x\n", priv->status);
	switch(priv->status)
	{
		case intr_data:		
			barrier();
			iowrite32(cpu_to_le32(intr_enable | (intr_datamask & intr_sendcompletemask) ), membase + intr_mask );
			barrier();
			//printk( "intr_data interrupt** %d\n", priv->status);
			if(napi_en)
				napi_schedule(&priv->napi);
			else
			{
				cna_rx(dev);
				barrier();
				iowrite32( cpu_to_le32(intr_enable), membase + intr_mask);
				barrier();
			}
			break;
		case intr_send_complete:		
			barrier();
			iowrite32(cpu_to_le32(intr_enable | (intr_sendcompletemask & intr_datamask) ), membase + intr_mask );
			barrier();
			//printk( "intr_data interrupt** %d\n", priv->status);
			if(napi_en)
				napi_schedule(&priv->napi);
			else
			{
				cna_rx(dev);
				barrier();
				iowrite32( cpu_to_le32(intr_enable), membase + intr_mask);
				barrier();
			}
			break;
		case intr_fifo_error:
			fer++;
			break;
		case intr_reject:
			rer++;
//			iowrite32( cpu_to_le32(intr_enable), membase + intr_mask );
			break;
		case intr_fifo_over:
			fovw++;

			break;
		case intr_tx_error:
			ter++;
			tx_error(dev);
			break;
		case intr_data | intr_send_complete:
			barrier();
			//iowrite32(cpu_to_le32(intr_enable),membase + intr_mask );
			iowrite32(cpu_to_le32(intr_enable | (intr_datamask & intr_sendcompletemask) ), membase + intr_mask );
			barrier();
			if(napi_en)
				napi_schedule(&priv->napi);
			else
			{
				cna_rx(dev);
				barrier();
				iowrite32( cpu_to_le32(intr_enable), membase + intr_mask);
				barrier();
			}
			break;
		default:
			barrier();
			//iowrite32(cpu_to_le32(intr_enable),membase + intr_mask );
			iowrite32(cpu_to_le32(intr_enable | (intr_datamask & intr_sendcompletemask) ), membase + intr_mask );
			barrier();
			printk( "undefined interrupt** %d\n", priv->status);
			if(napi_en)
				napi_schedule(&priv->napi);
			else
			{
				cna_rx(dev);
				barrier();
				iowrite32( cpu_to_le32(intr_enable), membase + intr_mask);
				barrier();
			}
			break;
	}
	iowrite8( 0x00, membase + intr_desc);
	spin_unlock( &(priv->lock));

	return IRQ_HANDLED;
}

static int cna_open(struct net_device *dev)		//打开网络设备	注册中断 初始化设备发送队列
{
	int err, i;
	unsigned int val;

//	unsigned int tst_read1, tst_read2;

	struct cna_private *priv = netdev_priv(dev);
	void *membase = priv->membase;

	struct pci_dev *pdev = priv->pdev;

	err = pci_enable_msi(pdev);
	if( err )
	{
		printk( "can't enable msi interrupt*\n");
		return err;
	}

	netif_napi_add(dev, &priv->napi, cna_napi_poll, 64);	//
	if(napi_en)
		napi_enable(&priv->napi);			//

	priv->tx = priv->cur = le32_to_cpu(ioread32(membase + tx_offset));
	priv->rx = le32_to_cpu(ioread32(membase + rx_offset));
	tx_desc_cnt = 0;
	printk("Initial: tx: %d, rx: %d\n", priv->tx, priv->rx);
	//priv->rx = (priv->rx+1) % DESC_NUM;

/*###########################*/
	for(i=0; i<DESC_NUM; i++)
	{
		priv->dma_rx_buff[i] = pci_alloc_consistent(priv->pdev, dmasize, &priv->dma_rx_addr[i]);//DMA一致性映射，
		                                                                                        //返回的是内核虚拟地址供驱动程序使用，
		                                                                                        //总线地址存入最后一个参数中
		if(priv->dma_rx_buff[i] == NULL)
		{
			err = ENOMEM;
			return err;
		}
		barrier();
		iowrite32( cpu_to_le32(priv->dma_rx_addr[i] + 0xc0000000), membase + dma_rx_buffer + 64*i + 8);
		iowrite32( cpu_to_le32(dmasize), membase + dma_rx_buffer + 64*i + 0x18);
		barrier();

		val = le32_to_cpu(ioread32(membase + dma_rx_buffer + 64*i + 0));
		//printk( "0x00: %x \n", val);
		val = le32_to_cpu(ioread32(membase + dma_rx_buffer + 64*i + 8));
		//printk( "0x08: %x \n", val);
		val = le32_to_cpu(ioread32(membase + dma_rx_buffer + 64*i + 0x18));
		//printk( "0x18: %x \n", val);
	}

	for(i=0;i<DESC_NUM;i++)
	{
		priv->dma_tx_buff[i] = pci_alloc_consistent(priv->pdev, dmasize, &priv->dma_tx_addr[i]);
		if(priv->dma_tx_buff[i] == NULL)
		{
			err = ENOMEM;
			return err;
		}
		barrier();
		iowrite32( cpu_to_le32(priv->dma_tx_addr[i]+ 0xc0000000), membase + dma_tx_buffer + 64*i+8);
//		iowrite32( cpu_to_le32(dmasize), membase + dma_tx_buffer + 64*i + 0x18);
		barrier();
	}
	barrier();
	iowrite8( 0x02, membase + chipstat);
/*	
	iowrite32( cpu_to_le32(0), membase + tx_head);
	iowrite32( cpu_to_le32(0), membase + tx_tail);
	iowrite32( cpu_to_le32(0), membase + rx_head);
	iowrite32( cpu_to_le32(0), membase + rx_tail);
*/
	barrier();
/*###########################*/	


	err = request_irq(pdev->irq, &netdev_interrupt, 0, "cna_intr", dev);	//在系统中注册网络中断
	if( err )
	{
		printk( "irq request failed***errno = %d\n", err);			//IRQF_DISABLED
		return err;
	}
	else
		netif_start_queue(dev);			//开始设备队列
    iowrite32( cpu_to_le32(0x1234), membase + 0x20);	//????????????
	
	nf10_dcb_init(dev);		//DCB initialize
	printk( "the netdev is been opened ,you can use the net next irq =%d\n", pdev->irq);
	return 0;
}

static int cna_release(struct net_device *dev)	//释放网络设备
{
	struct cna_private *priv = netdev_priv(dev);
	struct pci_dev *pdev = priv->pdev;
	void *membase = priv->membase;
	int i;

	
	printk( "called disable msi\n");
	free_irq(pdev->irq,dev);
	pci_disable_msi(pdev);	
	
	if(napi_en)
		napi_disable(&priv->napi);
	netif_napi_del(&priv->napi);
	
	netif_stop_queue(dev);

	
	for(i=0;i<DESC_NUM;i++)
	{
		pci_free_consistent(priv->pdev, dmasize,priv->dma_rx_buff[i], priv->dma_rx_addr[i]);
		iowrite32(cpu_to_le32(0x00), membase  + dma_rx_buffer + 64*i + 8);
	}

	for(i=0;i<DESC_NUM;i++)
	{
		pci_free_consistent(priv->pdev, dmasize,priv->dma_tx_buff[i], priv->dma_tx_addr[i]);
		iowrite32(cpu_to_le32(0x00), membase  + dma_tx_buffer + 64*i + 8);
		iowrite32(cpu_to_le32(0x00), membase  + dma_tx_buffer + 64*i + 0x18);
	}

	iowrite32( cpu_to_le32(priv->cur), membase + tx_offset);	//
	iowrite32( cpu_to_le32(priv->rx), membase + rx_offset);		//

	printk( "the netdev is now released**\n");

	return 0;
}
/********************************open end*********************************************/

/********************************send begin********************************************/
static int cna_tx(struct sk_buff *skb,struct net_device *dev)		//发送网络数据包
{
	struct cna_private *priv = netdev_priv(dev);
	void *membase = priv->membase;
	int tx, cur, isopt = 0;	//isopt表示发送的帧是否过小而被填充
	struct timeval start, end;
	unsigned char *data;
	unsigned char *skbopt;
	unsigned int skb_len = skb->len, j;
	unsigned int ctrl;

//	tx = priv->tx;
	cur= priv->cur;
	data = skb->data;

	//printk( "head:%x data:%x gap:%d\n", skb->head, skb->data, skb->data-skb->head);
	if(!netif_running(dev))
	{
		printk( " the dev is not enabled *\n");
		return 0;
	}

	if(skb_len < ETH_ZLEN)	//当数据包过小时，需填充
	{
		skbopt = (unsigned char*)kmalloc(ETH_ZLEN, GFP_ATOMIC); //当帧过短时复制到这块区域并填充至一定大小，注意一定要定义在堆上，至硬件确认发送完毕后再释放
		memset(skbopt, 0, ETH_ZLEN*sizeof(unsigned char));
		memcpy(skbopt, data, skb_len);
		skb_len = ETH_ZLEN;
		data = skbopt;
		isopt = 1;	//被填充
	}

	do_gettimeofday(&start);
	spin_lock(&(priv->tx_lock));
	dev->trans_start = jiffies;
	
//	barrier();

	//ctrl = le32_to_cpu(ioread32(membase + dma_tx_buffer + 8*cur));	
	ctrl = le32_to_cpu(ioread32(membase + dma_tx_buffer + 64*cur + 0x1c));	
//	printk("ctrl: %x\n", ctrl);
	if( (ctrl & 0x80000000) == 0 )
	{
		memcpy(priv->dma_tx_buff[cur], data, skb_len);

		barrier();
		iowrite32(cpu_to_le32(0x0c<<24 | skb_len),membase + dma_tx_buffer + 64*cur + 0x18);
		//iowrite32(cpu_to_le32(0), membase + dma_tx_buffer + 64*cur + 0x1c);
		barrier();

		if(isopt == 1)
		{
			kfree( data );
		}

		priv->cur = (cur + 1) % DESC_NUM;	
		tx_cnt++;
		tx_desc_cnt++;
		if( !( (tx_desc_cnt + 1 )<= DESC_NUM ) )	//发送描述符全部占用，通知内核关闭发送队列
		{
			printk( "netif_stop_queue\n");
			netif_stop_queue(dev);
		}
	
		barrier();
		iowrite32( cpu_to_le32(priv->cur), membase + tx_tail);	//#################
		barrier();

		printk( "TX sucess!\n");
	}
	else
	{
		priv->stats.tx_dropped++;
		dev_kfree_skb(skb);
		if(isopt == 1)
		{
			kfree( data );
		}
		spin_unlock(&(priv->tx_lock));
		//printk( "cna_tx:Tx buffers full");
		printk( "Tx packet drop protocol\n");
		return NETDEV_TX_OK;
	}

	dev_kfree_skb(skb);

	priv->stats.tx_packets++;
	priv->stats.tx_bytes += skb_len;
	spin_unlock(&(priv->tx_lock));
	do_gettimeofday(&end);
	ttime = (end.tv_sec - start.tv_sec)*1000000 + end.tv_usec - start.tv_usec;
	//tx_cnt++;
	//printk( "tx_cnt=%d rx_cnt=%d rer=%d fovw =%d total=%ld rxover=%ld ttime=%ld rtime=%ld \n", tx_cnt, rx_cnt, rer, fovw, totals, rxover, ttime, rtime);

	return NETDEV_TX_OK;	
}

static void tx_error(struct net_device *dev)
{
	struct cna_private *priv = netdev_priv(dev);
	int i;
	unsigned int state;
	priv->stats.tx_errors++;
	
	iowrite8( fifo_rst_cmd, priv->membase + chipcmd );
	printk( "send packet error * \n");
}
/********************************send end***********************************************/

/********************************receive begin******************************************/
static void skb_copy_w(struct net_device *dev, struct sk_buff *skb, u32 skb_size, int i) 
{
	struct cna_private *priv = netdev_priv(dev);
	
	void *dma_buffer = priv->dma_rx_buff[i];
	memcpy(skb_put(skb,skb_size), dma_buffer, skb_size);
	
	return ;
}

static void cna_rx(struct net_device *dev)		//接收数据包
{
	struct cna_private *priv = netdev_priv(dev);
	void *membase = priv->membase;
	struct sk_buff *skb;
	unsigned char *data;
	struct timeval start, end;
	unsigned int state, dma_ctrl, intr_now;
	int i, skb_size;
	int rx;
	int j, num;
	int tag =0, lastrv;
	int tx, cur;
	
	if(!netif_running(dev))
		return;
	//printk( "the driver recieve a packet * \n");
	do_gettimeofday(&start);

/**********send completion*****/
	tx = priv->tx;
	//cur = priv->cur;
	cur = le32_to_cpu(ioread32(membase + tx_head));
	while( tx != cur )
	{
		//dma_ctrl = le32_to_cpu(ioread32(membase + dma_tx_buffer + 8*tx));
		dma_ctrl = le32_to_cpu(ioread32(membase + dma_tx_buffer + 64*tx + 0x1c));
		if( (dma_ctrl & 0x80000000) !=  0 )
		{
			tx_desc_cnt--;
			barrier();
			iowrite32(cpu_to_le32(0), membase + dma_tx_buffer + 64*tx + 0x1c);
			barrier();
		}
		else
		{
//			printk( "hardware send a little slow!");
			break;
		}

		tx = (tx + 1) % DESC_NUM;
	}

	if(priv->tx != tx)
		priv->tx = tx;

	if(tx_desc_cnt + 4 <= DESC_NUM)			//发送描述符有一定的数目可用，如果发送队列关闭则通知内核唤醒并开启
	{
		if( netif_queue_stopped(dev) )
		{
			printk( "netif_wake_queue \n");
        	netif_wake_queue(dev);
		}
	}
	intr_now = le32_to_cpu( ioread32(membase + intr_mask));
	barrier();
	iowrite32( cpu_to_le32(intr_now & ~intr_sendcompletemask), membase + intr_mask);	//重新打开发送完成通知中断
	barrier();
/**********send completion*****/

	//rx = le32_to_cpu(ioread32(membase + rx_offset));
	i = priv->rx;
	lastrv = i;

	while( 1 )
	{
		state = le32_to_cpu(ioread32(membase + dma_rx_buffer + 64*i + 0x1c));
		//printk( "RX begin!");
		if( (state & 0x80000000) !=  0)
		{
			skb_size = state & 0x007fffff;

			//printk( "REVC_LEN:%d", skb_size);
			skb = dev_alloc_skb(skb_size + 2);
			if(!skb)
			{
				priv->stats.rx_dropped++;
				printk( "can't have enough memory packet dropped*\n");
				continue;	
			}
			skb_reserve(skb, 2);
			skb_copy_w(dev, skb, skb_size, i);
			
			data = skb->data;
			//printk( "Rx packet sucess protocol: %x %x\n", data[16], data[17]);

			skb->dev = dev;
			skb->protocol = eth_type_trans(skb, dev);

			netif_rx(skb);
			dev->last_rx = jiffies;
			priv->stats.rx_packets++;
			priv->stats.rx_bytes += skb_size;
			rx_cnt++;

			barrier();
			iowrite32(cpu_to_le32(0), membase + dma_rx_buffer + 64*i + 0x1c);
			barrier();

			tag = 2;
			lastrv = (i + 1) % DESC_NUM;

			barrier();
			iowrite32( cpu_to_le32(lastrv), membase + rx_tail);	//###################
			barrier();
			
			//printk( "RX sucess!\n");
		}
		else
		{
			//printk( "cna_rx:  recv handle not called\n");
			if(tag == 2)
				break;
			tag = 1;
			//break;
		}

		i = (i+1) % DESC_NUM;
		if( i == rx && tag == 1)	//接收一轮之后暂停接收处理
			break;
	}

	do_gettimeofday(&end);
	rtime = (end.tv_sec - start.tv_sec)*1000000 + end.tv_usec - start.tv_usec;
	if ( priv->rx != lastrv)
	{
		priv->rx = lastrv;
		//iowrite32( cpu_to_le32(priv->rx), membase + rx_offset);
	}
//	iowrite32( cpu_to_le32(priv->rx), membase + rx_tail);	//###################

	return ;	
}

static int cna_napi_poll(struct napi_struct *napi, int budget)
{
	struct net_device *dev = napi->dev;
	int n_rx = 0;

	struct cna_private *priv = netdev_priv(dev);
	void *membase = priv->membase;
	struct sk_buff *skb;
	unsigned char *data;
	struct timeval start, end;
	unsigned int state, dma_ctrl, intr_now, desc_now;
	int i, skb_size;
	int rx;
	int j, num;
	int tag = 0, lastrv;
	int tx, cur, head;
	
	if(!netif_running(dev))
		return budget;

	do_gettimeofday(&start);

/**********send completion*****/
	tx = priv->tx;
//	cur = priv->cur;
//	while( tx != cur || tx_desc_cnt == DESC_NUM)
	cur = (le32_to_cpu(ioread32(membase + tx_head)) + 1) % DESC_NUM;
	printk("head: %d\n", (cur - 1 + DESC_NUM) % DESC_NUM);
	while( tx != cur )
	{
		//dma_ctrl = le32_to_cpu(ioread32(membase + dma_tx_buffer + 8*tx));
		dma_ctrl = le32_to_cpu(ioread32(membase + dma_tx_buffer + 64*tx + 0x1c));
		if( (dma_ctrl & 0x80000000) !=  0 )
		{
			tx_desc_cnt--;
			barrier();
			iowrite32(cpu_to_le32(0x00000000), membase + dma_tx_buffer + 64*tx + 0x18);
			iowrite32(cpu_to_le32(0x00000000), membase + dma_tx_buffer + 64*tx + 0x1c);
			barrier();

		}
		else
		{
			head = le32_to_cpu(ioread32(membase + tx_head));
			printk( "hardware send a little slow!  head: %d", head);

			if( tx_desc_cnt == DESC_NUM )
			{
				printk( "  head_2: %d ", head);
			}

			break;
		}

		tx = (tx + 1) % DESC_NUM;
	}
/*???????????????????  tao quan tiao shi
	if(tx_desc_cnt == DESC_NUM)
		printk("tx catch current\n");
???????????????????*/

	if(priv->tx != tx)
		priv->tx = tx;

	if(tx_desc_cnt + 32 <= DESC_NUM)			//发送描述符有一定的数目可用，如果发送队列关闭则通知内核唤醒并开启
	{
		if( netif_queue_stopped(dev) )
        {
			printk( "netif_wake_queue \n");
        	netif_wake_queue(dev);
		}
	}
	intr_now = le32_to_cpu( ioread32(membase + intr_mask));
	barrier();
	iowrite32( cpu_to_le32(intr_now & ~intr_sendcompletemask), membase + intr_mask);	//重新打开发送完成通知中断
	barrier();
/**********send completion*****/
	
	//	rx = le32_to_cpu(ioread32(membase + rx_offset));
	i = rx = priv->rx;
	lastrv = i; 
	//printk( "in poll\n");

	while( n_rx < budget )
	{
		state = le32_to_cpu(ioread32(membase + dma_rx_buffer + 64*i + 0x1c));
		//printk( "RX begin!");
		if( (state & 0x80000000) !=  0)
		{
			skb_size = state & 0x007fffff;

			//printk( "REVC_LEN:%d", skb_size);
			skb = dev_alloc_skb(skb_size + 2);
			if(!skb)
			{
				priv->stats.rx_dropped++;
				printk( "can't have enough memory packet dropped*\n");
				continue;	
			}
			skb_reserve(skb, 2);
			skb_copy_w(dev, skb, skb_size, i);
			
			data = skb->data;
			//for( num = 0; num < 12; num += 2)
			//	printk( "Rx packet sucess protocol: %x %x\n", data[num], data[num+1]);
			//printk("\n");

			skb->dev = dev;
			skb->protocol = eth_type_trans(skb, dev);

			netif_rx(skb);
			dev->last_rx = jiffies;
			priv->stats.rx_packets++;
			priv->stats.rx_bytes += skb_size;
			rx_cnt++;

			barrier();
			iowrite32(cpu_to_le32(0), membase + dma_rx_buffer + 64*i + 0x1c);
			barrier();

			tag = 2;
			lastrv = (i + 1) % DESC_NUM;

			barrier();
			iowrite32( cpu_to_le32(lastrv), membase + rx_tail);	//###################
			barrier();
			
			n_rx++;
			//printk( "RX sucess!");
		}
		else
		{
			//printk( "cna_rx:  recv handle not called\n");
			if(tag == 2)
				break;
			tag = 1;
		}

		i = (i+1) % DESC_NUM;
		if( i == rx && tag == 1)	//接收一轮之后暂停接收处理
			break;
	}

	do_gettimeofday(&end);
	rtime = (end.tv_sec - start.tv_sec)*1000000 + end.tv_usec - start.tv_usec;

	if ( priv->rx != lastrv)
	{
		priv->rx = lastrv;
		//iowrite32( cpu_to_le32(priv->rx), membase + rx_offset);
	}

//	iowrite32( cpu_to_le32(priv->rx), membase + rx_tail);	//###################

	if(n_rx < budget)
	{
		//printk( "NAPI complete!");
		napi_complete(napi);		//设备已无数据要接收，napi完成
		barrier();
		iowrite32( cpu_to_le32(intr_enable), membase + intr_mask);	//重新打开中断
		barrier();
	}
	else
	{
		//printk( "wait for next schedule poll!");
		return budget;			//等待下一次poll
	}
	
	return 0;	
}
/********************************receive end********************************************/

static int cna_set_mac(struct net_device *netdev, void *p)
{
	struct cna_private *priv = netdev_priv(netdev);
	void *membase = priv->membase;
	struct sockaddr *addr = p;
	u32 rar_high;
	u32 rar_low;
	u8 mac_addr[netdev->addr_len];
	int i;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	//memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	//memcpy(hw->mac.addr, addr->sa_data, netdev->addr_len);
	memcpy(mac_addr, addr->sa_data, netdev->addr_len);
//	rar_low = mac_addr[3]<<24 | mac_addr[2]<<16 | mac_addr[1]<<8 | mac_addr[0];
//	rar_high = mac_addr[5]<<8 | mac_addr[4];

	//printk( "write the fcoe_mac to the hardware\n");
	for(i=0; i<6; i++)
	{
		iowrite8(cpu_to_le32(mac_addr[i]), membase + fcoe_mac_addr + i);
		printk( "%x", mac_addr[i]);
	}
	
//	iowrite32(cpu_to_le32(rar_low), membase + fcoe_macaddr_low);
//	iowrite32(cpu_to_le32(rar_high), membase + fcoe_macaddr_high);

	printk( "The foce mac addr is:%x%x%x%x%x%x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

	return 0;
}

static int cna_change_mtu(struct net_device *netdev, int new_mtu)
{
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN;

	/* MTU < 68 is an error and causes problems on some kernels */
	if ((new_mtu < 68) || (max_frame > CNA_MAX_JUMBO_FRAME_SIZE))
		return -EINVAL;
	
//	printk( "changing MTU from %d to %d\n", netdev->mtu, new_mtu);

	/* must set new MTU before calling down or up */
	netdev->mtu = new_mtu;
/*
	if (netif_running(netdev))
	{
		netif_stop_queue(netdev);
		netif_start_queue(netdev);
	}
*/
	return 0;
}

static void cna_tx_timeout(struct net_device *netdev)
{
    printk(KERN_WARNING "%s: WARNING: cna_tx_timeout(): hit timeout!\n", cna_driver_name);
}


static struct net_device_stats *cna_stats(struct net_device *dev)
{
	struct cna_private *priv = netdev_priv(dev);
	return &priv->stats;
}

static struct pci_driver my_driver = {			//创建一个pci_driver设备驱动程序结构体 并初始化
	.name 		= 	"cna netdriver",
	.id_table 	= 	cna_ids,
	.probe 		= 	cna_probe,
	.remove		= 	cna_remove,
};

static int __init cna_init_module(void)		//将我们的pci_driver结构体注册到系统PCI
{
	return pci_register_driver(&my_driver);
}

static void __exit cna_remove_module(void)		//从系统中删除驱动模块
{
	pci_unregister_driver(&my_driver);
}


//驱动模块加载
module_init(cna_init_module);
//驱动模块卸载
module_exit(cna_remove_module);
	
