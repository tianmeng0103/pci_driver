#include<linux/module.h>
#include<linux/init.h>
#include<linux/pci.h>
#include<linux/netdevice.h>
#include<linux/etherdevice.h>
#include<linux/skbuff.h>
#include<linux/errno.h>
#include<linux/mm.h>
#include<linux/interrupt.h>
#include<linux/semaphore.h>
#include<linux/spinlock.h>
#include<asm/io.h>
#include<asm/system.h>
#include<asm/uaccess.h>
#include<asm/delay.h>
//#include<linux/dma-mapping>
//#include<linux/byteorder/little_endian.h>
#include<asm/byteorder.h>
#include<linux/sched.h>
#include<linux/mod_devicetable.h>
#include<linux/string.h>
#include <linux/if_ether.h>


#ifndef SET_MODULE_OWNER
#define SET_MODULE_OWNER(dev)
#endif

#define io_region  "net_resource"
#define cna_pci_region 0x8000	//网卡的io或内存区域长度

#define DESC_NUM	128

#define packetsize	2048
#define dmasize		2200
#define CNA_MAX_JUMBO_FRAME_SIZE	9728

enum registers{
	chipstat   = 0x00,		//8位状态	
	intr_mask  = 0x04,		//32位中断使能
	intr_desc  = 0x08,		//8位中断	
	chipcmd    = 0x0c,		//8位芯片复位
		
	mac_addr       = 0x10,		//MAC地址寄存器，连续6个字节，从高位至低位

	fcoe_mac_addr       = 0x28,	//FCOE MAC地址寄存器，连续6个字节，从高位至低位

//	fcoe_macaddr_low = 0x28,
//	fcoe_macaddr_high = 0x2c,

	rx_offset	=0x18,		//
	tx_offset	=0x1c,		//

	tst_addr1 	=0x20,		//用作测试
	tst_addr2	=0x24,		//用作测试

	dma_rx_buffer  = 1024,	//DMA 发送描述符基地址 0x0400 1024  1k处
	dma_tx_buffer  = 17024,	//DMA 接受描述符基地址 0x4000 16384 16k处

	tx_head 	=	0x30,
	tx_tail		=	0x34,
	rx_head		=	0x38,
	rx_tail		=	0x3c,
};
	
enum registers_content{
	rx_len_mask = 0xffff,		//接收长度
	tx_len_mask = 0xffff,		//发送长度
	
	recv_enable = 0x10000,		//接收有效位
	send_enable = 0x10000,		//发送有效位
	
	send_timeout =0x20000,		//发送超时
	send_overflow = 0x40000,	//v6板溢出 接收数据 复位芯片	
	send_error = 0x80000,		//发送出错 接收数据 复位芯片
	
	intr_data = 0x01,		//数据到达中断
	intr_send_complete = 0x02,	//发送完成
	intr_fifo_error = 0x04,		//fifo error 中断
	intr_reject = 0x08,		//拒绝接收中断
	intr_fifo_over = 0x10,		//fifo over 中断
	intr_tx_error  = 0x20,
	
	intr_enable	= 0x80000000,	//中断使能
	intr_datamask	=0x01,		//中断屏蔽位 0使能 1禁止
	intr_sendcompletemask = 0x02, 
	intr_fifoerrormask = 0x04,
	intr_rejectmask = 0x08,
	intr_fifoovermask =0x10,
	intr_txerrormask =0x20,
	
	chip_rst_cmd = 0x40,		//芯片复位
	mac_rst_cmd =0x80,
	fifo_rst_cmd = 0x10,		//重启发送fifo
	chip_stop_cmd = 0x01,		//stop the hardware when driver remove

	ready_stat = 0x01,
};
	
	
struct cna_private{
	void *membase;			//网卡起始基地址
	int memtag;			//判别为IO内存映射还是端口映射，1为IO内存映射,0为端口映射

	spinlock_t lock;		//在中断处理函数中使用，用于保证单一的读取中断状态并作出中断处理后重新打开
	spinlock_t tx_lock;		//用于发送数据，保证每次发送一个数据帧

	int cur;			//cur指向下一个也就是即将用来存放发送数据的描述符。cur前面是都已经用过的描述符，后面是可用的描述符

	int tx;				//tx指向第一个已存放数据但未被底层发送即未发送完成的描述符，或者等于cur即指向未发送完的描述符
					//tx前面是已经完全发送完成的，后面是驱动已经发送但底层还未确定发送完成的

	int rx;				//rx指向第一个还未处理的接收数据描述符，配合rx_offset使用，即向后至处为底层已接收待驱动接收
					//接收的数据，前方为已被驱动接收的数据
	

	struct napi_struct napi;	

	void *	    dma_tx_buff[DESC_NUM];		//一致性DMA缓冲区，用于存放即将发送的内核上层数据包（虚拟地址）
	dma_addr_t  dma_tx_addr[DESC_NUM];		//同上，但为总线地址

	void *	    dma_rx_buff[DESC_NUM];		//一致性DMA缓冲区，用于存放接收到的来自底层的数据包（虚拟地址）
	dma_addr_t  dma_rx_addr[DESC_NUM];		//同上，但为总线地址
	
	struct sk_buff *dma_skb[DESC_NUM];		//用于记录并标志skb是否已经发送完成（未完成为skb地址，完成后置为NULL）
	unsigned char *dma_skbopt[DESC_NUM];		//用于存放填充后新得到的帧（未填充时为NULL）
	
	unsigned char status;				//存放中断状态
	struct net_device_stats stats;			//网络设备状态结构体，存放多种因子，不过似乎没有初始化 ???
	struct pci_dev *pdev;				//指向pci设备
	
	void *pci_address;				//貌似有问题，没初始化过再释放网络设备时却对其做判断是否为0  ？？？
	dma_addr_t  dma_address;			//结合上个变量使用，仅在释放地址时用作释放函数的第三个参数，基本没用
};
	
	
	
	

