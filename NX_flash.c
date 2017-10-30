
/*********************************************************************************
*Flash (Data Record Management) driver                             					                                                          *
*Version: 1.0.0                                                                                                                 *
*2007/07/16 writed by Bulla Liu                       
*(C) Copyright 2007-2009, Emeson NetworkPower China Inc.     
*
*   
*********************************************************************************/   
#include <linux/config.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <asm/param.h>
//#include <linux/malloc.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/vmalloc.h>

#include <linux/signal.h>
#include <asm/bitops.h>
#include <linux/ctype.h>
//#include <asm/irq.h>

#include <asm/ptrace.h>
#include <linux/fs.h>
#include <linux/wrapper.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>


#ifdef MODULE
#include <linux/module.h>  
#else
#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT
#endif

#include "NX_flash.h"
#include "NX_record.h"

/*==================================================*/
/*=========����Ϊ�����豸ͨ�ö��岿��===============*/
/*==================================================*/

#define DEVICE_NAME "dataflash"
#define	DEVICE_MAJOR	166			/* Reserverd for "data flash" */

//#define DEBUG 1						/*����ģʽ�����Դ�ӡһЩ������Ϣ*/


/* Function Prototypes */
static int dataflash_device_init (void) __init;
static int dataflash_device_release(struct inode *, struct file *);
static int dataflash_device_open(struct inode *, struct file *);
static ssize_t dataflash_device_read(struct file *,char *,size_t,loff_t *);
static ssize_t dataflash_device_write(struct file *file,char *buf,size_t count,loff_t *ppos);
static int dataflash_device_ioctl(struct inode *inode, struct file *filp,unsigned int cmd, unsigned long arg);

static int Device_Open=0;

static struct file_operations Fops_dataflash = 
{	
    owner:		THIS_MODULE,
    open:		dataflash_device_open,	
    release:	dataflash_device_release,
    read:		dataflash_device_read,
    write:		dataflash_device_write,
    ioctl:		dataflash_device_ioctl,
};	

/*==================================================*/
/*=========����Ϊ�����豸ר�ö��岿��==============*/
/*==================================================*/
static char * dataflash_device_driver_ver="Emerson NX data flash driver version: 1.00\n";

//Flash���ϲ����ĵ�ַָ��
static volatile unsigned short * data_flash_base = NULL;

//Flash�Ļ���������Ϣ
static CFIDENT 	CFIident;
static CFIFRI 	CFIpri;
static CFISYS	CFIsys;

//Flash�Ĳ���״̬��Unknown, Reading, Programming, Block erasing, Chip erasing.
static volatile unsigned char ucFlashStatus = 0;

//Flash�Ĳ����ļ�����
static volatile unsigned int uiErase200msCnt = 0;
//��ǰ���ڲ�����Block���
static volatile unsigned char	ucBlockInErasing = 0xFF;

//Flash���ĸ������ڳ䵱��������.
static volatile unsigned int uiBasicBlock;
//Flash��ÿ��������������Flash�ռ��ƫ�Ƶ�ַ,ƫ������ڶ�������������. �ҵ�һ��Boot��������Ϊ��һ�������š�
static volatile unsigned int uiBlockOffsetAddr[200];
//Flash��ÿ���������ڴ�С,��λΪ��������������.
static volatile unsigned int BlockSizeList[200];



//����������, ��Flash���ڶ�ȡ״̬, �ᶨ������Dirty�������в���.
static struct timer_list erase_timer;
static void erase_state_poll(unsigned long period);

//��ȡFlashоƬ��CFI�ӿ�����
static void ReadCFI(void);

//FlashоƬ�ĵײ��������
static void EraseSuspend();
static void EraseResume();
static inline unsigned short ReadWord(unsigned int addr);
static void BlockErase(unsigned short No);
static void ChipErase(void);
static void RecordDataBlockErase(unsigned short uBlockSeclect);
static void Send_CMD(unsigned char cmd);
static unsigned char WriteWord(unsigned int addr, unsigned short data);

//FlashоƬ�Ļ�����Ϣ���ݽṹ����
static BASIC_FLASH_INFO_T	tBasicFlashInfo;

//FlashоƬ�м�¼��ͨ�����ݽṹ����
static FLASH_RECORD_T tFlashRecordW;

//FlashоƬ��¼�������ݽṹ����
static RECORD_MANAGE_T tRecordManageParameterp[INV_FAULT_RECORD+1];

//��Flash��д��һ��ָ���ļ�¼����
static unsigned char ucWriteOneRecord(FLASH_RECORD_T *pFlashRecord);

//��Flash�ж���һ��ָ���ļ�¼����
static unsigned char ucReadOneRecord(FLASH_RECORD_T *pFlashRecord);

//ˢ��Flash�Ļ�����Ϣ, ���û�н���������Ϣ��¼,�����½���
static unsigned char ucUpdateFlashBasicInformation(void);

//����Flash�и�����¼����״, ˢ��Flash���������ݽṹ
static unsigned char ucInitialRecordManagement(void);

//��Flash�ж�ȡһ����¼ָ����ŵļ�¼����
static unsigned char ucReadOneRecordFromFlash(FLASH_RECORD_T *pFlashRecord);

//��Flash��д��һ���¼�¼����
static unsigned char ucWriteOneNewRecordToFlash(FLASH_RECORD_T * pRecord);




static FLASH_RECORD_T tFlashRecordR;
static FLASH_RECORD_T tRecordForTest;

static void vWriteAEventRecordForTest();
static void vReadAEventRecordForTest();


//static unsigned char ucReadOneFaultRecordFromFlash(FLASH_RECORD_T *pFlashRecord);
//static unsigned char ucWriteOneFaultRecordToFlash(FLASH_RECORD_T * pRecord);
static void vWriteAFaultRecordForTest();
static void vReadAFaultRecordForTest();

static void vPrintAllValidDataOnOneBlock(unsigned char ucBlock);
static void vPrintAllValidRecordOnOneBlock(unsigned char ucBlock);
static void vWriteABlockRecordForTest(unsigned char ucRecordType);
static void vReadABlockRecordForTest(unsigned char ucRecordType);



int Debug = 1;

MODULE_PARM(Debug,"i");

/* Initialize the driver - Register the character device */
static int __init dataflash_device_init (void)
{
	int ret;	
	int i;	
	int x,y;
	struct timeval tv1,tv2;
	unsigned long tCurrentTime;
	
	unsigned long flags;	
	
	
	
	/* Register the character device */
	if ((ret = register_chrdev(DEVICE_MAJOR, DEVICE_NAME, &Fops_dataflash)) < 0) 
	{	
		printk ("dataflash Device failed with %d\n", ret);
		return (ret);
	}
	
	PRINT_TRACE(Debug)("%s the major is %d.\n",dataflash_device_driver_ver,DEVICE_MAJOR);	

	data_flash_base = (volatile unsigned short *)ioremap(DATA_FLASH_BASE,0x800000);
	
	ReadCFI();
	
	Send_CMD(CMD_RESET_DATA);	
	//ChipErase();	
	
	
	ret = ucUpdateFlashBasicInformation();
	ret = ucInitialRecordManagement();
	//for test
	printk ("--flash:record block1= %d,block2=%d---\n", tRecordManageParameterp[EVENT_RECORD].tUsedBlocks[0].ucBlockNum,
													tRecordManageParameterp[EVENT_RECORD].tUsedBlocks[1].ucBlockNum);
	
	
	del_timer_sync(&erase_timer);
	init_timer(&erase_timer);
	
	erase_timer.function = erase_state_poll;               
    erase_timer.data = ERASE_CHECK_PERIOD;    			/*check state per 100ms*/
    erase_timer.expires = jiffies + erase_timer.data;
    
    add_timer(&erase_timer);
	
	return (0);
}



static int dataflash_device_open (struct inode *inode, struct file *file)
{
	int num = MAJOR(inode->i_rdev);
	int type = MINOR(inode->i_rdev);

	printk ("flash device_open(%p,%p)\n", inode, file);
	printk("dataflash device major is %d\n",num);
	printk("dataflash device minor is %d\n",type);

	if (Device_Open>0)
	{
		printk("dataflash Device is busy!\n");
		return -EBUSY;
	}
	
	MOD_INC_USE_COUNT;

	Device_Open++;

	return 0;
}


static int dataflash_device_release (struct inode *inode, struct file *file)
{
 	printk ("flash device_release(%p,%p)\n", inode, file);

	MOD_DEC_USE_COUNT;
	
	Device_Open--;

	return 0;
}


static ssize_t dataflash_device_read (struct file *file,char *buf, size_t count, loff_t *ppos)
{	
	int rc=0;
	unsigned char i;	
	
	if (count!=sizeof(FLASH_RECORD_T))
	{
		printk("dataflash Read Frame size is error!\n");
		return ERROR_READ_SYSCALL_COUNT;
	}

	rc = verify_area(VERIFY_WRITE,buf,count);

	if (rc == -EFAULT)
	{
		printk("dataflash driver read syscall verify area fail\n");
		return ERROR_READ_SYSCALL_VERIRY;
	}		
	
	copy_from_user((char *)(&tFlashRecordR), buf, count);
	
	PRINT_TRACE(Debug)("dataflash read syscall entry ---- %d\n", tFlashRecordR.usID);
		
	//��ʼ��ȡFlash�еļ�¼
	rc = ucReadOneRecord(&tFlashRecordR);	
	
	if (rc == 0x00)
	{
		copy_to_user((char *)buf,(char *)(&tFlashRecordR),count);
	}
	
	return rc;
}	

static ssize_t dataflash_device_write(struct file *file,char *buf,size_t count,loff_t *ppos)
{
	int rc=0;
	unsigned char ret, i;
	
	if (count!=sizeof(FLASH_RECORD_T))
	{
		printk("dataflash Write Frame size is error!\n");
		return (ERROR_WRITE_SYSCALL_COUNT);
	}	
	
	rc = verify_area(VERIFY_WRITE,buf,count);

	if (rc == -EFAULT)
	{
		printk("dataflash Write read syscall verify area fail\n");
		return ERROR_WRITE_SYSCALL_VERIRY;
	}		
	
	copy_from_user((char *)(&tFlashRecordW), buf, count);
		
	ret = ucWriteOneRecord(&tFlashRecordW);
	
	return(ret);
}

static int dataflash_device_ioctl(struct inode *inode, struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	int rc = 0;
		
	/*��������������Ѷ����*/
	if ((cmd != CMD_SECTOR_ERASE_UNLOCK_DATA) && (cmd != CMD_CHIP_ERASE_UNLOCK_DATA)
		&& (cmd != CMD_GET_FLASH_OPRATION_STATE) && (cmd != CMD_EVENT_RECORD_ERASE))
	{
		printk("dataflash_device_ioctl() Error: no recognized cmd value!\n");	
		rc = -EINVAL;
		return(ERROR_IOCTL_SYSCALL_CMD); 
	}
	
	
	/*��������Ĳ���Ҳ�������Ѿ��������*/
	if (cmd == CMD_SECTOR_ERASE_UNLOCK_DATA)
	{
		if ((arg > tBasicFlashInfo.usTotalBlocks)||(arg == tBasicFlashInfo.usTotalBlocks))
		{
			printk("dataflash_device_ioctl() Error: no recognized arg value!  --- %d\n",arg);	
			rc = -EINVAL;
			return(ERROR_IOCTL_SYSCALL_ARG); 
		}
	}
	
	//FLASH state
	if (cmd == CMD_GET_FLASH_OPRATION_STATE)
	{
		return ucFlashStatus;
	}
	
	if (ucFlashStatus != READING)
	{
		printk("dataflash_device_ioctl() Error: The chip is not in Reading state -- %d\n", ucFlashStatus);
		rc = -EINVAL;
		return(ERROR_IOCTL_SYSCALL_BUSY);
	}
	
	//�����
	if (cmd == CMD_SECTOR_ERASE_UNLOCK_DATA)
	{
		BlockErase(arg);		
	}
	
	//Ƭ����
	if (cmd == CMD_CHIP_ERASE_UNLOCK_DATA)
	{
		ChipErase();		
	}

	//erase event record
	if (cmd == CMD_EVENT_RECORD_ERASE)
	{
		RecordDataBlockErase(arg);	
	}
	
	
	return (rc);
}


/*
 * Cleanup - unregister the driver
 */
static void  __exit uninst_dataflash_device(void)
{
	int minor, ret;	
	
	del_timer_sync(&erase_timer);
	
	/* Unregister the device */
	ret = unregister_chrdev (DEVICE_MAJOR, DEVICE_NAME);
	
	iounmap((unsigned char *)data_flash_base);	

	/* If there's an error, report it */
	if (ret < 0) 
	{
		printk ("unregister_chrdev: error %d\n", ret);
		return;
	}
	
	PRINT_TRACE(Debug)("dataflash driver succesfully uninstalled\n");
}



/****************************************************/
/*---�� Flash �����ĵײ����������------*/
/****************************************************/
static unsigned int uiPower(unsigned char base, unsigned short factor)
{
	unsigned short i;
	unsigned int result;
	
	result = 1;
	
	for (i=0; i<factor; i++)
	{
		result = result * base;	
	}	
	
	return(result);
}

/****************************************************/
/*��������:erase_state_poll							*/
/*��������:Erase process counter					*/	
/*�������:											*/
/*			period:									*/
/*�������:	N/A										*/
/*�޸ļ�¼:											*/
/****************************************************/
static int testvar = 0;

static void erase_state_poll(unsigned long period)
{
	unsigned char i,j;	
	unsigned short status1, status2, ret;
	unsigned char ucSize;
	unsigned int uiStartAddressOfFlag;
	
	ucBlockInErasing = 0xFF;	
		
	if ((ucFlashStatus == BLOCKERASING) || (ucFlashStatus == CHIPERASING))
	{
		status1 = * data_flash_base;	/*��ȡһ��״̬�Ĵ���*/
		status2 = * data_flash_base;	/*��ȡһ��״̬�Ĵ���*/
						
		if ((status1 & 0x40) == (status2 & 0x40)) 
       	{
       		//��������
       		ucFlashStatus = READING;					//Register a reading status
       		uiErase200msCnt = 0;
       		PRINT_TRACE(Debug)("Erase is finished\n");        
       		
       		//���������־
       		for (i=0; i<INV_FAULT_RECORD; i++)	
       		{
       			if (tRecordManageParameterp[i+1].tUsedBlocks[0].ucBlockStatus == BLOCK_ERASING)
       			{
       				tRecordManageParameterp[i+1].tUsedBlocks[0].ucBlockStatus = BLOCK_NO_DATA;
       				break;
       			}     
       			else if (tRecordManageParameterp[i+1].tUsedBlocks[1].ucBlockStatus == BLOCK_ERASING)
       			{
       				tRecordManageParameterp[i+1].tUsedBlocks[1].ucBlockStatus = BLOCK_NO_DATA;
       				break;
       			}       			  			
       		}  		    					    		
       	}
       	else if ((status1 & 0x0020) == 0x0020)
       	{
       		/*�쳣����*/
       		ucFlashStatus = READING;					/*Register a reading status*/
			uiErase200msCnt = 0;						/*stop a 100ms counter*/   		
       		
       		/*ǿ�ƻָ�����״̬*/
       		*(data_flash_base+0x55)=0xF0;
       		
       		//���������־
       		for (i=0; i<INV_FAULT_RECORD; i++)	
       		{
       			if (tRecordManageParameterp[i+1].tUsedBlocks[0].ucBlockStatus == BLOCK_ERASING)
       			{
       				tRecordManageParameterp[i+1].tUsedBlocks[0].ucBlockStatus = BLOCK_NO_DATA;
       				break;
       			}     
       			else if (tRecordManageParameterp[i+1].tUsedBlocks[1].ucBlockStatus == BLOCK_ERASING)
       			{
       				tRecordManageParameterp[i+1].tUsedBlocks[1].ucBlockStatus = BLOCK_NO_DATA;
       				break;
       			}       			  			
       		}  		   
       	}
       	else
       	{
       		uiErase200msCnt ++;
       		
       		if (ucFlashStatus == CHIPERASING)	
       		{
       			PRINT_TRACE(Debug)("In progress of Chip erasing ---%d\n", uiErase200msCnt);
       		}
       	}						
	}
	else if (ucFlashStatus == READING)					//Flash ���ڶ�״̬ʱ��������Dirty״̬��������д
	{			
		//�·���������һ��Dirty�������в���
		for (i=0; i<INV_FAULT_RECORD; i++)		
		{
			if (tRecordManageParameterp[i+1].tUsedBlocks[0].ucBlockStatus == BLOCK_IS_DIRTY)
			{
				ucBlockInErasing = tRecordManageParameterp[i+1].tUsedBlocks[0].ucBlockNum;					
				BlockErase(ucBlockInErasing);	//�ҵ�һ��Dirty�������Ϳ�ʼ����
					
				tRecordManageParameterp[i+1].tUsedBlocks[0].ucBlockStatus = BLOCK_ERASING;
				
				PRINT_TRACE(Debug)("******The %d block will be erased \n", ucBlockInErasing);				
				return;
			}
			else if (tRecordManageParameterp[i+1].tUsedBlocks[1].ucBlockStatus == BLOCK_IS_DIRTY)
			{
				ucBlockInErasing = tRecordManageParameterp[i+1].tUsedBlocks[1].ucBlockNum;					
				BlockErase(ucBlockInErasing);	//�ҵ�һ��Dirty�������Ϳ�ʼ����
					
				tRecordManageParameterp[i+1].tUsedBlocks[1].ucBlockStatus = BLOCK_ERASING;
				
				PRINT_TRACE(Debug)("******The %d block will be erased \n", ucBlockInErasing);
				return;
			}
		}
	}
	
	//û���ҵ�һ��Dirty����������û�ڶ�ȡ״̬�£��������´ε���Ϊ 200ms �Ժ�
	
	erase_timer.data = ERASE_CHECK_PERIOD;    
    erase_timer.expires = jiffies + erase_timer.data;
    
    add_timer(&erase_timer);
			
}

/****************************************************/
/*��������:ReadCFI									*/
/*��������:��ȡflash�Ĺ���flash�ӿ�				 	*/	
/*�������:	N/A										*/
/*													*/
/*�������:	N/A										*/
/*�޸ļ�¼:											*/
/****************************************************/
static void ReadCFI(void)
{
	unsigned char i,j,k;
	unsigned int FlashSize;
	unsigned short *p, number, blocksize;
	
	/*    ��ȡCFI ��Ϣ */
	
	/* 1, ��д���������*/
    *(data_flash_base + ADDR_CFI_READ) = CMD_CFI_READ;
    
    /* 2, ��ȡCFI����*/
    CFIident.qry[0] = *(data_flash_base + 0x10);  
    CFIident.qry[1] = *(data_flash_base + 0x11); 
    CFIident.qry[2] = *(data_flash_base + 0x12); 
    
    if ((CFIident.qry[0] == 'Q') || (CFIident.qry[1] == 'R') ||(CFIident.qry[2] == 'Y'))
    {
    	/*������ȡCFI identify*/
	    CFIident.P_ID = *(data_flash_base + 0x13); 
	    CFIident.P_ADR = *(data_flash_base + 0x15);
	    
	    CFIident.DevSize = *(data_flash_base + 0x27);
	    
	    CFIident.InterfaceDesc = *(data_flash_base + 0x28);
	    
	    CFIident.MaxBufWriteSize = *(data_flash_base + 0x2A);
	    
	    CFIident.NumEraseRegions = *(data_flash_base + 0x2C);
	    CFIident.NumFirstRegion = ((*(data_flash_base + 0x2E))<<8) + (*(data_flash_base + 0x2D));
	    CFIident.SizeFirstRegion = ((*(data_flash_base + 0x30))<<8) + (*(data_flash_base + 0x2F));
	    CFIident.NumSecondRegion = ((*(data_flash_base + 0x32))<<8) + (*(data_flash_base + 0x31));
	    CFIident.SizeSecondRegion = ((*(data_flash_base + 0x34))<<8) + (*(data_flash_base + 0x33));
	    CFIident.NumThirdRegion = ((*(data_flash_base + 0x36))<<8) + (*(data_flash_base + 0x35));
	    CFIident.SizeThirdRegion = ((*(data_flash_base + 0x38))<<8) + (*(data_flash_base + 0x37));
	    CFIident.NumFouthRegion = ((*(data_flash_base + 0x3a))<<8) + (*(data_flash_base + 0x39));
	    CFIident.SizeFouthRegion = ((*(data_flash_base + 0x3c))<<8) + (*(data_flash_base + 0x3b));
	    
    
   		/*������ȡCFI system*/
    	CFIsys.TyProgram = *(data_flash_base + 0x1F); 
    	CFIsys.TyBlkErase = *(data_flash_base + 0x21); 
    	CFIsys.TyChipErase = *(data_flash_base + 0x22); 
    	CFIsys.MaxProgrom = *(data_flash_base + 0x23); 
    	CFIsys.MaxBlkErase = *(data_flash_base + 0x25); 
    	CFIsys.MaxChipErase = *(data_flash_base + 0x26); 
    	
    	/*������ȡCFI pri*/
    	CFIpri.pri[0] = *(data_flash_base + 0x40); 
    	CFIpri.pri[1] = *(data_flash_base + 0x41); 
    	CFIpri.pri[2] = *(data_flash_base + 0x42); 
    	
    	CFIpri.MajorVersion = *(data_flash_base + 0x43); 
    	CFIpri.MinorVersion = *(data_flash_base + 0x44); 
    	
    	CFIpri.EraseSuspend = *(data_flash_base + 0x46); 
    	CFIpri.TopBottomType = *(data_flash_base + 0x4F); 
    	
    	////////////////////////////////////
    	//��ӡFlash�Ļ�����Ϣ
    	PRINT_TRACE(Debug)(" ========================================= \n");	
    	PRINT_TRACE(Debug)(" Flash Information (Support CFI interface) \n");
    	PRINT_TRACE(Debug)(" ========================================= \n");	
    	PRINT_TRACE(Debug)(" 1, Algorithm type:  %d                    \n", CFIident.P_ID);
    	PRINT_TRACE(Debug)(" 2, Flash Bus type:  %d (0:8 1:16 2:8,16)  \n", CFIident.InterfaceDesc);
    	PRINT_TRACE(Debug)(" 3, Flash Size    :  %d   %d bytes         \n", CFIident.DevSize,uiPower(2,CFIident.DevSize));
    	PRINT_TRACE(Debug)(" 4, Flsh Boot type:  %d (0:None 2:B 3:T)   \n", CFIpri.TopBottomType);
    	PRINT_TRACE(Debug)(" 5, Flash Erase Sus: %d (0:None 1:OR 2:RW) \n", CFIpri.EraseSuspend);
    		
    	PRINT_TRACE(Debug)(" 6, Block Regions:   %d 				   \n", CFIident.NumEraseRegions);    		
    	PRINT_TRACE(Debug)(" 7, 1nd Region Num:  %d 				   \n", CFIident.NumFirstRegion+1);
    	PRINT_TRACE(Debug)(" 8, 1nd Region Size: %d 				   \n", CFIident.SizeFirstRegion * 256);
    	PRINT_TRACE(Debug)(" 9, 2nd Region Num:  %d 				   \n", CFIident.NumSecondRegion+1);
    	PRINT_TRACE(Debug)(" a, 2nd Region Size: %d 				   \n", CFIident.SizeSecondRegion * 256);
    	PRINT_TRACE(Debug)(" b, 3nd Region Num:  %d 				   \n", CFIident.NumThirdRegion+1);
    	PRINT_TRACE(Debug)(" c, 3nd Region Size: %d 				   \n", CFIident.SizeThirdRegion * 256);
    	PRINT_TRACE(Debug)(" d, 4nd Region Num:  %d 				   \n", CFIident.NumFouthRegion+1);
    	PRINT_TRACE(Debug)(" e, 4nd Region Size: %d 				   \n", CFIident.SizeFouthRegion * 256);
    		
    	PRINT_TRACE(Debug)(" f, Type Prg time:   %d   %d us			   \n", CFIsys.TyProgram,uiPower(2,CFIsys.TyProgram));
    	PRINT_TRACE(Debug)(" g, Type B Er time:  %d   %d ms			   \n", CFIsys.TyBlkErase,uiPower(2,CFIsys.TyBlkErase));
    	PRINT_TRACE(Debug)(" h, Type C Er time:  %d   %d ms		       \n", CFIsys.TyChipErase,uiPower(2,CFIsys.TyChipErase));
    		
    	PRINT_TRACE(Debug)(" i, Max Prg time:    %d   %d us			   \n", CFIsys.MaxProgrom,uiPower(2,CFIsys.TyProgram) * uiPower(2,CFIsys.MaxProgrom));
    	PRINT_TRACE(Debug)(" j, Max B Er time:   %d   %d ms			   \n", CFIsys.MaxBlkErase,uiPower(2,CFIsys.TyBlkErase) * uiPower(2,CFIsys.MaxBlkErase));
    	PRINT_TRACE(Debug)(" k, Max C Er time:   %d   %d ms			   \n", CFIsys.MaxChipErase,uiPower(2,CFIsys.MaxChipErase));	
    		
    	PRINT_TRACE(Debug)(" ========================================= \n");
    	   	   
    	
    	/***************************************************/
    	/*���ÿ��Block����ʼ��ַ  --- TBD*/
    	/***************************************************/
    	for (i=0; i<200; i++)
    	{
    		BlockSizeList[i] = 0;
    		uiBlockOffsetAddr[i] = 0;
    	}
    	
    	p = (unsigned short *)(&CFIident.NumFirstRegion);
    	k = 0; 
    	
    	for (i=0; i<4; i++)
    	{
    		number = *(p + 2*i) + 1;
    		blocksize = *(p + 2*i +1);
    		
    		for (j=0; j<number; j++)
    		{
    			BlockSizeList[k++] = 256 * blocksize;	//��λΪshort int��
    		}	
    	}
    	
    	FlashSize = uiPower(2,CFIident.DevSize);
    	
    	if (CFIpri.TopBottomType == BOTTOM_FLASH)
    	{    			
    		uiBlockOffsetAddr[0] = 0x00;
    		for (i=1; i<200; i++)
    		{
    			uiBlockOffsetAddr[i] = uiBlockOffsetAddr[i-1] + BlockSizeList[i-1];
    		}
    	}
    	else if (CFIpri.TopBottomType == TOP_FLASH)
		{
			uiBlockOffsetAddr[0] = FlashSize - BlockSizeList[0];
			for (i=1; i<200; i++)
    		{
    			uiBlockOffsetAddr[i] = uiBlockOffsetAddr[i-1] - BlockSizeList[i];
    		}
		}
		
		//Ҫ��������������ĵ�λ������INT16
		for (i=0; i<200; i++)
    	{
    		BlockSizeList[i] = (BlockSizeList[i] >> 1);
    		uiBlockOffsetAddr[i] = (uiBlockOffsetAddr[i] >> 1);
    	}
		
		for (i=0; i<200; i++)
		{
			PRINT_TRACE(Debug)(" %d block offset is %d\n",i,uiBlockOffsetAddr[i]);    
		}    	
    	
    	Send_CMD(CMD_RESET_DATA);    	
    	
    	//Flash״̬���ڶ�
    	ucFlashStatus = READING;
    }
    else
    {
    	PRINT_TRACE(Debug)("This flash is not a CFI standard device.\n");
    }
}

/****************************************************/
/*��������:Send_CMD									*/
/*��������:��flash�ӿڷ���ָ��������			 	*/	
/*�������:	cmd										*/
/*													*/
/*�������:	N/A										*/
/*�޸ļ�¼:											*/
/****************************************************/
static void Send_CMD(unsigned char cmd)
{
	switch (cmd)
	{
		case CMD_RESET_DATA:
		{
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_UNLOCK_DATA_1;
			*(data_flash_base + ADDR_UNLOCK_2) = CMD_UNLOCK_DATA_2;	
				
			*(data_flash_base + ADDR_UNLOCK_2) = CMD_RESET_DATA;	
			
			break;		
		}
		case CMD_MANUFACTURER_UNLOCK_DATA:
		{
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_UNLOCK_DATA_1;
			*(data_flash_base + ADDR_UNLOCK_2) = CMD_UNLOCK_DATA_2;	
			
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_MANUFACTURER_UNLOCK_DATA;	
			
			break;		
		}
		case CMD_PROGRAM_UNLOCK_DATA:
		{
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_UNLOCK_DATA_1;
			*(data_flash_base + ADDR_UNLOCK_2) = CMD_UNLOCK_DATA_2;	
			
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_PROGRAM_UNLOCK_DATA;	
			
			break;		
		}
		case CMD_SECTOR_ERASE_UNLOCK_DATA:
		{
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_UNLOCK_DATA_1;
			*(data_flash_base + ADDR_UNLOCK_2) = CMD_UNLOCK_DATA_2;					
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_CHIP_SECTOR_ERASE;	
			
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_UNLOCK_DATA_1;
			*(data_flash_base + ADDR_UNLOCK_2) = CMD_UNLOCK_DATA_2;				
			
			break;		
		}
		case CMD_CHIP_ERASE_UNLOCK_DATA:
		{
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_UNLOCK_DATA_1;
			*(data_flash_base + ADDR_UNLOCK_2) = CMD_UNLOCK_DATA_2;					
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_CHIP_SECTOR_ERASE;	
			
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_UNLOCK_DATA_1;
			*(data_flash_base + ADDR_UNLOCK_2) = CMD_UNLOCK_DATA_2;	
			*(data_flash_base + ADDR_UNLOCK_1) = CMD_CHIP_ERASE_UNLOCK_DATA;					
			
			break;		
		}
		default:
			printk("Unvalid Flash CMD\n");
			break;		
	}	
}

/****************************************************/
/*��������:ChipErase								*/
/*��������:Erase the entire chip	 				*/	
/*�������:	N/A										*/
/*													*/
/*�������:	N/A										*/
/*�޸ļ�¼:											*/
/****************************************************/
static void ChipErase(void)
{
	unsigned char ret;		
	
	Send_CMD(CMD_CHIP_ERASE_UNLOCK_DATA);	/*Send a chip erase command*/
	ucFlashStatus = CHIPERASING;			/*Register a chiperaseing status*/
	uiErase200msCnt = 1;					/*Start a 100ms counter*/
	
	del_timer_sync(&erase_timer);
	init_timer(&erase_timer);
	
	erase_timer.function = erase_state_poll;               
    erase_timer.data = ERASE_CHECK_PERIOD;    			/*check state per 100ms*/
    erase_timer.expires = jiffies + erase_timer.data;
    
    add_timer(&erase_timer);
	
	return;
}

/****************************************************/
/*��������:BlockErase								*/
/*��������:Erase the designated block 				*/	
/*�������:											*/
/*			No:		the Block Number				*/
/*�������:	N/A										*/
/*�޸ļ�¼:											*/
/****************************************************/
static void BlockErase(unsigned short No)
{
	unsigned char ret;	
	//unsigned short TypeBlockEraseTime;	
		
	Send_CMD(CMD_RESET_DATA);						/*Send a reset common command*/
	
	Send_CMD(CMD_SECTOR_ERASE_UNLOCK_DATA);		/*Send a block erase common command*/
	*(data_flash_base + uiBlockOffsetAddr[No]) = CMD_SECTOR_ERASE_UNLOCK_DATA;	
	
	ucFlashStatus = BLOCKERASING;				/*Register a chiperaseing status*/
	uiErase200msCnt = 1;						/*Start a 100ms counter*/
	
	//TypeBlockEraseTime = uiPower(2,CFIsys.TyBlkErase);
	
	del_timer_sync(&erase_timer);
	init_timer(&erase_timer);
	
	erase_timer.function = erase_state_poll;               
   	erase_timer.data = ERASE_CHECK_PERIOD;    
    	erase_timer.expires = jiffies + erase_timer.data;
    
    	add_timer(&erase_timer);
	
	return;
}


/****************************************************/
/*��������:RecordDataBlockErase								*/
/*��������:Erase the record block 				*/	
/*�������:											*/
/*			No:		the Block Number				*/
/*�������:	N/A										*/
/*�޸ļ�¼:											*/
/****************************************************/
static void RecordDataBlockErase(unsigned short uBlockSeclect)
{
	if (uBlockSeclect>1)
	{
		uBlockSeclect = 0;
	}
	unsigned char ucBlockWillErasing = tRecordManageParameterp[EVENT_RECORD].tUsedBlocks[uBlockSeclect].ucBlockNum;					
	//for test
	printk ("--flash:will erase block= %d--\n", ucBlockWillErasing);
	BlockErase(ucBlockWillErasing);
	return;
}

/****************************************************/
/*��������:EraseSuspend								*/
/*��������:Erase process is pending	 				*/	
/*�������:	N/A										*/
/*�������:	N/A										*/
/*�޸ļ�¼:											*/
/****************************************************/
static void EraseSuspend(void)
{
	unsigned char ret;	
	
	if ((ucFlashStatus == BLOCKERASING) || (ucFlashStatus == CHIPERASING))
	{		
		*(data_flash_base + ADDR_UNLOCK_1) = CMD_ERASE_SUSPEND;	
		
		ucFlashStatus = ERASEPENDING;				/*Register a erase pending status*/		
		
		/*ֹͣ�������*/
		del_timer_sync(&erase_timer);
		
	}
	else
	{
		PRINT_TRACE(Debug)("Flash is not in erase process\n");	
	}
	
	return;
}

/****************************************************/
/*��������:EraseResume								*/
/*��������:Erase process to be continued			*/	
/*�������:	N/A										*/
/*�������:	N/A										*/
/*�޸ļ�¼:											*/
/****************************************************/
static void EraseResume(void)
{
	unsigned char ret;	
	
	if (ucFlashStatus == ERASEPENDING)
	{		
		*(data_flash_base + ADDR_UNLOCK_1) = CMD_ERASE_RESUME;	
		
		ucFlashStatus = BLOCKERASING;				/*Register a block erase status*/	
		
		/*�����������*/
		init_timer(&erase_timer);
	
		erase_timer.function = erase_state_poll;               
	    erase_timer.data = ERASE_CHECK_PERIOD;    
	    erase_timer.expires = jiffies + erase_timer.data;
	    
	    add_timer(&erase_timer);	
	}
	else
	{
		PRINT_TRACE(Debug)("Flash is not in erase pending process\n");	
	}
	
	return;
}

/****************************************************/
/*��������:ReadByte   								*/
/*��������:Read a bytes from Flash					*/	
/*�������:	   										*/
/*				addr: start address of flash		*/
/*�������:	   										*/
/*				value: result of this byte			*/
/*�޸ļ�¼:											*/
/****************************************************/
static unsigned char ReadByte(unsigned int addr)
{	
	if ((addr & 0x01) == 0x01)
	{
		return ((*(data_flash_base + (addr >> 1))) >> 8) ;
	}
	else
	{
		return ((*(data_flash_base + (addr >> 1))) & 0xFF) ;
	}
}

/****************************************************/
/*��������:ReadWord   								*/
/*��������:Read a short int data from Flash			*/	
/*�������:	   										*/
/*				addr: the offset of flash addr 		*/
/*					  per a short int type			*/
/*�������:	   										*/
/*				value: result of this short int		*/
/*�޸ļ�¼:											*/
/****************************************************/
static inline unsigned short ReadWord(unsigned int addr)
{		
	return *(data_flash_base + addr);
}

/****************************************************/
/*��������:WriteWord   								*/
/*��������:Write a short int data from Flash		*/	
/*�������:	   										*/
/*				addr: the offset of flash addr 		*/
/*					  per a short int type			*/
/*				data: the data to be writen			*/
/*�������:	   										*/
/*				result: successful or fail			*/
/*�޸ļ�¼:											*/
/****************************************************/
static unsigned char WriteWord(unsigned int addr, unsigned short data)
{		
	unsigned short status1, status2;
	unsigned char i, Maxtimes, TypeT, result;
//	unsigned char ucFlashStatusBak;
	
//	printk("The addr is %d , data is %d\n", addr, data);
	
//	ucFlashStatusBak = ucFlashStatus;
	
//	ucFlashStatus = PROGRAMING;				/*Register a programming status*/	
	
	/*���ͱ������*/
	*(data_flash_base + ADDR_UNLOCK_1) = CMD_UNLOCK_DATA_1;
	*(data_flash_base + ADDR_UNLOCK_2) = CMD_UNLOCK_DATA_2;	
			
	*(data_flash_base + ADDR_UNLOCK_1) = CMD_PROGRAM_UNLOCK_DATA;	
	*(data_flash_base + addr) = data;	
	
	/*������͵ı��ʱ���Լ����ȴ�ѭ��*/
	TypeT = uiPower(2,CFIsys.TyProgram);
	Maxtimes = uiPower(2,CFIsys.MaxProgrom);
	udelay(TypeT);
	
	/*�״ζ�����ر�־�Ĵ���*/
	status1 = *(data_flash_base + addr);		
	
	/*��ʼ����̽����־Ϊ��ʱ*/
	result = 3;
	
	for (i=0; i<Maxtimes; i++)	
	{
		status2 = status1;
		status1 = *(data_flash_base + addr);		
		
		if ((status1 & 0x40) == (status2 & 0x40)) 
       	{
       		result = 0;						/*��������*/
       		break;
       	}
       	if ((status1 & 0x0020) == 0x0020)
       	{
       		result = 1;       				/*�쳣����*/
       		
       		/*ǿ�ƻָ�����״̬*/
       		*(data_flash_base+0x55)=0xF0;	
       		
       		break;
       	}
       	
		udelay(TypeT);			
	}
	
	if (result == 0)
	{
		if (status1 != data)
		{
			result = 2;						/*��̴���*/			
		}
	}
	else if (result == 3)
	{
		/*ǿ�ƻָ�����״̬*/
		*(data_flash_base+0x55)=0xF0;
	}
	
//	ucFlashStatus = ucFlashStatusBak;				/*feed back the original status*/	
//	printk("result is %d,  i is %d\n", result, i);
	
	return (result);
}

/********************************************************************/
/*��������:ucWriteDataToFlash										*/
/*��������:��Flash��ָ��λ��д��һ������							*/
/*�������:	   														*/
/*				uiAddress:  ��ʼ��ַ								*/
/*				pData: 	    ���ݴ洢ָ��							*/
/*				usSize: 	���С									*/
/*�������:	   														*/
/*				result: successful or fail							*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucWriteDataToFlash(unsigned int uiAddress, unsigned short * pData, unsigned short usSize)
{
	unsigned short i, usWriteData, usReadData;
	unsigned int uiStartAddress;	
	unsigned char ret;
	
	
	uiStartAddress = uiAddress;
	
	//�������ɺ���ǰ��˳��д��Flash��
	for (i=usSize; i>0; i--)
	{
		usWriteData = *(pData + (i-1));			//ȡ����¼�е�һ����Ԫ����������
		usReadData = ReadWord(uiStartAddress + (i-1));
		
		if (usWriteData != usReadData)		//��ͬ�����ݾͲ���Ҫ��Flashд��
		{
			ret = WriteWord(uiStartAddress + (i-1), usWriteData);			
		}
		else
		{
			ret = 0x00;	
		}
			
		//printk("+++++++the %d 's value is %d, ----%d\n", i, usWriteData, ret);
			
		//д�κ�һ����¼�е����ݴ��󣬾�ֱ���˳����μ�¼���ݵ�д����
		if (ret)	break;			
	}	
		
	return(ret);	
}

/********************************************************************/
/*��������:vReadDataFromFlash										*/
/*��������:��Flash��ָ��λ�ö�ȡָ����С��Flash���ݿ�				*/
/*�������:	   														*/
/*				uiAddress:  ��ʼ��ַ								*/
/*				pData: 	    ���ݴ洢ָ��							*/
/*				ucSize: 	���С									*/
/*�������:	   														*/
/*				NONE												*/
/*�޸ļ�¼:															*/
/********************************************************************/
static void vReadDataFromFlash(unsigned int uiAddress, unsigned short * pData, unsigned char ucSize)
{	
	unsigned char i;
	
	for (i=0; i<ucSize; i++)
	{
		*(pData + i) = ReadWord(uiAddress + i);
	}	
}

/****************************************************/
/*---�� Flash �������в����������(ƫ���¼�Ĺ���)------*/
/****************************************************/
/****************************************************/

/********************************************************************/
/*��������:ucQueryFlashBasicInformation								*/
/*��������:��ѯFlashоƬ��ǰ�����������Ƿ��л�����Ϣ��¼����		*/	
/*�������:	   														*/
/*				ucBlockNo: ����ѯ��������� 						*/
/*�������:	   														*/
/*				result: successful or fail							*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucQueryFlashBasicInformation(unsigned char ucBlockNo)
{
	unsigned short *p = (unsigned short *)&tBasicFlashInfo;
	unsigned short i, Size;
	unsigned char ret;
	
	/*Send a reset common command for read flash*/
	Send_CMD(CMD_RESET_DATA);						
	
	Size = sizeof(BLOCK_ATTR_T)/2;	
	Size = sizeof(tBasicFlashInfo)/2;
	
	PRINT_TRACE(Debug)("The size of tBasicFlashInfo is %d\n", Size);
	
	/*��֤Flash������Ϣ��ʶ�����Ƿ���ȷ*/
	if ((ReadWord(uiBlockOffsetAddr[ucBlockNo] + IDENTIFY1_ADDR) == FLASH_IDENTIFY1) && 
		(ReadWord(uiBlockOffsetAddr[ucBlockNo] + IDENTIFY2_ADDR) == FLASH_IDENTIFY2) &&
		(ReadWord(uiBlockOffsetAddr[ucBlockNo] + IDENTIFY3_ADDR) == FLASH_IDENTIFY3))
	{				
		for (i=0; i<Size; i++)
		{
			*(p + i) = ReadWord(uiBlockOffsetAddr[ucBlockNo] + IDENTIFY1_ADDR + i);				
			
			PRINT_TRACE(Debug)("The %d word of tBasicFlashInfo is %d\n",i, *(p + i));	
		}
		
		PRINT_TRACE(Debug)("Already read a basic flash information in %d block\n", ucBlockNo);
		
		ret = 0;		// normally read.
		
	}
	else
	{
		ret = 1;		// abnormal read.	
				
		PRINT_TRACE(Debug)("(ReadWord(uiBlockOffsetAddr[ucBlockNo] + IDENTIFY1_ADDR)= %d\n",ReadWord(uiBlockOffsetAddr[ucBlockNo] + IDENTIFY1_ADDR));
		PRINT_TRACE(Debug)("(ReadWord(uiBlockOffsetAddr[ucBlockNo] + IDENTIFY2_ADDR)= %d\n",ReadWord(uiBlockOffsetAddr[ucBlockNo] + IDENTIFY2_ADDR));
		PRINT_TRACE(Debug)("(ReadWord(uiBlockOffsetAddr[ucBlockNo] + IDENTIFY2_ADDR)= %d\n",ReadWord(uiBlockOffsetAddr[ucBlockNo] + IDENTIFY2_ADDR));
		
	}
	
	return(ret);
}


/********************************************************************/
/*��������:ucSetupFlashBasicInformation								*/
/*��������:��ָ���������н���������Ϣ��¼����						*/
/*�������:	   														*/
/*				ucBlockNo: ����ѯ��������� 						*/
/*�������:	   														*/
/*				result: successful or fail							*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucSetupFlashBasicInformation(unsigned char ucBlockNo)
{
	unsigned short *p = (unsigned short *)&tBasicFlashInfo;
	unsigned short i,Size,usFist64KBlock;
	unsigned char ret;
	
	/*********************************/
	/*���½���Flash������Ϣ*/	
	/*********************************/
		
	/*********************************/
	/*1, ����ȱʡ��Flash������Ϣ��� */
	/*********************************/
		
	/*1-0, ��������,Ȼ������Flash�Ĳ�����Ϣ����*/
	Size = sizeof(tBasicFlashInfo)/2;	
	for (i=0; i<Size; i++)
	{
		*(p + i) = NOT_USED;	
	}
	
	//�����ʶ��
	tBasicFlashInfo.usIdentifyInfo[0] = FLASH_IDENTIFY1;
	tBasicFlashInfo.usIdentifyInfo[1] = FLASH_IDENTIFY2;
	tBasicFlashInfo.usIdentifyInfo[2] = FLASH_IDENTIFY3;	
	
	//�������Flash����
	tBasicFlashInfo.usFlashCapacity = uiPower(2,CFIident.DevSize)/0x400; 	//Flash��С(����Kbytes)
	tBasicFlashInfo.usBusWidth = CFIident.InterfaceDesc;
	tBasicFlashInfo.usHasSuspend = CFIpri.EraseSuspend;
	
	//ͳ���ܹ��ж��ٸ�����
	for (i=0; i<MAX_BLOCKS; i++)
	{
		if (BlockSizeList[i] == 0x00)	break;
	}
	
	tBasicFlashInfo.usTotalBlocks = i;
	
	PRINT_TRACE(Debug)("tBasicFlashInfo.usTotalBlocks = %d\n", i);
	
	
	/*1-1, ����ȱʡ��Flash������Ϣ��������*/
	tBasicFlashInfo.tBlockAttrList[ucBlockNo].usBlockState = BLOCK_ACTIVE;
	tBasicFlashInfo.tBlockAttrList[ucBlockNo].usRecordType = FLASH_BASIC_INFO;
	tBasicFlashInfo.tBlockAttrList[ucBlockNo].usPages = BlockSizeList[ucBlockNo]/0x200;
	
	/*1-2, ����һ��������ʼ,���ҵ�һ��64Kbtyes(32K short int)���������*/	
	usFist64KBlock = 0x00;
	for (i=(ucBlockNo+1); i<MAX_BLOCKS; i++)
	{
		if (BlockSizeList[i] == 0x8000)	
		{
			usFist64KBlock = i;	
			PRINT_TRACE(Debug)("The first 64kbytes block number is %d\n", i);
			break;	
		}
	}
	
	/*���û�з���64Kbytes��С������, ����󷵻�*/
	if (usFist64KBlock == 0x00)		
	{
		ret = 0xFF;
		printk("Can not find any one block is 64kbyts\n");
		return (ret);	
	}	
	
	/*2-3, ��������ȱʡ�ļ�¼��������*/
	for (i=0;i<INV_FAULT_RECORD;i++)
	{		
		tBasicFlashInfo.tBlockAttrList[usFist64KBlock + 2*i].usBlockState = BLOCK_ACTIVE;
		tBasicFlashInfo.tBlockAttrList[usFist64KBlock + 2*i].usRecordType = i+1;
		tBasicFlashInfo.tBlockAttrList[usFist64KBlock + 2*i].usPages = BlockSizeList[usFist64KBlock + 2*i]/0x200;
		
		tBasicFlashInfo.tBlockAttrList[usFist64KBlock + 2*i+1].usBlockState = BLOCK_ACTIVE;
		tBasicFlashInfo.tBlockAttrList[usFist64KBlock + 2*i+1].usRecordType = i+1;
		tBasicFlashInfo.tBlockAttrList[usFist64KBlock + 2*i+1].usPages = BlockSizeList[usFist64KBlock + 2*i+1]/0x200;		
	}		
		
	/*3, ��ȱʡ��Flash������Ϣ���д�뵽Flash��*/		
	ret = ucWriteDataToFlash(uiBlockOffsetAddr[ucBlockNo], p, Size);
	
	return (ret);
}


/********************************************************************/
/*��������:ucUpdateFlashBasicInformation							*/
/*��������:����Ƿ���Flash������Ϣ���ݣ�û����������Ϣ			*/
/*�������:	   														*/
/*				NONE 												*/
/*�������:	   														*/
/*				result: successful or fail							*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucUpdateFlashBasicInformation(void)
{
	unsigned char i,ret,j;
	
	/*���ǰ��3��Block�Ƿ���Flash������Ϣ*/
	for (i=0; i<3; i++)
	{
		ret = ucQueryFlashBasicInformation(i);
		if (ret == 0) 
		{
			uiBasicBlock = i;
			break;			
		}
	}
	
	/*3��Block����Flash������Ϣ*/
	if (ret)
	{
		/*��ǰ����������ѡ���һ�������������н���ȱʡ�Ļ�����Ϣ���*/
		for (i=0; i<3; i++)
		{
			ret = ucSetupFlashBasicInformation(i);
			if (ret == 0) 
			{
				uiBasicBlock = i;
				break;			
			}
		}
	}
	
	return(ret);
}
	
	
/********************************************************************/
/*��������:vFindCurrentBlockAndPointerOfRecord						*/
/*��������:Ϊָ���ļ�¼���Ͳ��ҵ�ǰ�����͵�ǰָ��					*/
/*�������:	   														*/
/*				ucRecordType: ָ���ļ�¼����						*/
/*�������:	   														*/
/*				NONE												*/
/*�޸ļ�¼:															*/
/********************************************************************/
static void vFindCurrentBlockAndPointerOfRecord(unsigned char ucRecordType)
{
	unsigned char ucBlkNo1, ucBlkNo2, ucSizeOfRecord;
	unsigned short int i, usNewestPtr1, usNewestPtr2;
	unsigned int uiStartAddress;
	
	
	ucBlkNo1 = tRecordManageParameterp[ucRecordType].tUsedBlocks[0].ucBlockNum;
	ucBlkNo2 = tRecordManageParameterp[ucRecordType].tUsedBlocks[1].ucBlockNum;
	
	ucSizeOfRecord = tRecordManageParameterp[ucRecordType].ucSizeofRecord;	
	
	
	//�ҳ���һ��������д��ĵ�һ����¼ƫ����
	uiStartAddress = uiBlockOffsetAddr[ucBlkNo1];
	usNewestPtr1 = tRecordManageParameterp[ucRecordType].usMaxNumOfRecord;
	tRecordManageParameterp[ucRecordType].tUsedBlocks[0].usBadRecordNum = 0x00;
	for (i=0; i<tRecordManageParameterp[ucRecordType].usMaxNumOfRecord; i++)
	{
		//�����¼�е�����Ϊ�գ������ʹ��
		if (ReadWord(uiStartAddress + ucSizeOfRecord * i) == VACANT_RECORD)
		{					
			usNewestPtr1 = i;				
			break;	
		}
		else if (ReadWord(uiStartAddress + ucSizeOfRecord * i) == INVALID_RECORD)
		{
			tRecordManageParameterp[ucRecordType].tUsedBlocks[0].usBadRecordNum ++;
		}
	}
	
	//�ҳ��ڶ���������д��ĵ�һ����¼ƫ����
	uiStartAddress = uiBlockOffsetAddr[ucBlkNo2];
	usNewestPtr2 = tRecordManageParameterp[ucRecordType].usMaxNumOfRecord;
	tRecordManageParameterp[ucRecordType].tUsedBlocks[1].usBadRecordNum = 0x00;
	for (i=0; i<tRecordManageParameterp[ucRecordType].usMaxNumOfRecord; i++)
	{
		//�����¼�е�����Ϊ�գ������ʹ��
		if (ReadWord(uiStartAddress + ucSizeOfRecord * i) == VACANT_RECORD)
		{					
			usNewestPtr2 = i;		
			break;
		}
		else if (ReadWord(uiStartAddress + ucSizeOfRecord * i) == INVALID_RECORD)
		{
			tRecordManageParameterp[ucRecordType].tUsedBlocks[1].usBadRecordNum ++;
		}
	}
	
	//ȷ�����������е�״̬
	if (usNewestPtr1 == 0x00)
	{
		tRecordManageParameterp[ucRecordType].tUsedBlocks[0].ucBlockStatus = BLOCK_NO_DATA;
	}
	else
	{
		tRecordManageParameterp[ucRecordType].tUsedBlocks[0].ucBlockStatus = BLOCK_HAS_DATA;
	}
	
	if (usNewestPtr2 == 0x00)
	{
		tRecordManageParameterp[ucRecordType].tUsedBlocks[1].ucBlockStatus = BLOCK_NO_DATA;
	}
	else
	{
		tRecordManageParameterp[ucRecordType].tUsedBlocks[1].ucBlockStatus = BLOCK_HAS_DATA;
	}
	
	//�������������Ϊ�գ���ѡ����һ��������Ϊ��ǰ����
	if ((usNewestPtr1 == 0x00) && (usNewestPtr2 == 0x00))
	{
		tRecordManageParameterp[ucRecordType].ucCurrentBlock = ucBlkNo1;
		tRecordManageParameterp[ucRecordType].usNewestPointer = usNewestPtr1;
	}
	//����ڶ������������ҵ�һ����������ѡ���һ������Ϊ��ǰ���������Ҳ�����һ������
	//����ǰλ��ָ���ʼ��
	else if (usNewestPtr2 == tRecordManageParameterp[ucRecordType].usMaxNumOfRecord)
	{
		if (usNewestPtr1 == tRecordManageParameterp[ucRecordType].usMaxNumOfRecord)
		{
			tRecordManageParameterp[ucRecordType].ucCurrentBlock = ucBlkNo1;
			tRecordManageParameterp[ucRecordType].usNewestPointer = 0x00;

			//����һ��������ΪDIRTY״̬���ȴ���ʱ����ʱ���в����������ϵͳ���ò�����ȡ
			//֮ǰ�����꣬�����Ǹ�ʱ��Ҫ���������
			tRecordManageParameterp[ucRecordType].tUsedBlocks[0].ucBlockStatus = BLOCK_IS_DIRTY;
		}
		else
		{
			tRecordManageParameterp[ucRecordType].ucCurrentBlock = ucBlkNo1;
			tRecordManageParameterp[ucRecordType].usNewestPointer = usNewestPtr1;
		}
	}
	//�����һ������������һ���ǵڶ��������ǵ�ǰ����
	else if (usNewestPtr1 == tRecordManageParameterp[ucRecordType].usMaxNumOfRecord)
	{
		tRecordManageParameterp[ucRecordType].ucCurrentBlock = ucBlkNo2;
		tRecordManageParameterp[ucRecordType].usNewestPointer = usNewestPtr2;
	}
	//�������������δ������һ�����ݶ�������ǵ�ǰ����
	else if (usNewestPtr1 < usNewestPtr2)
	{
		tRecordManageParameterp[ucRecordType].ucCurrentBlock = ucBlkNo2;
		tRecordManageParameterp[ucRecordType].usNewestPointer = usNewestPtr2;
	}
	else
	{
		tRecordManageParameterp[ucRecordType].ucCurrentBlock = ucBlkNo1;
		tRecordManageParameterp[ucRecordType].usNewestPointer = usNewestPtr1;
	}
	
			
	
	PRINT_TRACE(Debug)("The block %d state is %d, NewestPtr is %d\n", ucBlkNo1, 
		  	tRecordManageParameterp[ucRecordType].tUsedBlocks[0].ucBlockStatus, usNewestPtr1);
	PRINT_TRACE(Debug)("The block %d state is %d, NewestPtr is %d\n", ucBlkNo2, 
			tRecordManageParameterp[ucRecordType].tUsedBlocks[1].ucBlockStatus, usNewestPtr2);	
	PRINT_TRACE(Debug)("The current block is %d\n", tRecordManageParameterp[ucRecordType].ucCurrentBlock);

	return;
}


/********************************************************************/
/*��������:ucRegisterNewBlockForRecord								*/
/*��������:��ʼ��������¼�������ݽṹ								*/
/*�������:	   														*/
/*				NONE												*/
/*�������:	   														*/
/*				result: successful or fail							*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucRegisterNewBlockForRecord(unsigned char cnt, unsigned short j)
{
	unsigned short i, Size;
	unsigned short *p = (unsigned short *)&tBasicFlashInfo;
	unsigned char ret;
	
	Size = sizeof(tBasicFlashInfo)/2;
	
	for (i=0; i<tBasicFlashInfo.usTotalBlocks; i++)
	{
		if ((BlockSizeList[i] == 0x8000) &&
			(tBasicFlashInfo.tBlockAttrList[i].usBlockState == BLOCK_NOT_USED) &&
			(tBasicFlashInfo.tBlockAttrList[i].usRecordType == BLOCK_NOT_USED) &&
			(tBasicFlashInfo.tBlockAttrList[i].usPages == BLOCK_NOT_USED))
		{
			tBasicFlashInfo.tBlockAttrList[i].usBlockState = BLOCK_ACTIVE;
			tBasicFlashInfo.tBlockAttrList[i].usRecordType = cnt;
			tBasicFlashInfo.tBlockAttrList[i].usPages = BlockSizeList[i]/0x200;
			
			ret = ucWriteDataToFlash(uiBlockOffsetAddr[uiBasicBlock], p, Size);
			
			PRINT_TRACE(Debug)("uiBasicBlock is %d, i = %d, ret = %d\n", uiBasicBlock, i, ret);
			
			if (ret == 0)
			{
				PRINT_TRACE(Debug)("==========================================\n");
				PRINT_TRACE(Debug)("The %dnd block is assigned to %s\n", i, cucRecordName[cnt]);
				break;	
			}
		}
	}
	
	if (i == tBasicFlashInfo.usTotalBlocks)
	{
		return (1);		//δ�ҵ�������
	}
	else
	{
		return (0);		//�ɹ��ҵ��󣬷���
	}
}

/********************************************************************/
/*��������:ucInitialRecordManagement								*/
/*��������:��ʼ��������¼�������ݽṹ								*/
/*�������:	   														*/
/*				NONE												*/
/*�������:	   														*/
/*				result: successful or fail							*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucInitialRecordManagement(void)
{
	unsigned short i,j;
	unsigned char cnt,ucSizeOfRecord;
	unsigned short usMaxActiveBlockNum;
	unsigned char ret;
	
	//Ϊÿһ���¼������Ӧ��������
	for (cnt=0; cnt<INV_FAULT_RECORD; cnt++)
	{
		//�Ӻ���ǰ�����洢ÿ�ּ�¼���ݵ�������, ������2��
		j = 0;
		for (i=(tBasicFlashInfo.usTotalBlocks-1); ; i--)
		{
			if ((tBasicFlashInfo.tBlockAttrList[i].usBlockState == BLOCK_ACTIVE) && 
			   (tBasicFlashInfo.tBlockAttrList[i].usRecordType == (cnt + 1)))
			{
				tRecordManageParameterp[cnt+1].tUsedBlocks[j++].ucBlockNum = i;
				
				PRINT_TRACE(Debug)("The %s used the %d block\n", cucRecordName[cnt+1], i);	
			}			
			
			//�ҵ���������Ӧ��������,�������˳�����
			if (j == 2)
			{
				//printk("More than 2 blocks store the Record %d\n", cnt+1);	
				break;
			}
			
			//����ҵ���������,����������е��������˳����β���
			//��������ҵ���һ����64Kbytes��С���������˳����β���
			if ((BlockSizeList[i] != 0x8000) || (i == 0x00))
			{				
				break;
			}
		}			
		
		//����ü�¼�洢�����������������������������ʹ�ã�TBD
		if (j<2)
		{
			for (; j<2; j++)
			{				
				PRINT_TRACE(Debug)("Apply the new block for the %dnd of %s\n", j, cucRecordName[cnt+1]);
				ret = ucRegisterNewBlockForRecord(cnt+1, j);
				
				//�������ʧ�ܣ���˵��Flash�Ѿ�ʹ����ϡ�
				if (ret)
				{
					printk("The Flash has no any good block for application\n");	
					
					/*
					for (;;)
					{
						
					}
					*/
				}				
			}	
		}
		
		//���һ��Block(64Kbytes)�������Դ�ŵļ�¼��
		if ((cnt+1) < EVENT_RECORD)				//���¼
		{			
			tRecordManageParameterp[cnt+1].ucSizeofRecord = sizeof(BLOCK_RECORD_T)/sizeof(short);	
			tRecordManageParameterp[cnt+1].usMaxNumOfRecord = ((0x8000 / (sizeof(BLOCK_RECORD_T)/sizeof(short))) - 1);		
			tRecordManageParameterp[cnt+1].usPrewarnNum	= MAX_BLOCK_PREWARN_NUM;
		}
		else if ((cnt+1) == EVENT_RECORD)		//�¼���¼
		{
			tRecordManageParameterp[cnt+1].ucSizeofRecord = sizeof(EVENT_RECORD_T)/sizeof(short);
			tRecordManageParameterp[cnt+1].usMaxNumOfRecord = ((0x8000 / (sizeof(EVENT_RECORD_T)/sizeof(short))) - 1);			
			tRecordManageParameterp[cnt+1].usPrewarnNum	= MAX_EVENT_PREWARN_NUM;
		}
		else									//���ϼ�¼
		{
			tRecordManageParameterp[cnt+1].ucSizeofRecord = sizeof(FAULT_RECORD_T)/sizeof(short);
			tRecordManageParameterp[cnt+1].usMaxNumOfRecord = ((0x8000 / (sizeof(FAULT_RECORD_T)/sizeof(short))) - 1);
			tRecordManageParameterp[cnt+1].usPrewarnNum	= MAX_FAULT_PREWARN_NUM;
		}
		
		PRINT_TRACE(Debug)("The %s 's MaxNumber of Records is %d\n",cucRecordName[cnt+1], tRecordManageParameterp[cnt+1].usMaxNumOfRecord);
				
		//Send a reset common command for read flash
		Send_CMD(CMD_RESET_DATA);
		
		//Ϊ�ü�¼������ǰ�����͵�ǰָ��
		vFindCurrentBlockAndPointerOfRecord(cnt+1);		
	}
	
	return(0);
}






/********************************************************************/
/*��������:ucReadOneRecordFromFlash									*/
/*��������:��Flash�ж�ȡָ����¼��ָ�������ļ�¼����				*/
/*�������:	   														*/
/*				pFlashRecord:  ��¼ָ���еļ�¼���ͣ���¼������		*/
/*�������:	   														*/
/*				pFlashRecord:  ��¼�ľ�������						*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucReadOneRecordFromFlash(FLASH_RECORD_T *pFlashRecord)
{
	unsigned short usAttr, usCnt, usIndex, usEventID;
	unsigned short usRcdID, usPointer, i;
	unsigned char ucBlock,ucSize;
	unsigned char ucOldBlk, ucNewBlk, ucTempBlk;
	unsigned int uiStartAddress;
	FLASH_RECORD_T tRecord;	
	unsigned short *ptRecord = (unsigned short *)(&tRecord);	
	
	unsigned char ret = ERROR_READ_RECORD_NONE;

	unsigned short usErrRecordCnt = 0; 
	
	
	PRINT_TRACE(Debug)("The ucReadOneRecordFromFlash function enry --1\n"); 	
			
		
	//�ӹ�һЩ��ʱ����
	usRcdID = pFlashRecord ->usID;											//��д��ļ�¼ID��
	ucNewBlk = tRecordManageParameterp[usRcdID].ucCurrentBlock;				//��ǰBlock
	ucSize = tRecordManageParameterp[usRcdID].ucSizeofRecord;				//��¼�Ĵ�С
	usPointer = tRecordManageParameterp[usRcdID].usNewestPointer;			//��ǰ��д���¼��ָ��
	
	if (pFlashRecord -> Data[0] == 0x00)
	{
		PRINT_TRACE(Debug)("Read record index must larger than 0x00\n");
		ret = ERROR_READ_RECORD_INDEX;
		return(ret);
	}
	else
	{	
		usIndex = pFlashRecord ->Data[0];
		usCnt = usIndex;
	}
	
	if (ucNewBlk == tRecordManageParameterp[usRcdID].tUsedBlocks[0].ucBlockNum)
	{		
		ucOldBlk = tRecordManageParameterp[usRcdID].tUsedBlocks[1].ucBlockNum;
	}
	else
	{		
		ucOldBlk = tRecordManageParameterp[usRcdID].tUsedBlocks[0].ucBlockNum;
	}
	
	//1, �����ǰ���ڲ��������У�����Ҫ��ͣ
	if (ucFlashStatus == BLOCKERASING)
	{
		EraseSuspend();
	}	
	
	//��������ǰ����
	uiStartAddress = uiBlockOffsetAddr[ucNewBlk];							//��ǰBlock����ʼ��ַ
	ucTempBlk = ucNewBlk;
	for (;;)
	{
		if (usPointer == 0x00)
		{
			if (uiStartAddress == uiBlockOffsetAddr[ucNewBlk])
			{
				uiStartAddress = uiBlockOffsetAddr[ucOldBlk];
				ucTempBlk = ucOldBlk;
				usPointer = tRecordManageParameterp[usRcdID].usMaxNumOfRecord - 1;

				continue;
			}
			else
			{
				ret = ERROR_READ_RECORD_NONE;	

				break;
			}			
		}
		else
		{		
			usPointer --;
		}
		
		vReadDataFromFlash(uiStartAddress + usPointer * ucSize, ptRecord, ucSize);
		usAttr = tRecord.usAttribute;	
			
		
		if ((usAttr == VALID_CURRENT_EVENT_RECORD) ||
			(usAttr == INVALID_HISTORY_EVENT_RECORD) ||
			(usAttr == VALID_HISTORY_EVENT_RECORD))			//VALID_HISTORY_EVENT_RECORD = VALID_RECORD
		{										
			usCnt --;
			
			if (usCnt == 0x00)		//�ҵ���һ��ָ���ļ�¼
			{				
				PRINT_TRACE(Debug)("RecordID = %d, Block = %d, usPointer = %d, usAttr = %d\n", usRcdID, ucTempBlk,usPointer,usAttr);
								
				for (i=0; i<ucSize; i++)
				{
					*((unsigned short *)pFlashRecord + i) = *(ptRecord + i);
				}	
				ret = 0x00;				
				break;
		    	}
			
		}
		else
		{
			usErrRecordCnt = usErrRecordCnt + 1;

			if (usErrRecordCnt >= MAX_READ_ERROR_COUNT)
			{
				break;
			}
		}
		
	}
	
	PRINT_TRACE(Debug)("Find %d record return a %d value\n", usIndex, ret);

	
	//5, �����ǰ���ڲ������ҹ����У�����Ҫ��������
	if (ucFlashStatus == ERASEPENDING)
	{
		EraseResume();
	}
	
	return (ret);
}


/********************************************************************/
/*��������:ucReadOneRecord											*/
/*��������:��Flash�ж�ȡָ����¼��ָ�������ļ�¼����				*/
/*�������:	   														*/
/*				pFlashRecord:  ��¼ָ���еļ�¼���ͣ���¼������		*/
/*�������:	   														*/
/*				pFlashRecord:  ��¼�ľ�������						*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucReadOneRecord(FLASH_RECORD_T *pFlashRecord)
{
	unsigned char ret;
	
	if ((pFlashRecord -> usID < EVENT_RECORD) && (pFlashRecord -> usID > FLASH_BASIC_INFO))
	{		
		pFlashRecord -> Data[0] = 0x01;
		ret = ucReadOneRecordFromFlash(pFlashRecord);				
	}
	else if (pFlashRecord -> usID == EVENT_RECORD)
	{		
		ret = ucReadOneRecordFromFlash(pFlashRecord);	
	}
	else if ((pFlashRecord -> usID == REC_FAULT_RECORD) ||
			 (pFlashRecord -> usID == INV_FAULT_RECORD))
	{
		ret = ucReadOneRecordFromFlash(pFlashRecord);
	}
	else
	{
		ret = ERROR_READ_RECORD_ID;
	}
	
	return (ret);
}


/********************************************************************/
/*��������:ucWriteOneNewRecordToFlash								*/
/*��������:��Flash��д��һ��ָ���¼�¼����							*/
/*�������:	   														*/
/*				pFlashRecord:   Ҫд��ļ�¼��ָ��					*/
/*�������:	   														*/
/*				result: successful or fail							*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucWriteOneNewRecordToFlash(FLASH_RECORD_T * pRecord)
{
	unsigned short usAttr, usValue;
	unsigned short usPointer, i;
	unsigned char ucBlock,ucSize;
	unsigned int uiStartAddress;
	unsigned short usRcdID;
	unsigned char ucBlock1, ucBlock2;
	unsigned char ucOldBlock;
	int ucErrCnt;
	unsigned char ucFlashStatusBak;
	
	FLASH_RECORD_T tRecord;	
	unsigned short *ptRecord = (unsigned short *)(&tRecord);
	
	unsigned char k, ret, usNew;	
	
		
	//0, ׼��һЩ����ʹ�õ��м���ʱ����	
	usRcdID = pRecord ->usID;											//��д��ļ�¼ID��	
	
	ucBlock = tRecordManageParameterp[usRcdID].ucCurrentBlock;			//��ǰBlock
	ucSize = tRecordManageParameterp[usRcdID].ucSizeofRecord;			//��¼�Ĵ�С
	usPointer = tRecordManageParameterp[usRcdID].usNewestPointer;		//��ǰ��д���¼��ָ��
	
	uiStartAddress = uiBlockOffsetAddr[ucBlock] + ucSize * usPointer;	//��ǰBlock�Ĵ�д���¼�ĵ�ַ
	
	ucBlock1 = tRecordManageParameterp[usRcdID].tUsedBlocks[0].ucBlockNum;		//��¼��ռ�õĵ�һ��Block
	ucBlock2 = tRecordManageParameterp[usRcdID].tUsedBlocks[1].ucBlockNum;		//��¼��ռ�õ���һ��Block	
	ucOldBlock = ucBlock;
	
	//���вδ������ļ�¼���ݿ��������ؾֲ�������
	for (i=0; i<ucSize; i++)
	{
		*(ptRecord + i) = *((unsigned short *)pRecord + i);
	}
	
	
	//1, �����ǰ���ڲ��������У�����Ҫ��ͣ
	if (ucFlashStatus == BLOCKERASING)
	{
		EraseSuspend();
	}		
	
		
	ucErrCnt = 0;
	//////////////////////////////////////////////////////////////////
	//2, ��ʼд��һ���¼�¼	
	//////////////////////////////////////////////////////////////////	
	//2.1, д��һ����¼�����û�гɹ���������һ��λ��д��
	for (;;)
	{
		//2.1.1���׼��д����������ڲ����У�����������æ����
		if (ucBlock1 == ucBlock)
		{
			if (tRecordManageParameterp[usRcdID].tUsedBlocks[0].ucBlockStatus == BLOCK_ERASING)
			{
				PRINT_TRACE(Debug)("The Block %d is eraseing, inhibit to be writed\n", ucBlock);
				ret = ERROR_WRITE_RECORD_BUSY;
				break;
			}
		}
		else
		{
			if (tRecordManageParameterp[usRcdID].tUsedBlocks[1].ucBlockStatus == BLOCK_ERASING)
			{
				PRINT_TRACE(Debug)("The Block %d is eraseing, inhibit to be writed\n", ucBlock);
				ret = ERROR_WRITE_RECORD_BUSY;
				break;
			}
		}
			
		//2.1.1���д��ļ�¼����������Ԥ�澯����,����Խ����������б�עΪDirty
		if (usPointer > tRecordManageParameterp[usRcdID].usPrewarnNum)
		{
			if (ucBlock1 == ucBlock)
			{
				if (tRecordManageParameterp[usRcdID].tUsedBlocks[1].ucBlockStatus == BLOCK_HAS_DATA)
				{
					tRecordManageParameterp[usRcdID].tUsedBlocks[1].ucBlockStatus = BLOCK_IS_DIRTY;
				}
			}
			else
			{
				if (tRecordManageParameterp[usRcdID].tUsedBlocks[0].ucBlockStatus == BLOCK_HAS_DATA)
				{
					tRecordManageParameterp[usRcdID].tUsedBlocks[0].ucBlockStatus = BLOCK_IS_DIRTY;
				}
			}
		}


		if (usPointer < tRecordManageParameterp[usRcdID].usMaxNumOfRecord)
		{
			//���¼�����ʼ��ַ
			uiStartAddress = uiBlockOffsetAddr[ucBlock] + ucSize * usPointer;	//��ǰBlock�Ĵ�д���¼�ĵ�ַ

			ucFlashStatusBak = ucFlashStatus;		
			ucFlashStatus = PROGRAMING;				/*Register a programming status*/	
			//2.1.2, ��ָ�����ָ��λ��д��һ����¼
			ret = WriteWord(uiStartAddress, INVALID_RECORD);			//��д��һ����Ч��¼��־
			ret = ucWriteDataToFlash(uiStartAddress, ptRecord, ucSize);
			ucFlashStatus = ucFlashStatusBak;				/*feed back the original status*/	
		}
			

		if (ret == 0)
		{
			PRINT_TRACE(Debug)("Write a new %s Record: Blk = %d, address = %d, P = %d,  usModuleIdx:%d, usEventID:%d\n\n", 
							cucRecordName[usRcdID], ucBlock, uiStartAddress, usPointer, *(ptRecord+2), *(ptRecord+3));
		}
		
		if (usPointer < tRecordManageParameterp[usRcdID].usMaxNumOfRecord)
		{
			tRecordManageParameterp[usRcdID].usNewestPointer++;
			usPointer++;	
		}
												//д���ָ���1
		
		//2.1.2, ע����Block�Ѿ�д������
		if (ucBlock == ucBlock1)
		{
			tRecordManageParameterp[usRcdID].tUsedBlocks[0].ucBlockStatus = BLOCK_HAS_DATA;
		}
		else
		{
			tRecordManageParameterp[usRcdID].tUsedBlocks[1].ucBlockStatus = BLOCK_HAS_DATA;
		}
			
		//2.1.3, ���ü�¼�Ƿ�д��ɹ�
		//������󣬾�������λ�ã�������һ��λ������
		if (ret)				
		{		
			//д�������1
			ucErrCnt = ucErrCnt + 1;

			if (ucErrCnt >= MAX_WRITE_ERROR_COUNT)
			{
				break;
			}
			
			//2.1.3.1, ע����Block��д��һ����¼
			if (ucBlock == ucBlock1)
			{
				tRecordManageParameterp[usRcdID].tUsedBlocks[0].usBadRecordNum ++;
			}
			else
			{
				tRecordManageParameterp[usRcdID].tUsedBlocks[1].usBadRecordNum ++;
			}
				
			//2.1.3.2, ���д����������ĩβ,�����뵽��һ����������д����
			if (usPointer >= tRecordManageParameterp[usRcdID].usMaxNumOfRecord)			
			{									
				//����Ѿ��л��������ˣ���д������ֻ�ܹ������˳�
				if (ucBlock != ucOldBlock)
				{
					PRINT_TRACE(Debug)("===================================================================");
					PRINT_TRACE(Debug)("To writing the %s Record, two blocks can not be writen correctly!\n", cucRecordName[usRcdID]);
					PRINT_TRACE(Debug)("The Block %d 's badrecordNum = %d\n", ucBlock1, tRecordManageParameterp[usRcdID].tUsedBlocks[0].usBadRecordNum);
					PRINT_TRACE(Debug)("The Block %d 's badrecordNum = %d\n", ucBlock2, tRecordManageParameterp[usRcdID].tUsedBlocks[1].usBadRecordNum);
					PRINT_TRACE(Debug)("===================================================================");
					ret = ERROR_WRITE_RECORD_BUSY;
					break;
				}
				else
				{
					if (ucBlock1 == ucBlock)
					{					
						tRecordManageParameterp[usRcdID].usNewestPointer = 0x00;
						usPointer = 0x00;
						tRecordManageParameterp[usRcdID].ucCurrentBlock = ucBlock2;
						ucBlock = ucBlock2;					
					}
					else
					{
						tRecordManageParameterp[usRcdID].usNewestPointer = 0x00;
						usPointer = 0x00;
						tRecordManageParameterp[usRcdID].ucCurrentBlock = ucBlock1;
						ucBlock = ucBlock1;					
					}
				}
			}						
		}
		else
		{
			//2.1.3.2, ���д����������ĩβ,�����뵽��һ����������д����
			if (usPointer >= tRecordManageParameterp[usRcdID].usMaxNumOfRecord)			
			{										 
				if (ucBlock1 == ucBlock)
				{					
					tRecordManageParameterp[usRcdID].usNewestPointer = 0x00;
					usPointer = 0x00;
					tRecordManageParameterp[usRcdID].ucCurrentBlock = ucBlock2;
					ucBlock = ucBlock2;					
				}
				else
				{
					tRecordManageParameterp[usRcdID].usNewestPointer = 0x00;
					usPointer = 0x00;
					tRecordManageParameterp[usRcdID].ucCurrentBlock = ucBlock1;
					ucBlock = ucBlock1;					
				}
			}
			
			break;				//д�ü�¼ʱ���ɹ���ֱ���˳�����д��ѭ��
		}		
	}		
	
	
	//3, �����ǰ���ڲ������ҹ����У�����Ҫ��������
	if (ucFlashStatus == ERASEPENDING)
	{
		EraseResume();
	}
	
	return (ret);
	
}


/********************************************************************/
/*��������:ucWriteAHistoryEventLogToFlash							*/
/*��������:��Flash��д��һ��ָ���¼�¼����							*/
/*�������:	   		
/*				usModIdx: ģ���
/*				usID:   	Ҫд�����ʷ�¼���¼��ID				*/
/*				tEndTime:   Ҫд�����ʷ�¼���¼�Ľ���ʱ��			*/
/*�������:	   														*/
/*				result: successful or fail							*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucWriteAHistoryEventLogToFlash(unsigned short usModIdx, unsigned short usID, DATE_TIME_T tEndTime)
{
	unsigned short usModuleIdx, usEventID, usAttr;
	unsigned short usPointer, i, ret;
	unsigned char ucBlock,ucSize;
	unsigned int uiStartAddress;
	EVENT_RECORD_T tEventRecord;	
	unsigned short *ptEvent = (unsigned short *)(&tEventRecord);
	DATE_TIME_T tEndTimeLocal;
	unsigned short *ptTime = (unsigned short *)(&tEndTimeLocal);
	unsigned char ucFlashStatusBak;
	int usReadCounter;
	
	unsigned char ucOldBlk, ucNewBlk;

	//�����ǰ���ڲ��������У�����Ҫ��ͣ
	if (ucFlashStatus == BLOCKERASING)
	{
		EraseSuspend();
	}	
	
		
	ucNewBlk = tRecordManageParameterp[EVENT_RECORD].ucCurrentBlock;		//��ǰBlock
	ucSize = sizeof(EVENT_RECORD_T)/sizeof(short);							//��¼�Ĵ�С
	usPointer = tRecordManageParameterp[EVENT_RECORD].usNewestPointer;		//��ǰ��д���¼��ָ��
	
	tEndTimeLocal = tEndTime;
		
	if (ucNewBlk == tRecordManageParameterp[EVENT_RECORD].tUsedBlocks[0].ucBlockNum)
	{		
		ucOldBlk = tRecordManageParameterp[EVENT_RECORD].tUsedBlocks[1].ucBlockNum;
	}
	else
	{		
		ucOldBlk = tRecordManageParameterp[EVENT_RECORD].tUsedBlocks[0].ucBlockNum;
	}
	
	//��������ǰ����
	uiStartAddress = uiBlockOffsetAddr[ucNewBlk];							//��ǰBlock����ʼ��ַ

	PRINT_TRACE(Debug)("Find start Block%d address%d ...", ucNewBlk, uiStartAddress);

	usReadCounter = 0;
	
	for (;;)
	{
		if (usPointer == 0x00)
		{
			if (uiStartAddress == uiBlockOffsetAddr[ucNewBlk])
			{
				uiStartAddress = uiBlockOffsetAddr[ucOldBlk];			//û���ҵ�����Ҫ������һ������

				PRINT_TRACE(Debug)("Change another Block, Find start Block%d address%d\n", ucOldBlk, uiStartAddress);
				usPointer = tRecordManageParameterp[EVENT_RECORD].usMaxNumOfRecord - 1;
			}
			else
			{
				ret = ERROR_WRITE_EVENT_HIS_NONE;				
				break;
			}			
		}
		else
		{		
			usPointer --;
		}		
		
		vReadDataFromFlash(uiStartAddress + usPointer * ucSize, ptEvent, ucSize);
		usAttr = tEventRecord.usAttribute;
		usEventID = tEventRecord.tEventLog.usID;
		usModuleIdx = tEventRecord.tEventLog.usModuleIdx;

		usReadCounter = usReadCounter + 1;

		if (usReadCounter >= MAX_EVENT_COUNT)
		{
			break;
		}

		PRINT_TRACE(Debug)("finding usPointer:%d address:%d,  usAttr:%d usEventID:%d, usModuleIdx:%d... \n", usPointer, (uiStartAddress + usPointer * ucSize), usAttr, usEventID, usModuleIdx);
				
		if ((usAttr == VALID_CURRENT_EVENT_RECORD) 
			&& (usEventID == usID)
			&& (usModuleIdx == usModIdx))
		{
			ucFlashStatusBak = ucFlashStatus;		
			ucFlashStatus = PROGRAMING;				/*Register a programming status*/	
			//��д��һ����Ч����ʷ��¼
			WriteWord(uiStartAddress + usPointer * ucSize, INVALID_HISTORY_EVENT_RECORD);
			
			//��д�����ʱ��
			for (i=0; i<(sizeof(DATE_TIME_T)/2); i++)
			{
				WriteWord(uiStartAddress + usPointer * ucSize + 10 + i, *(ptTime + i));
			}
			
			//��д���¼���ͼ���Ч��ʷ��־
			WriteWord(uiStartAddress + usPointer * ucSize + 4, HISTORY_EVENT);
			WriteWord(uiStartAddress + usPointer * ucSize, VALID_HISTORY_EVENT_RECORD);

			ucFlashStatus = ucFlashStatusBak;				/*feed back the original status*/	
		
			ret = 0x00;											//�ҵ���һ�����ʵļ�¼
			break;
		}
		else if (usAttr == VACANT_RECORD)
		{
			ret = ERROR_WRITE_EVENT_HIS_NONE;					//û���ҵ�������������
			break;
		}		
	}
				
	PRINT_TRACE(Debug)("Find matched event log return a %d value\n", ret);	

	//�����ǰ���ڲ������ҹ����У�����Ҫ��������
	if (ucFlashStatus == ERASEPENDING)
	{
		EraseResume();
	}
		
	return (ret);
}



/********************************************************************/
/*��������:ucWriteOneRecord											*/
/*��������:��Flash��д��һ��ָ����¼����							*/
/*�������:	   														*/
/*				pFlashRecord:   Ҫд��ļ�¼��ָ��					*/
/*�������:	   														*/
/*				result: successful or fail							*/
/*�޸ļ�¼:															*/
/********************************************************************/
static unsigned char ucWriteOneRecord(FLASH_RECORD_T *pFlashRecord)
{
	unsigned char ret;
	EVENT_RECORD_T *tPEvent = (EVENT_RECORD_T *)pFlashRecord;
	DATE_TIME_T tEndTime;
	
	
	if ((pFlashRecord -> usID > FLASH_BASIC_INFO) && (pFlashRecord -> usID < EVENT_RECORD))
	{				
		pFlashRecord -> usAttribute = VALID_RECORD;
		ret = ucWriteOneNewRecordToFlash(pFlashRecord);					
	}
	else if (pFlashRecord -> usID == EVENT_RECORD)
	{
		//�ж�����ʷ�¼������ǵ�ǰ�¼�
		if (tPEvent->tEventLog.usType == HISTORY_EVENT)
		{
			PRINT_TRACE(Debug)("Create a histroy log: ID = %d\n", tPEvent->tEventLog.usID);
			//��ʼ�����¼���¼�еĸ�����ǰ��¼����ӽ���ʱ�䣬����ע����Ϊ��ʷ��¼	
			tEndTime = 	tPEvent->tEventLog.tEndTime;
			ret = ucWriteAHistoryEventLogToFlash(tPEvent->tEventLog.usModuleIdx, tPEvent->tEventLog.usID, tEndTime);
		}
		else
		{
			PRINT_TRACE(Debug)("Create a current log: ID = %d\n", tPEvent->tEventLog.usID);
			
			pFlashRecord -> usAttribute = VALID_CURRENT_EVENT_RECORD;
			ret = ucWriteOneNewRecordToFlash(pFlashRecord);	
		}
	}
	else if ((pFlashRecord -> usID == REC_FAULT_RECORD) || 
			(pFlashRecord -> usID == INV_FAULT_RECORD))
	{
		//ret = ucWriteOneFaultRecordToFlash(pFlashRecord);	
		ret = ucWriteOneNewRecordToFlash(pFlashRecord);
	}			
	else
	{
		ret = ERROR_WRITE_RECORD_ID;	
	}
	
	return (ret);
}




////////////////////////////////////////////////////////////////
// ���Դ���
////////////////////////////////////////////////////////////////
static void vPrintAllValidDataOnOneBlock(unsigned char ucBlock)
{
	unsigned short usValue;
	unsigned short i;
	unsigned char ucSize;
	unsigned int uiStartAddress;	
	
	
	ucSize = sizeof(BLOCK_RECORD_T)/2;									//��¼�Ĵ�С	
	uiStartAddress = uiBlockOffsetAddr[ucBlock];						//��ǰBlock����ʼ��ַ
	
	//1, �����ǰ���ڲ��������У�����Ҫ��ͣ
	if (ucFlashStatus == BLOCKERASING)
	{
		EraseSuspend();
	}	
	
	PRINT_TRACE(Debug)("Block %d -------Start Address is %d\n", ucBlock, uiStartAddress);
	PRINT_TRACE(Debug)("Block %d -------the Size is %d\n", ucBlock, BlockSizeList[ucBlock]);
	
	for (i=0; i<BlockSizeList[ucBlock]; i++)
	{
		usValue = ReadWord(uiStartAddress + i);
		
		if (usValue != NOT_USED)
		{
			PRINT_TRACE(Debug)("Block %d -------Address %d,     Value is %d\n", ucBlock, uiStartAddress + i, usValue);	
		}
	}	
	
	//5, �����ǰ���ڲ������ҹ����У�����Ҫ��������
	if (ucFlashStatus == ERASEPENDING)
	{
		EraseResume();
	}
}

static void vPrintAllValidRecordOnOneBlock(unsigned char ucBlock)
{
	unsigned short usRecordCnt, usValue;
	unsigned short i, j;
	unsigned char ucSize;
	unsigned int uiStartAddress;
	
	
	ucSize = sizeof(BLOCK_RECORD_T)/2;									//��¼�Ĵ�С	
	uiStartAddress = uiBlockOffsetAddr[ucBlock];						//��ǰBlock����ʼ��ַ
	usRecordCnt = ((BlockSizeList[ucBlock] / (sizeof(BLOCK_RECORD_T)/2)) - 1);
	
	
	
	//1, �����ǰ���ڲ��������У�����Ҫ��ͣ
	if (ucFlashStatus == BLOCKERASING)
	{
		EraseSuspend();
	}	
	
	for (j=0; j<usRecordCnt; j++)
	{
		usValue = ReadWord(uiStartAddress + ucSize * j);
		
		if (usValue != NOT_USED)
		{
			PRINT_TRACE(Debug)("The %d record in %d block is as following ========\n", j, ucBlock);
		
			
			for (i=0; i<ucSize; i++)
			{
				usValue = ReadWord(uiStartAddress + ucSize * j + i);
				
				if (usValue != NOT_USED)
				{
					PRINT_TRACE(Debug)("Address %d ----------- Value is %d\n", i, usValue);	
				}
			}	
		}
	}
	
	//5, �����ǰ���ڲ������ҹ����У�����Ҫ��������
	if (ucFlashStatus == ERASEPENDING)
	{
		EraseResume();
	}
}


static void vWriteABlockRecordForTest(unsigned char ucRecordType)
{	
	unsigned short i;
		
	tRecordForTest.usAttribute = VALID_RECORD;
	tRecordForTest.usID = ucRecordType;
	
	for (i=0; i<250; i++)
	{
		tRecordForTest.Data[i] = 0xFFFF;		
	}
	
	for (i=0; i<10; i++)
	{
		tRecordForTest.Data[i] = 0x06;		
	}
	
	ucWriteOneRecord(&tRecordForTest);
	
	/*
	for (i=1; i<128; i++)
	{
		ucWriteRecordAtOneLocation(&tRecordForTest,8,i);
		
		printk("============The %d record has been writen to flash========\n", i);
	}
	*/
	
}

static void vReadABlockRecordForTest(unsigned char ucRecordType)
{	
	unsigned short i, ret, usValue;
	
	unsigned short * p = (unsigned short *)&tRecordForTest;
	
	if ((ucRecordType>0) && (ucRecordType<EVENT_RECORD))
		tRecordForTest.usID = ucRecordType;
	else
		return;
	
	ret = ucReadOneRecord(&tRecordForTest);
	
	if (ret == 0x00)
	{
		for (i=0; i<(sizeof(FLASH_RECORD_T)/2); i++)
		{
			usValue = * (p+i);
			
			printk("=====The No %d data of %d record is %d========\n", i, ucRecordType, usValue);			
		}
	}
	else
	{
		printk("Read %d record is error ----%d\n", ucRecordType, ret);
	}
	/*
	for (i=1; i<128; i++)
	{
		ucWriteRecordAtOneLocation(&tRecordForTest,8,i);
		
		printk("============The %d record has been writen to flash========\n", i);
	}
	*/
	
}



static void vWriteAEventRecordForTest()
{	
	unsigned short i;
	unsigned char ucSize;
	EVENT_RECORD_T tEventRecord;
	EVENT_RECORD_T tEventRecordHistory;
	
	//unsigned short *usP1 = (unsigned short *)&tEventRecord;
	unsigned short *usP1 = (unsigned short *)&tEventRecordHistory;
	unsigned short *usP2 = (unsigned short *)&tRecordForTest;
		
	ucSize = sizeof(EVENT_RECORD_T)/2;
		
	tEventRecordHistory.usAttribute = VALID_RECORD;
	tEventRecordHistory.usID = EVENT_RECORD;
	
	tEventRecordHistory.tEventLog.usID = 0x04;
	tEventRecordHistory.tEventLog.usType = HISTORY_EVENT;
	
	tEventRecordHistory.tEventLog.tStartTime.ucWeeks = 0x02;
	tEventRecordHistory.tEventLog.tStartTime.ucCentraries = 0x02;
	tEventRecordHistory.tEventLog.tStartTime.ucYears = 0x02;
	tEventRecordHistory.tEventLog.tStartTime.ucMonths = 0x02;
	tEventRecordHistory.tEventLog.tStartTime.ucDates = 0x02;
	tEventRecordHistory.tEventLog.tStartTime.ucHours = 0x02;
	tEventRecordHistory.tEventLog.tStartTime.ucMinutes = 0x02;
	tEventRecordHistory.tEventLog.tStartTime.ucSeconds = 0x02;
	tEventRecordHistory.tEventLog.tStartTime.usMiliSecs = 0x02;
	
	tEventRecordHistory.tEventLog.tEndTime.ucWeeks = 0x1;
	tEventRecordHistory.tEventLog.tEndTime.ucCentraries = 0x1;
	tEventRecordHistory.tEventLog.tEndTime.ucYears = 0x1;
	tEventRecordHistory.tEventLog.tEndTime.ucMonths = 0x1;
	tEventRecordHistory.tEventLog.tEndTime.ucDates = 0x1;
	tEventRecordHistory.tEventLog.tEndTime.ucHours = 0x1;
	tEventRecordHistory.tEventLog.tEndTime.ucMinutes = 0x1;
	tEventRecordHistory.tEventLog.tEndTime.ucSeconds = 0x1;
	tEventRecordHistory.tEventLog.tEndTime.usMiliSecs = 0x1;
	
	
	//--------------------
	tEventRecord.usAttribute = VALID_RECORD;
	tEventRecord.usID = EVENT_RECORD;
	
	tEventRecord.tEventLog.usID = 0x07;
	tEventRecord.tEventLog.usType = CURRENT_EVENT;
	
	tEventRecord.tEventLog.tStartTime.ucWeeks = 0xFF;
	tEventRecord.tEventLog.tStartTime.ucCentraries = 0xFF;
	tEventRecord.tEventLog.tStartTime.ucYears = 0xFF;
	tEventRecord.tEventLog.tStartTime.ucMonths = 0xFF;
	tEventRecord.tEventLog.tStartTime.ucDates = 0xFF;
	tEventRecord.tEventLog.tStartTime.ucHours = 0xFF;
	tEventRecord.tEventLog.tStartTime.ucMinutes = 0xFF;
	tEventRecord.tEventLog.tStartTime.ucSeconds = 0xFF;
	tEventRecord.tEventLog.tStartTime.usMiliSecs = 0xFFFF;
	
	tEventRecord.tEventLog.tEndTime.ucWeeks = 0x1;
	tEventRecord.tEventLog.tEndTime.ucCentraries = 0x1;
	tEventRecord.tEventLog.tEndTime.ucYears = 0x1;
	tEventRecord.tEventLog.tEndTime.ucMonths = 0x1;
	tEventRecord.tEventLog.tEndTime.ucDates = 0x1;
	tEventRecord.tEventLog.tEndTime.ucHours = 0x1;
	tEventRecord.tEventLog.tEndTime.ucMinutes = 0x1;
	tEventRecord.tEventLog.tEndTime.ucSeconds = 0x1;
	tEventRecord.tEventLog.tEndTime.usMiliSecs = 0x1;
		
	for (i=0; i<250; i++)
	{
		tRecordForTest.Data[i] = 0xFFFF;		
	}
	
	for (i=0; i<ucSize; i++)
	{
		*(usP2 + i) = *(usP1 + i);
	}
		
	ucWriteOneRecord(&tRecordForTest);
	
	
	//mdelay(300);
	
	//ucWriteOneRecord(&tRecordForTest);
	
	/*
	for (i=1; i<128; i++)
	{
		ucWriteRecordAtOneLocation(&tRecordForTest,8,i);
		
		printk("============The %d record has been writen to flash========\n", i);
	}
	*/
	
}


static void vReadAEventRecordForTest()
{	
	unsigned short i, usCnt, usValue;
	unsigned char ucSize, ret;	
	EVENT_RECORD_T tEventRecord;
	
	unsigned short *usP2 = (unsigned short *)&tRecordForTest;
	unsigned short *usP1 = (unsigned short *)&tEventRecord;
	
	ucSize = sizeof(EVENT_RECORD_T)/2;
	
	usCnt = 0x07;
	
	tRecordForTest.Data[0] = usCnt;		
	ret = ucReadOneRecordFromFlash(&tRecordForTest);
	
	if (ret == 0x00)
	{
		for (i=0; i<ucSize; i++)
		{
			*(usP1 + i) = * (usP2+i);					
		}
		
		printk("Read EventLog record Index is ----%d\n", usCnt);
		
		printk("Event.Attr is    %d\n", tEventRecord.usAttribute);
		printk("Event.usID is    %d\n", tEventRecord.usID);
		
		printk("Event.tEventLog.usID is    %d\n", tEventRecord.tEventLog.usID);
		printk("Event.tEventLog.usType is    %d\n", tEventRecord.tEventLog.usType);
		
		printk("Event.tEventLog.tStartTime.ucWeeks is    %d\n", tEventRecord.tEventLog.tStartTime.ucWeeks);
		printk("Event.tEventLog.tStartTime.ucCentraries is    %d\n", tEventRecord.tEventLog.tStartTime.ucCentraries);
		printk("Event.tEventLog.tStartTime.ucYears is    %d\n", tEventRecord.tEventLog.tStartTime.ucYears);
		printk("Event.tEventLog.tStartTime.ucMonths is    %d\n", tEventRecord.tEventLog.tStartTime.ucMonths);
		printk("Event.tEventLog.tStartTime.ucDates is    %d\n", tEventRecord.tEventLog.tStartTime.ucDates);
		printk("Event.tEventLog.tStartTime.ucHours is    %d\n", tEventRecord.tEventLog.tStartTime.ucHours);
		printk("Event.tEventLog.tStartTime.ucMinutes is    %d\n", tEventRecord.tEventLog.tStartTime.ucMinutes);
		printk("Event.tEventLog.tStartTime.ucSeconds is    %d\n", tEventRecord.tEventLog.tStartTime.ucSeconds);
		printk("Event.tEventLog.tStartTime.usMiliSecs is    %d\n", tEventRecord.tEventLog.tStartTime.usMiliSecs);
		
		printk("Event.tEventLog.tEndTime.ucWeeks is    %d\n", tEventRecord.tEventLog.tEndTime.ucWeeks);
		printk("Event.tEventLog.tEndTime.ucCentraries is    %d\n", tEventRecord.tEventLog.tEndTime.ucCentraries);
		printk("Event.tEventLog.tEndTime.ucYears is    %d\n", tEventRecord.tEventLog.tEndTime.ucYears);
		printk("Event.tEventLog.tEndTime.ucMonths is    %d\n", tEventRecord.tEventLog.tEndTime.ucMonths);
		printk("Event.tEventLog.tEndTime.ucDates is    %d\n", tEventRecord.tEventLog.tEndTime.ucDates);
		printk("Event.tEventLog.tEndTime.ucHours is    %d\n", tEventRecord.tEventLog.tEndTime.ucHours);
		printk("Event.tEventLog.tEndTime.ucMinutes is    %d\n", tEventRecord.tEventLog.tEndTime.ucMinutes);
		printk("Event.tEventLog.tEndTime.ucSeconds is    %d\n", tEventRecord.tEventLog.tEndTime.ucSeconds);
		printk("Event.tEventLog.tEndTime.usMiliSecs is    %d\n", tEventRecord.tEventLog.tEndTime.usMiliSecs);
		
		
	}
	else
	{
		printk("Read EventLog record is error ----%d\n", ret);
	}
	/*
	for (i=1; i<128; i++)
	{
		ucWriteRecordAtOneLocation(&tRecordForTest,8,i);
		
		printk("============The %d record has been writen to flash========\n", i);
	}
	*/
	
}









static void vWriteAFaultRecordForTest()
{	
	unsigned short i;
	unsigned char ucSize;
	FAULT_RECORD_T tFaultRecord;
	
	unsigned short *usP1 = (unsigned short *)&tFaultRecord;
	unsigned short *usP2 = (unsigned short *)&tRecordForTest;
		
	ucSize = sizeof(FAULT_RECORD_T)/2;
		
	tFaultRecord.usAttribute = VALID_RECORD;
	tFaultRecord.usID = REC_FAULT_RECORD;
	
	tFaultRecord.tFaultLog.usID = 0x00;
	tFaultRecord.tFaultLog.usPiece = 0x01;
	
	tFaultRecord.tFaultLog.tStartTime.ucWeeks = 0x02;
	tFaultRecord.tFaultLog.tStartTime.ucCentraries = 0x02;
	tFaultRecord.tFaultLog.tStartTime.ucYears = 0x02;
	tFaultRecord.tFaultLog.tStartTime.ucMonths = 0x02;
	tFaultRecord.tFaultLog.tStartTime.ucDates = 0x02;
	tFaultRecord.tFaultLog.tStartTime.ucHours = 0x02;
	tFaultRecord.tFaultLog.tStartTime.ucMinutes = 0x02;
	tFaultRecord.tFaultLog.tStartTime.ucSeconds = 0x02;
	tFaultRecord.tFaultLog.tStartTime.usMiliSecs = 0x02;
	
	for (i=0; i<64; i++)
	{
		tFaultRecord.tFaultLog.usData[i] = 9;
	}
	
	for (i=0; i<250; i++)
	{
		tRecordForTest.Data[i] = 0xFFFF;		
	}
		
	for (i=0; i<ucSize; i++)
	{
		*(usP2 + i) = *(usP1 + i);		
	}
		
	
	ucWriteOneRecord(&tRecordForTest);
	
	
	//mdelay(300);
	
	//ucWriteOneRecord(&tRecordForTest);
	
	/*
	for (i=1; i<128; i++)
	{
		ucWriteRecordAtOneLocation(&tRecordForTest,8,i);
		
		printk("============The %d record has been writen to flash========\n", i);
	}
	*/
	
}


static void vReadAFaultRecordForTest()
{	
	unsigned short i, j, usCnt, usValue;
	unsigned char ucSize, ret;	
	FAULT_RECORD_T tFaultRecord;
	
	unsigned short *usP2 = (unsigned short *)&tRecordForTest;
	unsigned short *usP1 = (unsigned short *)&tFaultRecord;
	
	ucSize = sizeof(FAULT_RECORD_T)/2;
	
	usCnt = 0x02;
	
	tRecordForTest.usAttribute = VALID_RECORD;	
	tRecordForTest.usID = REC_FAULT_RECORD;	
	tRecordForTest.Data[0] = usCnt;		
	ret = ucReadOneRecordFromFlash(&tRecordForTest);
	
	if (ret == 0x00)
	{
		for (i=0; i<ucSize; i++)
		{
			*(usP1 + i) = * (usP2+i);					
		}
		
		printk("Read FaultRecord record Index is ----%d\n", usCnt);
		
		printk("Fault.Attr is    %d\n", tFaultRecord.usAttribute);
		printk("Fault.usID is    %d\n", tFaultRecord.usID);
		
		printk("Fault.tFaultLog.usID is    %d\n", tFaultRecord.tFaultLog.usID);
		printk("Fault.tFaultLog.usPiece is    %d\n", tFaultRecord.tFaultLog.usPiece);
		
		printk("Fault.tFaultLog.tStartTime.ucWeeks is    %d\n", tFaultRecord.tFaultLog.tStartTime.ucWeeks);
		printk("Fault.tFaultLog.tStartTime.ucCentraries is    %d\n", tFaultRecord.tFaultLog.tStartTime.ucCentraries);
		printk("Fault.tFaultLog.tStartTime.ucYears is    %d\n", tFaultRecord.tFaultLog.tStartTime.ucYears);
		printk("Fault.tFaultLog.tStartTime.ucMonths is    %d\n", tFaultRecord.tFaultLog.tStartTime.ucMonths);
		printk("Fault.tFaultLog.tStartTime.ucDates is    %d\n", tFaultRecord.tFaultLog.tStartTime.ucDates);
		printk("Fault.tFaultLog.tStartTime.ucHours is    %d\n", tFaultRecord.tFaultLog.tStartTime.ucHours);
		printk("Fault.tFaultLog.tStartTime.ucMinutes is    %d\n", tFaultRecord.tFaultLog.tStartTime.ucMinutes);
		printk("Fault.tFaultLog.tStartTime.ucSeconds is    %d\n", tFaultRecord.tFaultLog.tStartTime.ucSeconds);
		printk("Fault.tFaultLog.tStartTime.usMiliSecs is    %d\n", tFaultRecord.tFaultLog.tStartTime.usMiliSecs);
		
		for (j=0; j<64; j++)
		{
			printk("Fault.tFaultLog.usData[%d] is    %d\n", j, tFaultRecord.tFaultLog.usData[j]);
		}
		
	}
	else
	{
		printk("Read EventLog record is error ----%d\n", ret);
	}
	/*
	for (i=1; i<128; i++)
	{
		ucWriteRecordAtOneLocation(&tRecordForTest,8,i);
		
		printk("============The %d record has been writen to flash========\n", i);
	}
	*/
	
}







module_init (dataflash_device_init);
module_exit (uninst_dataflash_device);
MODULE_LICENSE("GPL");

MODULE_AUTHOR("Bulla@emersonnetwork.com.cn");
MODULE_DESCRIPTION("Direct character-device access to dataflash");


