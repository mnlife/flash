
/*********************************************************************************
*Flash (Data Record Management) driver                             					                                                          *
*Version: 1.0.0                                                                                                                 *
*2007/07/16 writed by Bulla Liu
*(C) Copyright 2007-2009, Emeson NetworkPower China Inc.     
*
*********************************************************************************/
#define DATA_FLASH_BASE 		0x30000000		/*KEYBOARD寄存器域基址*/

//typedef unsigned char __u8;
//typedef unsigned short __u16;
//typedef unsigned int __u32;

typedef struct _CFIDENT
{
  __u8  qry[3];  					/*应该为QRY三个字符*/
  __u16 P_ID;	   					/*命令集ID，应该为002（即AMD的命令集）*/
  __u16 P_ADR; 						/*该命令集算法信息的偏移地址*/
  __u8  DevSize;					/*容量大小单位（Bytes）：2的n次方*/
  __u16 InterfaceDesc;				/*可支持的数据总线类型*/
  __u16 MaxBufWriteSize;			/*最大支持一次性编程的缓冲区大小。*/
  __u8  NumEraseRegions;			/*不同大小的擦写块数量*/
  __u16	NumFirstRegion;				/*第一区域块的个数: + 1 */
  __u16	SizeFirstRegion;			/*第一区域每块的大小: * 256 bytes */
  __u16	NumSecondRegion;			/*第二区域块的个数: + 1 */
  __u16	SizeSecondRegion;			/*第二区域每块的大小: * 256 bytes */
  __u16	NumThirdRegion;				/*第三区域块的个数: + 1 */
  __u16	SizeThirdRegion;			/*第三区域每块的大小: * 256 bytes */
  __u16	NumFouthRegion;				/*第四区域块的个数: + 1 */
  __u16	SizeFouthRegion;			/*第四区域每块的大小: * 256 bytes */
}CFIDENT;


typedef struct _CFISYS
{
  __u8  TyProgram;  				/*典型的字节或者字编程时间：2的n次方*/
  __u8 	TyBlkErase;	   				/*典型的块擦除时间：2的n次方*/
  __u8  TyChipErase;				/*典型的片擦除时间：2的n次方*/
  __u8 	MaxProgrom;					/*最大的字节或者字编程时间：2的n次方*/
  __u8 	MaxBlkErase;				/*最大的块擦除时间：2的n次方*/
  __u8  MaxChipErase;				/*最大的片擦除时间：2的n次方*/
}CFISYS;

typedef struct __CFIFRI
{
  __u8  pri[3];						/*主命令集算法的标志，应该为"PRI"三个字符。*/
  __u8  MajorVersion;				/*该算法的大版本。*/
  __u8  MinorVersion;				/*该算法的小版本。*/
  __u8  EraseSuspend;				/*是否支持擦除悬挂。*/
  __u8  TopBottomType;				/*Boot BLOCK类型。*/
}CFIFRI;


#define DEVICE_TYPE_X8	(8 / 8)
#define DEVICE_TYPE_X16	(16 / 8)
#define DEVICE_TYPE_X32	(32 / 8)

/* Addresses */
#define ADDR_MANUFACTURER				0x0000
#define ADDR_DEVICE_ID					0x0001
#define ADDR_SECTOR_LOCK				0x0002
#define ADDR_HANDSHAKE					0x0003
#define ADDR_CFI_READ					0x0055

#define ADDR_UNLOCK_1					0x0555
#define ADDR_UNLOCK_2					0x02AA

/* Commands */
#define CMD_CFI_READ					0x0098

#define CMD_UNLOCK_DATA_1				0x00AA
#define CMD_UNLOCK_DATA_2				0x0055

#define CMD_RESET_DATA					0x00F0
#define CMD_MANUFACTURER_UNLOCK_DATA	0x0090
#define CMD_PROGRAM_UNLOCK_DATA			0x00A0
#define CMD_CHIP_SECTOR_ERASE			0x0080
#define CMD_SECTOR_ERASE_UNLOCK_DATA	0x0030
#define CMD_CHIP_ERASE_UNLOCK_DATA		0x0010

#define CMD_EVENT_RECORD_ERASE			0x0102
#define CMD_GET_FLASH_OPRATION_STATE	0x0101

#define CMD_UNLOCK_BYPASS_MODE			0x0020

#define CMD_ERASE_SUSPEND				0x00B0
#define CMD_ERASE_RESUME				0x0030

/* Manufacturers */
#define MANUFACTURER_AMD				0x0001
#define MANUFACTURER_ATMEL				0x001F
#define MANUFACTURER_FUJITSU			0x0004
#define MANUFACTURER_ST					0x0020
#define MANUFACTURER_SST				0x00BF
#define MANUFACTURER_TOSHIBA			0x0098

/* Flash status */
#define UKNOWN							0x0000
#define READING							0x0001
#define PROGRAMING						0x0002
#define BLOCKERASING					0x0003
#define CHIPERASING						0x0004
#define ERASEPENDING					0x0005

#define TOP_FLASH						0x0003
#define BOTTOM_FLASH					0x0002

#define ERASE_SUSPEND_NO				0x0000
#define ERASE_SUSPEND_OR				0x0001
#define ERASE_SUSPEND_RW				0x0002


#define ERASE_CHECK_PERIOD				10        /*100ms,每100ms就会检查是否擦除完成，以及检查是否有Dirty扇区需要擦除*/


//////////////////////////////////////////////////////
// 调试信息的打印处理 相关宏定义
/////////////////////////////////////////////////////
#define	FILE_NAME	__FILE__
#define	LINE_NUM	__LINE__

#define	PRINT_TRACE(x)		x ? (void)0 : (void)printk
