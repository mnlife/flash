
/*********************************************************************************
*Flash (Data Record Management) driver                             					                                                          *
*Version: 1.0.0                                                                                                                 *
*2007/07/16 writed by Bulla Liu
*(C) Copyright 2007-2009, Emeson NetworkPower China Inc.     
*
*********************************************************************************/
#define DATA_FLASH_BASE 		0x30000000		/*KEYBOARD�Ĵ������ַ*/

//typedef unsigned char __u8;
//typedef unsigned short __u16;
//typedef unsigned int __u32;

typedef struct _CFIDENT
{
  __u8  qry[3];  					/*Ӧ��ΪQRY�����ַ�*/
  __u16 P_ID;	   					/*���ID��Ӧ��Ϊ002����AMD�������*/
  __u16 P_ADR; 						/*������㷨��Ϣ��ƫ�Ƶ�ַ*/
  __u8  DevSize;					/*������С��λ��Bytes����2��n�η�*/
  __u16 InterfaceDesc;				/*��֧�ֵ�������������*/
  __u16 MaxBufWriteSize;			/*���֧��һ���Ա�̵Ļ�������С��*/
  __u8  NumEraseRegions;			/*��ͬ��С�Ĳ�д������*/
  __u16	NumFirstRegion;				/*��һ�����ĸ���: + 1 */
  __u16	SizeFirstRegion;			/*��һ����ÿ��Ĵ�С: * 256 bytes */
  __u16	NumSecondRegion;			/*�ڶ������ĸ���: + 1 */
  __u16	SizeSecondRegion;			/*�ڶ�����ÿ��Ĵ�С: * 256 bytes */
  __u16	NumThirdRegion;				/*���������ĸ���: + 1 */
  __u16	SizeThirdRegion;			/*��������ÿ��Ĵ�С: * 256 bytes */
  __u16	NumFouthRegion;				/*���������ĸ���: + 1 */
  __u16	SizeFouthRegion;			/*��������ÿ��Ĵ�С: * 256 bytes */
}CFIDENT;


typedef struct _CFISYS
{
  __u8  TyProgram;  				/*���͵��ֽڻ����ֱ��ʱ�䣺2��n�η�*/
  __u8 	TyBlkErase;	   				/*���͵Ŀ����ʱ�䣺2��n�η�*/
  __u8  TyChipErase;				/*���͵�Ƭ����ʱ�䣺2��n�η�*/
  __u8 	MaxProgrom;					/*�����ֽڻ����ֱ��ʱ�䣺2��n�η�*/
  __u8 	MaxBlkErase;				/*���Ŀ����ʱ�䣺2��n�η�*/
  __u8  MaxChipErase;				/*����Ƭ����ʱ�䣺2��n�η�*/
}CFISYS;

typedef struct __CFIFRI
{
  __u8  pri[3];						/*������㷨�ı�־��Ӧ��Ϊ"PRI"�����ַ���*/
  __u8  MajorVersion;				/*���㷨�Ĵ�汾��*/
  __u8  MinorVersion;				/*���㷨��С�汾��*/
  __u8  EraseSuspend;				/*�Ƿ�֧�ֲ������ҡ�*/
  __u8  TopBottomType;				/*Boot BLOCK���͡�*/
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


#define ERASE_CHECK_PERIOD				10        /*100ms,ÿ100ms�ͻ����Ƿ������ɣ��Լ�����Ƿ���Dirty������Ҫ����*/


//////////////////////////////////////////////////////
// ������Ϣ�Ĵ�ӡ���� ��غ궨��
/////////////////////////////////////////////////////
#define	FILE_NAME	__FILE__
#define	LINE_NUM	__LINE__

#define	PRINT_TRACE(x)		x ? (void)0 : (void)printk
