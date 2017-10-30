/*********************************************************************************  
*Flash (Data Record Management) driver                             					
*Version: 1.0.0                                                                     
*2007/07/16 writed by Bulla Liu                                                     
*(C) Copyright 2007-2009, Emeson NetworkPower China Inc.                            
*                                                                                   
*                                                                                   
*********************************************************************************/  

#ifndef PACKED
#define PACKED __attribute__( ( packed, aligned(1) ) )
#endif

#define MAX_BLOCKS		200

//扇区属性数据结构
typedef struct _BLOCK_ATTR_T
{
    unsigned short usBlockState;		//扇区的状态：NOT USED, ACTIVE, BAD
    unsigned short usRecordType;		//扇区存储的记录类型: 
    unsigned short usPages;    			//扇区的页数，每页512 UINT16
}PACKED BLOCK_ATTR_T;

//基本扇区记录数据结构
typedef struct _BASIC_FLASH_INFO_T
{
	unsigned short usIdentifyInfo[3];    
    
    unsigned short usFlashCapacity;			//单位为0x400字节
    unsigned short usBusWidth;
    unsigned short usHasSuspend;
    
    unsigned short usReserved[20];    
    
    unsigned short usTotalBlocks;
    
    BLOCK_ATTR_T tBlockAttrList[MAX_BLOCKS];
}PACKED BASIC_FLASH_INFO_T;


//基本扇区记录的辨识码宏定义
#define FLASH_IDENTIFY1		0x3333
#define FLASH_IDENTIFY2		0x5555
#define FLASH_IDENTIFY3		0xAAAA

//基本扇区记录的辨识码存储偏移量
#define IDENTIFY1_ADDR		0x0000
#define IDENTIFY2_ADDR		0x0001
#define IDENTIFY3_ADDR		0x0002


/*扇区状态标识*/
#define BLOCK_NOT_USED		0xFFFF			// the Block is not used
#define BLOCK_ACTIVE		0xCCCC			// the Block is active to use
#define BLOCK_BAD			0x0000			// the Block is bad


/*记录类型定义*/
#define NOT_USED							0xFFFF
#define UNICODE_FONT						0xFFFE
#define FLASH_BASIC_INFO					0x0000		//Flash基本信息类型

#define SYS_BASIC_SET_RECORD				0x001		//系统基本设置参数
#define SYS_MON_SET_RECORD				0x002 		//系统监控设置参数
#define SYS_BATT1_SET_RECORD				0x003		///电池组1设置参数
#define SYS_BATT2_SET_RECORD				0x004		//电池组2设置参数
#define SYS_DSP_SET_RECORD					0x005		//系统DSP综合设置参数

#define MOD1_REC_DSP_SET_RECORD			0x006		//模块1整流设置参数
#define MOD2_REC_DSP_SET_RECORD			0x007		//模块2整流设置参数
#define MOD3_REC_DSP_SET_RECORD			0x008		//模块3整流设置参数
#define MOD4_REC_DSP_SET_RECORD			0x009		//模块4整流设置参数
#define MOD5_REC_DSP_SET_RECORD			0x00a		//模块51整流设置参数
#define MOD6_REC_DSP_SET_RECORD			0x00b		//模块6整流设置参数
#define MOD7_REC_DSP_SET_RECORD			0x00c		//模块7整流设置参数
#define MOD8_REC_DSP_SET_RECORD			0x00d		//模块8整流设置参数
#define MOD9_REC_DSP_SET_RECORD			0x00e		//模块9整流设置参数
#define MOD10_REC_DSP_SET_RECORD			0x00f		//模块10整流设置参数

#define MOD1_INV_DSP_SET_RECORD			0x0010		//模块1逆变设置参数
#define MOD2_INV_REC_DSP_SET_RECORD		0x0011		//模块2逆变设置参数
#define MOD3_INV_REC_DSP_SET_RECORD		0x0012		//模块3逆变设置参数
#define MOD4_INV_REC_DSP_SET_RECORD		0x0013		//模块4逆变设置参数
#define MOD5_INV_REC_DSP_SET_RECORD		0x0014		//模块51逆变设置参数
#define MOD6_INV_REC_DSP_SET_RECORD		0x0015		//模块6逆变设置参数
#define MOD7_INV_REC_DSP_SET_RECORD		0x0016		//模块7逆变设置参数
#define MOD8_INV_REC_DSP_SET_RECORD		0x0017		//模块8逆变设置参数
#define MOD9_INV_REC_DSP_SET_RECORD		0x0018		//模块9逆变设置参数
#define MOD10_INV_REC_DSP_SET_RECORD		0x0019		//模块10逆变设置参数


#define AC_SET_RECORD						0x001a		//AC设置数据类型
#define AC_HISTORY_RECORD					0x001b		//AC记录数据类型

#define BATT1_SPECIAL_CURVE_RECORD		0x001c		//电池组1专用曲线数据类型
#define BATT2_SPECIAL_CURVE_RECORD		0x001d		//电池组2专用曲线数据类型

//#define BATT_GENERAL_CURVE_RECORD		0x000A		//电池通用曲线数据类型

#define BATT1_REAL_CURVE_RECORD			0x001e		//电池组1实时曲线数据类型
#define BATT2_REAL_CURVE_RECORD			0x001f		//电池组2实时曲线数据类型

//--20090730,增加一个BLOCK来存些零散的参数,目前增加一个运行时间
#define RESERVED_MON_RECORD				0x0020

#define EVENT_RECORD						0x0021		//事件记录数据类型
#define REC_FAULT_RECORD					0x0022		//整流故障记录数据类型
#define INV_FAULT_RECORD					0x0023		//逆变故障记录数据类型


//各个记录类型的名称
const unsigned char cucRecordName[INV_FAULT_RECORD+1][30] = 
{
	"Flash Basic Info Record",
	"System Basic Setting Record",
	"Monitor Setting Record",
	"Battery1 Setting Record",
	"Battery2 Setting Record",
	"System DSP Setting Record",
	"MOD1 Rectifier Setting Record",
	"MOD2 Rectifier Setting Record",
	"MOD3 Rectifier Setting Record",
	"MOD4 Rectifier Setting Record",
	"MOD5 Rectifier Setting Record",
	"MOD6 Rectifier Setting Record",
	"MOD7 Rectifier Setting Record",
	"MOD8 Rectifier Setting Record",
	"MOD9 Rectifier Setting Record",
	"MOD10 Rectifier Setting Record",
	"MOD1 Inverter Setting Record",
	"MOD2 Inverter Setting Record",
	"MOD3 Inverter Setting Record",
	"MOD4 Inverter Setting Record",
	"MOD5 Inverter Setting Record",
	"MOD6 Inverter Setting Record",
	"MOD7 Inverter Setting Record",
	"MOD8 Inverter Setting Record",
	"MOD9 Inverter Setting Record",
	"MOD10 Inverter Setting Record",
	"AC Setting Record",
	"AC History Record",
	"Battery1 Special Curve Record",
	"Battery2 Special Curve Record",
	"Battery1 Real Curve Record",
	"Battery2 Real Curve Record",
	"Reserved Mon Record",
	"Event Log Record",
	"Rectifier Fault Record",
	"Inverter Fault Record"
};

//通用记录数据结构
typedef struct _FLASH_RECORD_T
{
	unsigned short  usAttribute;		/*属性*/
	unsigned short  usID; 				/*记录号(13中记录中的一种)*/
	unsigned short  Data[250]; 			/*具体数据，根据ID的类型不同，实际的数据长度将会有不同。*/
} PACKED FLASH_RECORD_T;


//通用记录有效性标识宏定义
#define VACANT_RECORD						0xFFFF
#define INVALID_RECORD						0xCCCC
#define VALID_RECORD						0x0000

//事件记录专用有效性标识宏定义
#define INVALID_CURRENT_EVENT_RECORD		0xCCCC
#define VALID_CURRENT_EVENT_RECORD		0xCC00
#define INVALID_HISTORY_EVENT_RECORD		0xC000
#define VALID_HISTORY_EVENT_RECORD		0x0000


//记录占用的扇区及其使用状态匹配数据结构
typedef struct _BLOCK_MAPPING_T
{
	unsigned char 	ucBlockNum;				//占用的扇区号
	unsigned char 	ucBlockStatus;			//占用的扇区使用状态: 见下面的宏定义
	unsigned short 	usBadRecordNum;			//占用的扇区中，检测到的坏记录数量
}PACKED BLOCK_MAPPING_T;

//记录占用的扇区使用状态宏定义
#define BLOCK_NO_DATA			0xFF		//该扇区暂无数据
#define BLOCK_HAS_DATA			0xCC		//该扇区已有数据
#define BLOCK_IS_DIRTY			0x55		//该扇区数据已赃
#define BLOCK_ERASING			0x00		//该扇区正在被擦除


//记录管理数据结构
typedef struct _RECORD_MANAGE_T
{
	BLOCK_MAPPING_T	tUsedBlocks[5];			//分配的扇区及其状态匹配表
	unsigned char 		ucCurrentBlock;			//当前最新记录使用的扇区号
	unsigned short		usNewestPointer;		//新记录的写指针
	unsigned short 		usMaxNumOfRecord;		//一个扇区所能存储的最大记录数
	unsigned short		usPrewarnNum;			//扇区存储的记录数已达有效数,从而提醒备用扇区可以标注为赃的预告警限值
	unsigned char   		ucSizeofRecord;			//该记录的大小, 单位为UINT16
}PACKED RECORD_MANAGE_T;

/********************************************************************/
/*               块记录数据相关定义								*/
/********************************************************************/
typedef struct _BLOCK_RECORD_T
{
	unsigned short  usAttribute;		/*属性*/
	unsigned short  usID; 				/*记录号(块记录中的一种)*/
	unsigned short  Data[250]; 			/*具体数据，根据ID的类型不同，实际的数据长度将会有不同。*/
}PACKED BLOCK_RECORD_T;

#define MAX_BLOCK_PREWARN_NUM_FOR_TEST 	100
#define MAX_BLOCK_NUM_FOR_TEST 			60

#define MAX_BLOCK_PREWARN_NUM			100

/********************************************************************/
/*               事件记录数据相关定义								*/
/********************************************************************/
typedef struct _DATE_TIME_T
{
	unsigned char	ucWeeks;			/*第几个星期*/
	unsigned char	ucCentraries;		/*世纪*/
	unsigned char	ucYears;			/*某个世纪中的第几年*/
	unsigned char	ucMonths;
	unsigned char	ucDates;
	unsigned char 	ucHours;
	unsigned char 	ucMinutes;
	unsigned char	ucSeconds;
	unsigned short	usMiliSecs;			/*毫秒值*/
}PACKED DATE_TIME_T;

typedef struct _EVENT_LOG_T
{
	unsigned short  usModuleIdx;			//Module ID	
	unsigned short 	usID;				//Event ID
	unsigned short 	usType;				//Event Type
	
	DATE_TIME_T		tStartTime;
	DATE_TIME_T		tEndTime;	
}PACKED EVENT_LOG_T;

#define CURRENT_EVENT 0x5555
#define HISTORY_EVENT 0x1111

typedef struct _EVENT_RECORD_T
{
	unsigned short  usAttribute;		//属性
	unsigned short  usID; 				//记录号(固定为EVENT_RECORD)
	EVENT_LOG_T		tEventLog;			
}PACKED EVENT_RECORD_T;

#define MAX_EVENT_PREWARN_NUM_FOR_TEST 	80
#define MAX_EVENT_NUM_FOR_TEST 			160

#define MAX_EVENT_PREWARN_NUM			2048

/********************************************************************/
/*               故障记录数据相关定义								*/
/********************************************************************/

typedef struct _FAULT_LOG_T
{
	unsigned short 	usID;					//FAULT Index ID	(1-4)
	unsigned short 	usPiece;				//FAULT Frame		(1-32)
	
	DATE_TIME_T		tStartTime;	
	
	unsigned short 	usData[64];				//FAULT Data	
}PACKED FAULT_LOG_T;

typedef struct _FAULT_RECORD_T
{
	unsigned short  usAttribute;			//属性
	unsigned short  usID; 					//记录号(固定为REC_FAULT_RECORD or INV_FAULT_RECORD)
	FAULT_LOG_T		tFaultLog;			
}PACKED FAULT_RECORD_T;

#define MAX_FAULT_PREWARN_NUM_FOR_TEST 	256
#define MAX_FAULT_NUM_FOR_TEST 			512

#define MAX_FAULT_PREWARN_NUM			256

///////////////////////////////////////////////////////////////////////////////////
//	各种错误返回值的定义
///////////////////////////////////////////////////////////////////////////////////
#define		ERROR_READ_SYSCALL_COUNT	0x31
#define		ERROR_READ_SYSCALL_VERIRY	0x32

#define		ERROR_WRITE_SYSCALL_COUNT	0x41
#define		ERROR_WRITE_SYSCALL_VERIRY	0x42

#define		ERROR_IOCTL_SYSCALL_CMD		0x51
#define		ERROR_IOCTL_SYSCALL_ARG		0x52
#define		ERROR_IOCTL_SYSCALL_BUSY	0x53

#define		ERROR_READ_RECORD_ID		0x61
#define		ERROR_WRITE_RECORD_ID		0x62
#define		ERROR_READ_RECORD_INDEX		0x63

#define		ERROR_WRITE_RECORD_BUSY		0x66

#define		ERROR_READ_RECORD_NONE		0x81

#define		ERROR_WRITE_EVENT_HIS_NONE	0xA3


#define		MAX_WRITE_ERROR_COUNT		0x02
#define		MAX_READ_ERROR_COUNT		0x02

#define 		MAX_EVENT_COUNT 				2048