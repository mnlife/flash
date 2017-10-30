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

//�����������ݽṹ
typedef struct _BLOCK_ATTR_T
{
    unsigned short usBlockState;		//������״̬��NOT USED, ACTIVE, BAD
    unsigned short usRecordType;		//�����洢�ļ�¼����: 
    unsigned short usPages;    			//������ҳ����ÿҳ512 UINT16
}PACKED BLOCK_ATTR_T;

//����������¼���ݽṹ
typedef struct _BASIC_FLASH_INFO_T
{
	unsigned short usIdentifyInfo[3];    
    
    unsigned short usFlashCapacity;			//��λΪ0x400�ֽ�
    unsigned short usBusWidth;
    unsigned short usHasSuspend;
    
    unsigned short usReserved[20];    
    
    unsigned short usTotalBlocks;
    
    BLOCK_ATTR_T tBlockAttrList[MAX_BLOCKS];
}PACKED BASIC_FLASH_INFO_T;


//����������¼�ı�ʶ��궨��
#define FLASH_IDENTIFY1		0x3333
#define FLASH_IDENTIFY2		0x5555
#define FLASH_IDENTIFY3		0xAAAA

//����������¼�ı�ʶ��洢ƫ����
#define IDENTIFY1_ADDR		0x0000
#define IDENTIFY2_ADDR		0x0001
#define IDENTIFY3_ADDR		0x0002


/*����״̬��ʶ*/
#define BLOCK_NOT_USED		0xFFFF			// the Block is not used
#define BLOCK_ACTIVE		0xCCCC			// the Block is active to use
#define BLOCK_BAD			0x0000			// the Block is bad


/*��¼���Ͷ���*/
#define NOT_USED							0xFFFF
#define UNICODE_FONT						0xFFFE
#define FLASH_BASIC_INFO					0x0000		//Flash������Ϣ����

#define SYS_BASIC_SET_RECORD				0x001		//ϵͳ�������ò���
#define SYS_MON_SET_RECORD				0x002 		//ϵͳ������ò���
#define SYS_BATT1_SET_RECORD				0x003		///�����1���ò���
#define SYS_BATT2_SET_RECORD				0x004		//�����2���ò���
#define SYS_DSP_SET_RECORD					0x005		//ϵͳDSP�ۺ����ò���

#define MOD1_REC_DSP_SET_RECORD			0x006		//ģ��1�������ò���
#define MOD2_REC_DSP_SET_RECORD			0x007		//ģ��2�������ò���
#define MOD3_REC_DSP_SET_RECORD			0x008		//ģ��3�������ò���
#define MOD4_REC_DSP_SET_RECORD			0x009		//ģ��4�������ò���
#define MOD5_REC_DSP_SET_RECORD			0x00a		//ģ��51�������ò���
#define MOD6_REC_DSP_SET_RECORD			0x00b		//ģ��6�������ò���
#define MOD7_REC_DSP_SET_RECORD			0x00c		//ģ��7�������ò���
#define MOD8_REC_DSP_SET_RECORD			0x00d		//ģ��8�������ò���
#define MOD9_REC_DSP_SET_RECORD			0x00e		//ģ��9�������ò���
#define MOD10_REC_DSP_SET_RECORD			0x00f		//ģ��10�������ò���

#define MOD1_INV_DSP_SET_RECORD			0x0010		//ģ��1������ò���
#define MOD2_INV_REC_DSP_SET_RECORD		0x0011		//ģ��2������ò���
#define MOD3_INV_REC_DSP_SET_RECORD		0x0012		//ģ��3������ò���
#define MOD4_INV_REC_DSP_SET_RECORD		0x0013		//ģ��4������ò���
#define MOD5_INV_REC_DSP_SET_RECORD		0x0014		//ģ��51������ò���
#define MOD6_INV_REC_DSP_SET_RECORD		0x0015		//ģ��6������ò���
#define MOD7_INV_REC_DSP_SET_RECORD		0x0016		//ģ��7������ò���
#define MOD8_INV_REC_DSP_SET_RECORD		0x0017		//ģ��8������ò���
#define MOD9_INV_REC_DSP_SET_RECORD		0x0018		//ģ��9������ò���
#define MOD10_INV_REC_DSP_SET_RECORD		0x0019		//ģ��10������ò���


#define AC_SET_RECORD						0x001a		//AC������������
#define AC_HISTORY_RECORD					0x001b		//AC��¼��������

#define BATT1_SPECIAL_CURVE_RECORD		0x001c		//�����1ר��������������
#define BATT2_SPECIAL_CURVE_RECORD		0x001d		//�����2ר��������������

//#define BATT_GENERAL_CURVE_RECORD		0x000A		//���ͨ��������������

#define BATT1_REAL_CURVE_RECORD			0x001e		//�����1ʵʱ������������
#define BATT2_REAL_CURVE_RECORD			0x001f		//�����2ʵʱ������������

//--20090730,����һ��BLOCK����Щ��ɢ�Ĳ���,Ŀǰ����һ������ʱ��
#define RESERVED_MON_RECORD				0x0020

#define EVENT_RECORD						0x0021		//�¼���¼��������
#define REC_FAULT_RECORD					0x0022		//�������ϼ�¼��������
#define INV_FAULT_RECORD					0x0023		//�����ϼ�¼��������


//������¼���͵�����
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

//ͨ�ü�¼���ݽṹ
typedef struct _FLASH_RECORD_T
{
	unsigned short  usAttribute;		/*����*/
	unsigned short  usID; 				/*��¼��(13�м�¼�е�һ��)*/
	unsigned short  Data[250]; 			/*�������ݣ�����ID�����Ͳ�ͬ��ʵ�ʵ����ݳ��Ƚ����в�ͬ��*/
} PACKED FLASH_RECORD_T;


//ͨ�ü�¼��Ч�Ա�ʶ�궨��
#define VACANT_RECORD						0xFFFF
#define INVALID_RECORD						0xCCCC
#define VALID_RECORD						0x0000

//�¼���¼ר����Ч�Ա�ʶ�궨��
#define INVALID_CURRENT_EVENT_RECORD		0xCCCC
#define VALID_CURRENT_EVENT_RECORD		0xCC00
#define INVALID_HISTORY_EVENT_RECORD		0xC000
#define VALID_HISTORY_EVENT_RECORD		0x0000


//��¼ռ�õ���������ʹ��״̬ƥ�����ݽṹ
typedef struct _BLOCK_MAPPING_T
{
	unsigned char 	ucBlockNum;				//ռ�õ�������
	unsigned char 	ucBlockStatus;			//ռ�õ�����ʹ��״̬: ������ĺ궨��
	unsigned short 	usBadRecordNum;			//ռ�õ������У���⵽�Ļ���¼����
}PACKED BLOCK_MAPPING_T;

//��¼ռ�õ�����ʹ��״̬�궨��
#define BLOCK_NO_DATA			0xFF		//��������������
#define BLOCK_HAS_DATA			0xCC		//��������������
#define BLOCK_IS_DIRTY			0x55		//��������������
#define BLOCK_ERASING			0x00		//���������ڱ�����


//��¼�������ݽṹ
typedef struct _RECORD_MANAGE_T
{
	BLOCK_MAPPING_T	tUsedBlocks[5];			//�������������״̬ƥ���
	unsigned char 		ucCurrentBlock;			//��ǰ���¼�¼ʹ�õ�������
	unsigned short		usNewestPointer;		//�¼�¼��дָ��
	unsigned short 		usMaxNumOfRecord;		//һ���������ܴ洢������¼��
	unsigned short		usPrewarnNum;			//�����洢�ļ�¼���Ѵ���Ч��,�Ӷ����ѱ����������Ա�עΪ�ߵ�Ԥ�澯��ֵ
	unsigned char   		ucSizeofRecord;			//�ü�¼�Ĵ�С, ��λΪUINT16
}PACKED RECORD_MANAGE_T;

/********************************************************************/
/*               ���¼������ض���								*/
/********************************************************************/
typedef struct _BLOCK_RECORD_T
{
	unsigned short  usAttribute;		/*����*/
	unsigned short  usID; 				/*��¼��(���¼�е�һ��)*/
	unsigned short  Data[250]; 			/*�������ݣ�����ID�����Ͳ�ͬ��ʵ�ʵ����ݳ��Ƚ����в�ͬ��*/
}PACKED BLOCK_RECORD_T;

#define MAX_BLOCK_PREWARN_NUM_FOR_TEST 	100
#define MAX_BLOCK_NUM_FOR_TEST 			60

#define MAX_BLOCK_PREWARN_NUM			100

/********************************************************************/
/*               �¼���¼������ض���								*/
/********************************************************************/
typedef struct _DATE_TIME_T
{
	unsigned char	ucWeeks;			/*�ڼ�������*/
	unsigned char	ucCentraries;		/*����*/
	unsigned char	ucYears;			/*ĳ�������еĵڼ���*/
	unsigned char	ucMonths;
	unsigned char	ucDates;
	unsigned char 	ucHours;
	unsigned char 	ucMinutes;
	unsigned char	ucSeconds;
	unsigned short	usMiliSecs;			/*����ֵ*/
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
	unsigned short  usAttribute;		//����
	unsigned short  usID; 				//��¼��(�̶�ΪEVENT_RECORD)
	EVENT_LOG_T		tEventLog;			
}PACKED EVENT_RECORD_T;

#define MAX_EVENT_PREWARN_NUM_FOR_TEST 	80
#define MAX_EVENT_NUM_FOR_TEST 			160

#define MAX_EVENT_PREWARN_NUM			2048

/********************************************************************/
/*               ���ϼ�¼������ض���								*/
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
	unsigned short  usAttribute;			//����
	unsigned short  usID; 					//��¼��(�̶�ΪREC_FAULT_RECORD or INV_FAULT_RECORD)
	FAULT_LOG_T		tFaultLog;			
}PACKED FAULT_RECORD_T;

#define MAX_FAULT_PREWARN_NUM_FOR_TEST 	256
#define MAX_FAULT_NUM_FOR_TEST 			512

#define MAX_FAULT_PREWARN_NUM			256

///////////////////////////////////////////////////////////////////////////////////
//	���ִ��󷵻�ֵ�Ķ���
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