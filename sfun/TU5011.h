#ifndef __DLL_TU5011_API_H__
#define __DLL_TU5011_API_H__

#ifdef __cplusplus
extern "C"{
#endif
#ifndef __PS_DEVICE_COMMON_DEFINE__
#define __PS_DEVICE_COMMON_DEFINE__

#define MAX_DEVICE_ID_LEN	200

#ifndef __PANSINO_SOLUTIONS_UNSIGNED_INT_32_TYPE_DEFINED__
#define __PANSINO_SOLUTIONS_UNSIGNED_INT_32_TYPE_DEFINED__
#if defined(__linux__) || defined(VXWORKS)
typedef unsigned int	uInt32;
#else
typedef unsigned long	uInt32;
#endif
#endif

typedef void* PS_DevObject;


/// 设备信息句柄
typedef PS_DevObject PS_DevInfoHandle;

/// 设备句柄
typedef PS_DevObject PS_DevHandle;

/// 设备属性
typedef enum
{
	ATTR_MANF_ID	=	0,			///< VendorID				__[unsigned short]
	ATTR_MODEL_CODE,				///< DeviceID				__[unsigned short]
	ATTR_SLOT,						///< slot					__[short]
	ATTR_PXI_CHASSIS,				///< chassis				__[short]
	ATTR_PXI_DEV_NUM,				///< device number			__[unsigned short]
	ATTR_PXI_FUNC_NUM,				///< function number		__[unsigned short]
	ATTR_PXI_BUS_NUM,				///< bus number				__[unsigned short]
	ATTR_RSRC_NAME					///< address string			__[char*]

}ATTRIBUTE;

/// 函数返回状态
enum RETURNERRORS
{
	PS_SUCCESS = 0,					///< 操作成功:0
	PS_FAILED,						///< 操作不成功:1
	PS_UNKNOWN_STATUS,				///< 未知状态:2
	PS_ERROR_STATUS,				///< 错误状态:3
	PS_INVALID_PARAM,				///< 无效参数值:4

	PS_TIME_OUT,					///< 操作超时:5

	PS_ERR_OPEN_KERNEL,				///< 打开核心设备失败:6
	PS_ERR_CLOSE_KERNEL,			///< 关闭核心设备失败:7

	PS_RCV_BUFFER_EMPTY,			///< 接收缓冲为空:8
	PS_RCV_BUFFER_FULL,				///< 接收缓冲已满:9
	PS_SND_BUFFER_EMPTY,			///< 发送缓冲为空:10
	PS_SND_BUFFER_FULL,				///< 发送缓冲已满:11

	PS_DEV_BUSY,					///< 设备忙:12
	PS_DEV_INVALID_STATUS,			///< 设备无效状态，该操作在设备状态上无效:13

	PS_DANGER_DEV_INPUT,			///< 设备的外部输入会危害设备安全:14
	PS_WAIT_EVENT_CANCELLED,		///< 退出等待事件:15

	PS_DATA_IS_NOT_ENOUGH,			///< 数据量不足:16
	PS_SPACE_IS_NOT_ENOUGH,			///< 空间不足:17

	PS_TRANSMIT_TIME_OUT,			///< 数据发送超时:18

	PS_ERROR_SRAM,					///< 设备SRAM出错:19
	PS_ERROR_LOOP					///< 网络建立失败:20
};

#endif

#define ErrorNotSetupEventHndlr									(-10000)				///< 使能事件失败
#define ErrorNotOpenMemacc										(-10001)				///< 打开物理内存失败
#define ErrorNotAllocPhyMemory									(-10002)				///< 分配物理内存失败

typedef enum _ParityBit
{
	Bit0 = 0x00000000,						/// 0校验，表示无论数据为多少，该位为0
	Bit1 = 0x00000010,						/// 1校验，表示无论数据为多少，该位为1
	Odd  = 0x00000020,						/// 奇校验
	Even = 0x00000030,						/// 偶校验
}ParityBit,*PParityBit;

typedef enum _StopBit
{
	stop1   = 0x00000000,					/// 停止位为1
	stop1_5 = 0x00000100,					/// 停止位为1.5
	stop2   = 0x00000200,					/// 停止位为2
}StopBit, *PStopBit;

typedef enum _DataBit
{
	Data5 = 0x00000001,						/// 数据长度位为5
	Data6 = 0x00000002,						/// 数据长度位为6
	Data7 = 0x00000003,						/// 数据长度位为7
	Data8 = 0x00000000,						/// 数据长度位为8
}DataBit, *PDataBit;

typedef enum _SerialMode
{
	RS232 = 0x00000000,						/// 串口模式RS232
	RS422 = 0x00001000,						/// 串口模式RS422
	RS485 = 0x00002000,						/// 串口模式RS485
}SerialMode, *PSerialMode;

typedef enum _InterType
{
	TxInter = 0,							/// 发送中断
	RxInter,								/// 接收中断
	OverflowInter,							/// 接收溢出中断
}InterType, *PInterType;

typedef struct _InterMode
{
	int		        channel;				/// 通道号
	int		        threshold;				/// 门限值
	InterType		intertype;				/// 中断类型
}InterMode, *PInterMode;

typedef struct _AsyncChannelMode
{		
	ParityBit				parityBit;			/// 校验位
	StopBit				stopBit;			/// 停止位
	DataBit				lengthBit;			/// 数据长度位
	SerialMode         	 	serialMode;			/// 串口模式
	int					ParityEnable;		/// 校验使能，为1使能
	int					TermiResistor;		/// 设置匹配电阻，为1设置，组网通信时，需要一路接收通道配置匹配电阻
	int					ChooseWrongData;	/// 校验错误数据保留，为1则保留
	uInt32		baudrate;				/// 波特率配置
}AsyncChannelMode, *PAsyncChannelMode;

/// 扫描PTU5011设备
int TU5011_ScanDevInfo(PS_DevInfoHandle* infoHandle, int *pnCnt, char srcName[]);

/// 查找下一个设备信息
int TU5011_FindNextDevInfo(PS_DevInfoHandle infoHandle, char srcName[]);

/// 释放释放PTU5011设备信息集
int TU5011_ReleaseDevice(PS_DevInfoHandle infoHandle);

/// 打开PTU5011设备句柄
int TU5011_OpenDevice(PS_DevInfoHandle infoHandle, PS_DevHandle *phDev);

/// 根据槽位打开板卡
int TU5011_OpenDeviceBySlot(short chassis, short slot, PS_DevHandle* phDev);

/// 根据设备资源名称打开板卡
int TU5011_OpenDeviceByName(char srcName[], PS_DevHandle* phDev);

/// 关闭PTU5011设备
int TU5011_CloseDevice(PS_DevHandle hDev);

/// 获得设备属性
int TU5011_GetAttribute(PS_DevHandle hDev, ATTRIBUTE attribute, void *attrState);

/// 获得FPGA版本信息
int TU5011_GetFPGAVersionn(PS_DevHandle hDev, uInt32* version);

/// 复位板卡
int TU5011_Reset(PS_DevHandle hDev);

/// 清除上溢标志
int TU5011_ClearOverflowIndicator(PS_DevHandle hDev, int channel);

/// 设置异步通道模式
int TU5011_SetAsyncChannelMode(PS_DevHandle hDev, int channel, AsyncChannelMode Mode);

/// 获取异步发送个数
int TU5011_GetCountOfTxData(PS_DevHandle hDev, int channel, uInt32* count);

/// 获取异步接收个数
int TU5011_GetCountOfRxData(PS_DevHandle hDev, int channel, uInt32* count);

/// 使能发送
int TU5011_EnableSend(PS_DevHandle hDev, int channel, int Enable);

/// 使能接收
int TU5011_EnableReceive(PS_DevHandle hDev, int channel, int Enable);

/// 清空发送FIFO
int TU5011_ClearSendFIFO(PS_DevHandle hDev, int channel);

/// 清空接收FIFO
int TU5011_ClearReceiveFIFO(PS_DevHandle hDev, int channel);

/// 发送数据
int TU5011_LoadAsyncData(PS_DevHandle hDev, int channel, unsigned char* pBuffer, uInt32 lengthToWrite, uInt32 *lengthWritten);

/// 读取数据
int TU5011_ReadAsyncData(PS_DevHandle hDev, int channel, unsigned char* pBuffer, uInt32 lengthToRead, uInt32 *lengthRead);

/// 设置自检模式使能
int TU5011_SelfTestEnable(PS_DevHandle hDev, uInt32 enable);

/// 设置自检通道
int TU5011_SetSelfTestChannel(PS_DevHandle hDev, int TxChannel, int RxChannel);

/// 刷新功能
int TU5011_RefreshFunction(PS_DevHandle hDev, uInt32 enable);

/// 设置上电状态
int TU5011_SetPowerState(PS_DevHandle hDev, uInt32 State);

/// 获取各通道上电状态时的通道配置
int TU5011_GetPowerStateOfChannel(PS_DevHandle hDev, int channel, AsyncChannelMode* Mode);

/// ********************************Interrupt Operation ***********************************
/// 等待中断
int TU5011_WaitForInterrupt(PS_DevHandle hDev, int interval);

/// 取消中断
int TU5011_CancelWaitForInterrupt(PS_DevHandle hDev);

/// 禁用中断
int TU5011_DisableInterrupt(PS_DevHandle hDev, int interLength, InterMode* intermode);

/// 设置中断模式
int TU5011_SetInterruptMode(PS_DevHandle hDev, int interLength, InterMode* interMode);

/// 获取中断模式
int TU5011_GetInterruptMode(PS_DevHandle hDev, int interLength, InterMode* interMode, int* lengthSet);

/// 获得中断个数
int TU5011_GetInterLength(PS_DevHandle hDev, int* interLength);

#ifdef __cplusplus
}
#endif
#endif

