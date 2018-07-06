#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "TU5011.h"

#define LENGTH 10
unsigned char pBuffer[LENGTH];
unsigned char ReadpBuffer[LENGTH];

int SendChannel = 0;				//发送通道,对应线缆的 1 
int ReceiveChannel = 1;				// 接收通道，对应线缆的 2

int main()
{
	int 				result = PS_SUCCESS;
	PS_DevHandle 		handle;
	AsyncChannelMode 	ModeAsy;
	int 				loop = 0,i=0;
	unsigned long 		lengthWritten = 0;
	int 				Tx_data_cnt = 0;
	int 				Rd_data_cnt = 0;				//接收总个数
	unsigned long 		RVCount = 0;
	unsigned long 		lengthRead = 0;
	
	// 根据资源名称打开板卡，lspci | grep 8038 可查看，
	//如18:0d.0 Bridge: Unknown device 8038:0db6 (rev ba)即对应 24 13
	result = TU5011_OpenDeviceByName("PXI13::13::INSTR", &handle); 
	if(result != 0)
	{
		printf("Open device is error，The error code is %d.\n", result);
		return -1;
	}
	

	/// 清空接收FIFO和发送FIFO
	TU5011_ClearReceiveFIFO(handle, ReceiveChannel);
	TU5011_ClearSendFIFO(handle, SendChannel);
	

	//异步测试 - 接收
	ModeAsy.baudrate = 9600;
	ModeAsy.lengthBit = Data8;
	ModeAsy.parityBit = Odd;
	ModeAsy.stopBit = stop1;
	ModeAsy.ParityEnable = 1;
	ModeAsy.serialMode = RS485;
	ModeAsy.TermiResistor = 0;
	ModeAsy.ChooseWrongData = 0;

	/// 设置发送通道模式
	result = TU5011_SetAsyncChannelMode(handle, SendChannel, ModeAsy);
	/// 设置接收通道模式
	result = TU5011_SetAsyncChannelMode(handle, ReceiveChannel, ModeAsy);
	
	
	/// 使能通道发送和接收
	result = TU5011_EnableReceive(handle, ReceiveChannel, 1);

	///设置自检
	//result = TU5011_SetSelfTestChannel(handle, SendChannel, ReceiveChannel);	/// 自检模块必须放置于设置通道模式的后面，若两通道设置不一致，此模块会出现错误提醒
	//Sleep(100);
	//if(result != PS_SUCCESS)												/// 注意：若两通道参数设置不一致，则不能执行自检功能，容易烧坏板卡
	//{
	//	return result;
	//}
	//result = TU5011_SelfTestEnable(handle, 1);								/// 自检功能使能，放置于自检通道选择的后面，若不再选用自检功能，需要将使能复位为0
	//Sleep(1000);

	/// 使能通道发送和接收
	result = TU5011_EnableSend(handle, SendChannel, 1);
	for(i=0; i<LENGTH; i++)
	{
		pBuffer[i] = i;
	}
	//数据载入FIFO
	result = TU5011_LoadAsyncData(handle, SendChannel, pBuffer, LENGTH, &lengthWritten);
	printf("The number of data to be sent is:%d\t\r\n", lengthWritten);

	/// 查询发送FIFO里面的数据是否为0，若为0则表示当前发送数据已经发送完成
	do
	{
		TU5011_GetCountOfTxData(handle, SendChannel, &Tx_data_cnt);
		loop ++;
		
		printf("The actual number of data is sent is:%d\t\r\n", LENGTH - Tx_data_cnt);
	}while(Tx_data_cnt > 0 && loop < 5000);	/// loop为超时限制
	
	printf("Send Over!\r\n");
	Sleep(500);
	
	result = TU5011_GetCountOfRxData(handle, ReceiveChannel, &RVCount);
	printf("The actual number of data to be read is:%d\t\r\n", RVCount);
		if(RVCount > 0)
		{
			result = TU5011_ReadAsyncData(handle, ReceiveChannel, ReadpBuffer, RVCount, &lengthRead);

			Rd_data_cnt += lengthRead;

			for(i=0; i<lengthRead; i++)
			{
				printf("%d\t", ReadpBuffer[i]);
			}
		}


	/// 上电状态设置，若为0则关闭板卡前会保留最后一次通道配置，否则为默认状态
	//TU5011_SetPowerState(handle, 0);
	//TU5011_Reset(handle);
	//TU5011_GetPowerStateOfChannel(handle, SendChannel, &Mode);
	//TU5011_GetPowerStateOfChannel(handle, ReceiveChannel, &Mode);

	result = TU5011_CloseDevice(handle);
	printf("exit！\n");

	return result;
}

