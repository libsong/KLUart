
#include <stdlib.h>
#include <stdio.h>
#include "TU5011.h"

static PS_DevHandle handle;
int main()
{
	int result = PS_SUCCESS;
	result = TU5011_OpenDeviceByName("PXI13::13::INSTR", &handle); /// 根据资源名称打开板卡
	if(result != 0)
	{
		printf("Open device is error，The error code is %d.\n", result);
		return -1;
	}
	else
	{
		printf("Open device Success!\n");
	}
	Sleep(2000);
	result = TU5011_CloseDevice(handle);
	if(result != 0)
	{
		printf("Close device is error，The error code is %d.\n", result);
		return -1;
	}
	else
	{
		printf("Close device Success!\n");
	}

	return result;
}





