#pragma once

#ifndef PROGNAME
#define PROGNAME "AsyncSerialUtils"
#endif

#if !defined (EOK)
#define EOK	0
#endif

#define NO_FLOW_CONTROL 1
#define HW_FLOW_CONTROL 2
#define SW_FLOW_CONTROL 3	// Not supported yet.

#if defined(__linux__)
#include <stdio.h>   /*标准输入输出的定义*/
#include <errno.h>  /*错误号定义*/
#include <sys/stat.h>
#include <fcntl.h>  /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <sys/types.h>
#include <unistd.h> /*UNIX 标准函数定义*/
#include <asm/ioctls.h>
#include <linux/serial.h>
#include <string.h>
#endif

#if defined(__QNXNTO__)
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <devctl.h>
#include <sys/dcmd_chr.h>
#endif

#include <errno.h>

#define MOXA_MAXCTLCHL 128

enum we_baud{
	WE_BAUD9600 = 13,
	WE_BAUD19200,
	WE_BAUD38400,
	WE_BAUD57600,
	WE_BAUD115200,
	WE_BAUD500000=21
};
enum we_datasize{
	WE_DATASIZE5,
	WE_DATASIZE6,
	WE_DATASIZE7,
	WE_DATASIZE8
};
enum we_parity{
	WE_PARITY_None = 1,
	WE_PARITY_Even,
	WE_PARITY_Odd
};
enum we_stopbit{
	WE_STOPBIT1,
	WE_STOPBIT2
};
enum we_flowctl{
	WE_FLOWCTL_None = 1,
	WE_FLOWCTL_HW,
	WE_FLOWCTL_SW	//Not supported yet.
};


typedef struct serinfo{
	int ctlid;
	int portnum; 
	int baud; //13 -> 9600 ,17 -> 115200
	int size; //0 1 2 3 -> 5 6 7 8
	int parity;//1 2 3 -> None Even Odd
	int stop;//0 1 -> 1 2
	int flow;	// 1 2 3 -> None Hardware(RTS CTS) Software(XON XOFF)
}Opal_Serial_Async_Ctrl_Icon;
