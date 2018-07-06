/*
  *
  *   --- THIS FILE GENERATED BY S-FUNCTION BUILDER: 3.0 ---
  *
  *   This file is a wrapper S-function produced by the S-Function
  *   Builder which only recognizes certain fields.  Changes made
  *   outside these fields will be lost the next time the block is
  *   used to load, edit, and resave this file. This file will be overwritten
  *   by the S-function Builder block. If you want to edit this file by hand, 
  *   you must change it only in the area defined as:  
  *
  *        %%%-SFUNWIZ_wrapper_XXXXX_Changes_BEGIN 
  *            Your Changes go here
  *        %%%-SFUNWIZ_wrapper_XXXXXX_Changes_END
  *
  *   For better compatibility with the Simulink Coder, the
  *   "wrapper" S-function technique is used.  This is discussed
  *   in the Simulink Coder User's Manual in the Chapter titled,
  *   "Wrapper S-functions".
  *
  *   Created: Mon Apr 23 15:13:32 2018
  */


/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

//!!!!!!!!!!!!!!!!!!!!!!!!!!!
//本文件被用来发送模块了，与文件名称不一致

/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "TU5011.h"

static char  g_szMsg[1024];
static void ErrorPrint(char *str)
{
	static FILE *fp = 0;
	if( fp == 0)
	{
		fp = fopen("/tmp/KL_Uart_sendlog.txt","w");
	}
	
	if( fp != 0)
	{
		fprintf(fp, "%s\n",str);
		fflush(fp);
	}
}

#define PS3510SERIAL_MAXDEV 256
#define LENGTH 1024
extern PS_DevHandle g_ps3510_handle[PS3510SERIAL_MAXDEV][PS3510SERIAL_MAXDEV];
extern int g_DevInited[PS3510SERIAL_MAXDEV][PS3510SERIAL_MAXDEV];

void kl_serial_ctl_sfun_Start_wrapper(const real_T *u0,
                          const real_T *u1,
                          real_T *y0,
                          real_T *y1  , 
                          const real_T  *ctlid, const int_T  p_width0, 
                          const real_T  *resv0, const int_T  p_width1, 
                          const real_T  *resv1, const int_T  p_width2, 
                          const real_T  *resv2, const int_T  p_width3, 
                          const real_T  *resv3,  const int_T p_width4,
			     const int_T y_width, const int_T u_width)
{
}

void kl_serial_ctl_sfun_Outputs_wrapper(const real_T *u0,
                          const real_T *u1,
                          real_T *y0,
                          real_T *y1  , 
                          const real_T  *ctlid, const int_T  p_width0, 
                          const real_T  *resv0, const int_T  p_width1, 
                          const real_T  *resv1, const int_T  p_width2, 
                          const real_T  *resv2, const int_T  p_width3, 
                          const real_T  *resv3,  const int_T p_width4,
			     const int_T y_width, const int_T u_width)
{
	static int cnt = 0;
	
	uInt32 lengthWritten = 0;
	int devType = (int)*ctlid;
	int i;
	int loop = 0;
	int SendChannel = (int)*resv0 - 1;
	int en = (int)*u1;
	int Tx_data_cnt = 0;
	int rc = 0;
	unsigned char pBuffer[LENGTH];
	int pxim = (int)*resv1;
	int pxin = (int)*resv2;
	
	if (en == 1) {
		if (devType == 1) {//TU-5011
		
			for(i=0; i<u_width; i++)
			{
				pBuffer[i] = (unsigned char)u0[i];
			}
			//数据载入FIFO
			rc = TU5011_LoadAsyncData(g_ps3510_handle[pxim][pxin], SendChannel, pBuffer, u_width, &lengthWritten);

			/// 查询发送FIFO里面的数据是否为0，若为0则表示当前发送数据已经发送完成
			do
			{
				TU5011_GetCountOfTxData(g_ps3510_handle[pxim][pxin], SendChannel, &Tx_data_cnt);
				loop ++;
			}while(Tx_data_cnt > 0 && loop < 2);	/// loop为超时限制
			
			for(i=0; i<u_width; i++)
			{
				y0[i] = rc;
			}
			*y1 = lengthWritten;
			
			// if (cnt++ % 1000 == 0) {
				// sprintf(g_szMsg,"ps3510 send in loop %d, SendChannel:%d ,u_width:%d - wrote:%d, rc:%d,Tx_data_cnt:%d",cnt,SendChannel,u_width,\
																			// lengthWritten,rc,Tx_data_cnt);
				// ErrorPrint(g_szMsg);
			// }
		}
	}
}

void kl_serial_ctl_sfun_Terminate_wrapper(const real_T *u0,
                          const real_T *u1,
                          real_T *y0,
                          real_T *y1  , 
                          const real_T  *ctlid, const int_T  p_width0, 
                          const real_T  *resv0, const int_T  p_width1, 
                          const real_T  *resv1, const int_T  p_width2, 
                          const real_T  *resv2, const int_T  p_width3, 
                          const real_T  *resv3,  const int_T p_width4,
			     const int_T y_width, const int_T u_width)
{

}













