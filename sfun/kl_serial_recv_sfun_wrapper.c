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
  *   Created: Mon Apr 23 17:04:00 2018
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

/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "TU5011.h"
#include "we_types.h"

static char  g_szMsg[1024];
static void ErrorPrint(char *str)
{
	static FILE *fp = 0;
	if( fp == 0)
	{
		fp = fopen("/tmp/KL_Uart_recvlog.txt","w");
	}
	
	if( fp != 0)
	{
		fprintf(fp, "%s",str);
		fflush(fp);
	}
}

#define PS3510SERIAL_MAXDEV 256
#define LENGTH 1024

extern PS_DevHandle g_ps3510_handle[PS3510SERIAL_MAXDEV][PS3510SERIAL_MAXDEV];
extern Opal_Serial_Async_Ctrl_Icon g_icon[MOXA_MAXCTLCHL];

void kl_serial_recv_sfun_Start_wrapper(const real_T *u0,
                          const real_T *u1,
                          real_T *y0,
                          real_T *y1  , 
                          const real_T  *ctlid, const int_T  p_width0, 
                          const real_T  *recvchl, const int_T  p_width1, 
                          const real_T  *resv0, const int_T  p_width2, 
                          const real_T  *resv1, const int_T  p_width3, 
                          const real_T  *resv2, const int_T  p_width4, 
                          const real_T  *resv3,  const int_T p_width5,
			     const int_T y_width, const int_T u_width)
{
}

void kl_serial_recv_sfun_Outputs_wrapper(const real_T *u0,
                          const real_T *u1,
                          real_T *y0,
                          real_T *y1  , 
                          const real_T  *ctlid, const int_T  p_width0, 
                          const real_T  *recvchl, const int_T  p_width1, 
                          const real_T  *resv0, const int_T  p_width2, 
                          const real_T  *resv1, const int_T  p_width3, 
                          const real_T  *resv2, const int_T  p_width4, 
                          const real_T  *resv3,  const int_T p_width5,
			     const int_T y_width, const int_T u_width)
{
	static int cnt = 0;
	
	int result;
	int devType = (int)*ctlid;
	int ReceiveChannel = (int)*recvchl - 1;
	int en = (int)*u1;
	int RVCount = u_width;
	int lengthRead = 0;
	int i;
	unsigned char ReadpBuffer[LENGTH];
	int pxim = (int)*resv0;
	int pxin = (int)*resv1;
	
	if (RVCount > 1024) {
		RVCount = 1024;
	}
	
	if (en == 1) {
		if (devType == 1) {
			result = TU5011_GetCountOfRxData(g_ps3510_handle[pxim][pxin], ReceiveChannel, &RVCount);
			// if (cnt++ % 1000 == 0) {
				// sprintf(g_szMsg,"ps3510 recv in loop %d, Channel:%d ,u_width:%d ,RVCount:%d,result:%d",cnt,ReceiveChannel,u_width,\
																			// RVCount,result);
				// ErrorPrint(g_szMsg);
			// }
			
			//The actual number of data to be read is RVCount
			if(RVCount > 0)
			{
				result = TU5011_ReadAsyncData(g_ps3510_handle[pxim][pxin], ReceiveChannel, ReadpBuffer, RVCount, &lengthRead);
			
				// sprintf(g_szMsg,"ps3510 recv in loop TU5011_ReadAsyncData [pxim][pxin]=%d:%d , result:%d",pxim,pxin,result);
				// ErrorPrint(g_szMsg);
			
				// Rd_data_cnt += lengthRead;
				// sprintf(g_szMsg,"cnt : %d -- ",cnt++);
				// ErrorPrint(g_szMsg);
				for(i=0; i<RVCount; i++)
				{
					// printf("%d\t", ReadpBuffer[i]);
					// sprintf(g_szMsg,"%d ",ReadpBuffer[i]);
					// ErrorPrint(g_szMsg);
					y0[i] = ReadpBuffer[i];
				}
				// sprintf(g_szMsg,"\n");
				// ErrorPrint(g_szMsg);
				*y1 = result;
			}
		}
		
		if (devType == 2 || devType == 3 || devType == 4) {
			lengthRead = ReadSerialWithTimeout(&g_icon[ReceiveChannel],ReadpBuffer,RVCount,0,50);
			if(lengthRead > 0) {
				for(i=0; i<lengthRead; i++)
				{
					//printf("%d\t", ReadpBuffer[i]);
					
					y0[i] = ReadpBuffer[i];
				}
				*y1 = lengthRead;
			}
		}
	}
}

void kl_serial_recv_sfun_Terminate_wrapper(const real_T *u0,
                          const real_T *u1,
                          real_T *y0,
                          real_T *y1  , 
                          const real_T  *ctlid, const int_T  p_width0, 
                          const real_T  *recvchl, const int_T  p_width1, 
                          const real_T  *resv0, const int_T  p_width2, 
                          const real_T  *resv1, const int_T  p_width3, 
                          const real_T  *resv2, const int_T  p_width4, 
                          const real_T  *resv3,  const int_T p_width5,
			     const int_T y_width, const int_T u_width)
{

}

