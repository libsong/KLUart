// #ifndef OPAL_ASYNC_SERIAL_UTILS_H
// #define OPAL_ASYNC_SERIAL_UTILS_H

#include "we_types.h"


int            oldbits[MOXA_MAXCTLCHL];
int            fd_ser[MOXA_MAXCTLCHL] = {0};
struct termios termios[MOXA_MAXCTLCHL], old_termios[MOXA_MAXCTLCHL];

static char  g_szMsg[128];
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

/************************************************************************/
int SetSerialSpeed(Opal_Serial_Async_Ctrl_Icon *ld,int speed_index)
{
	int id = ld->ctlid;
	
#if defined(__QNXNTO__)

	/* Standard baud rates described in termios.h (QNX) */
	speed_t baud_rates[] = {B0,		B50,	B75,
							B110,	B134,	B150,
							B200,	B300,	B600,	
							B1200, 	B1800,	B2400,	
							B4800,	B9600,	B19200, 
							B38400, B57600, B115200};
	int baud_rates_val[] = {0,		50,		75, 
							110,	134,	150, 
							200,	300,	600, 
							1200,	1800,	2400, 
							4800,	9600,	19200, 
							38400,	57600,	115200};

	/* Higher non-standard baud rates (Bluestorm) */
	int extra_baud[] = {230400, 460800, 921600, 1843200};

#elif defined(__linux__)

	/* Standard baud rates described in bits/termios.h (Linux) */
	speed_t baud_rates[] = {B0,		B50,	B75,
							B110,	B134,	B150,
							B200,	B300,	B600,	
							B1200, 	B1800,	B2400,	
							B4800,	B9600,	B19200, 
							B38400, B57600, B115200, 
							B230400,B460800,B921600};
	int baud_rates_val[] = {0,		50,		75, 
							110,	134,	150, 
							200,	300,	600, 
							1200,	1800,	2400, 
							4800,	9600,	19200, 
							38400,	57600,	115200,
							230400,	460800,	921600};

	/* Higher non-standard baud rates (Bluestorm) */
	// int extra_baud[] = {1843200};
	int extra_baud[] = {500000};

#endif

	int baud_len = sizeof(baud_rates)/sizeof(speed_t);
	
	/* Set baud rate if standard */
	if(speed_index < baud_len){
		if (-1 == cfsetospeed (&termios[id], baud_rates[speed_index]))
		{
			printf("\tError setting baud rate (%d), errno = %d\n", baud_rates_val[speed_index], errno);
			return (-1);
		}
		if (-1 == cfsetispeed (&termios[id], baud_rates[speed_index]))
		{
			printf("\tError setting baud rate (%d), errno = %d\n", baud_rates_val[speed_index], errno);
			return (-1);
		}

		sprintf(g_szMsg,"\tBaud rate set to %d\n", baud_rates_val[speed_index]);
		ErrorPrint(g_szMsg);
		return 0;
	}

	/* If baud rate non standard */
	speed_index = speed_index - baud_len;
	int extra_baud_len = sizeof(extra_baud)/sizeof(int);
	if(speed_index >= extra_baud_len){
		printf("\tError invalid baud rate (index %d out of range)\n", speed_index);
		return -1;
	}

	int rate = extra_baud[speed_index];

#if defined(__QNXNTO__)

	if (-1 == cfsetospeed (&termios, rate))
	{
		printf("\tError setting baud rate (%d), errno = %d\n", rate, errno);
		return (-1);
	}
	if (-1 == cfsetispeed (&termios, rate))
	{
		printf("\tError setting baud rate (%d), errno = %d\n", rate, errno);
		return (-1);
	}

	printf("\tBaud rate set to %d\n", rate);
	return 0;

#elif defined(__linux__)

	/* Configure port to use custom speed instead of 38400 through custom divisor */
	printf("+++we set baud to %d\n",rate);
	struct serial_struct serinfo;
	serinfo.reserved_char[0] = 0;
	if (ioctl(fd_ser[id], TIOCGSERIAL, &serinfo) < 0)
	{
		printf("\tError setting baud rate (%d) - error getting device status / errno = %d\n", rate, errno);
		return (-1);
	}
	if(serinfo.baud_base < rate){
		printf("\tWarning: Maximum baud rate: %d (targeted: %d)\n", serinfo.baud_base, rate);
	}
	serinfo.baud_base=14745600;
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = (serinfo.baud_base + (rate / 2)) / rate;
	if (serinfo.custom_divisor < 1)
		serinfo.custom_divisor = 1;
	if (ioctl(fd_ser[id], TIOCSSERIAL, &serinfo) < 0)
	{
		printf("\tError setting baud rate (%d) - error setting device status / errno = %d\n", rate, errno);
		return (-1);
	}
	if (ioctl(fd_ser[id], TIOCGSERIAL, &serinfo) < 0)
	{
		printf("\tError setting baud rate (%d) - error getting device status / errno = %d\n", rate, errno);
		return (-1);
	}

	/* B38400 has just been redefined to custom baud rate */
	cfsetispeed(&termios[id], B38400);
	cfsetospeed(&termios[id], B38400);
	
	printf("\tNon standard baudrate set to %f ( = %d / %d)\n",
		(float)serinfo.baud_base / serinfo.custom_divisor, 
		serinfo.baud_base, serinfo.custom_divisor);

	return 0;

#endif

}
/************************************************************************/

/************************************************************************/
int InitSerial (Opal_Serial_Async_Ctrl_Icon *ld,int ttyType)
{
	char           buf [256];
	unsigned short data_size [4]	= {CS5, CS6, CS7, CS8};
	unsigned short data_size_val [4] = {CS8,CS5, CS6, CS7};//保持与模型界面顺序一致
	int id = ld->ctlid;
	
#if defined(__QNXNTO__)
	int serCfg = 0;
	sprintf(buf, "/dev/ser%d", ld->portnum);
#elif defined(__linux__)
	if (ttyType == 2)//simulink 界面上第 2 个
		sprintf(buf, "/dev/ttyM%d", ld->portnum);
	if (ttyType == 3)
		sprintf(buf, "/dev/ttyS%d", ld->portnum);
	if (ttyType == 4)
		sprintf(buf, "/dev/ttyMUE%d", ld->portnum);
#endif

	sprintf(g_szMsg,"%s: Opening serial device %s\n", PROGNAME, buf);
	ErrorPrint(g_szMsg);
#if defined(__QNXNTO__)
	if ((fd_ser = open (buf, O_RDWR)) == -1)
#elif defined(__linux__)
	if ((fd_ser[id] = open (buf, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1)
#endif
	{
		sprintf(g_szMsg,"%s: Error opening serial device %s, errno %d\n", PROGNAME, buf, errno);
		ErrorPrint(g_szMsg);
		return (-1);
	}

	/* Get terminal attributes */
	if (tcgetattr (fd_ser[id], &termios[id]))
	{
		sprintf(g_szMsg,"%s: Error tcgetattr, errno %d\n", PROGNAME, errno);
		ErrorPrint(g_szMsg);
		close (fd_ser[id]); // release the file descriptor
		return (-1);
	}
	/* Save old terminal values */
	memcpy(&old_termios[id], &termios[id], sizeof(struct termios));

#if defined(__linux__)
	memset(&termios[id], 0, sizeof(termios));
#endif

	/* Set RAW mode */
	cfmakeraw(&termios[id]);

	/* Set terminal input and output baud rates */
	printf("%s: Setting baud rate...\n", PROGNAME);
	if (-1 == SetSerialSpeed(ld,ld->baud))
		return (-1);

	/* Enable the receiver and set local mode */
	termios[id].c_cflag |= (CLOCAL | CREAD);

	/* Set terminal data length */
	termios[id].c_cflag &= ~(CSIZE);
	// termios[id].c_cflag |=  (data_size[ld->size]);
	termios[id].c_cflag |=  (data_size_val[ld->size]);
	sprintf(g_szMsg,"%s: Data size set to %d\n", PROGNAME, data_size_val[ld->size]);
	ErrorPrint(g_szMsg);

	/* Set terminal parity */
	termios[id].c_cflag &= ~(PARENB | PARODD);
	switch (ld->parity)
	{
	case 1: // None
		sprintf(g_szMsg,"%s: Parity bit set to None\n", PROGNAME);
		ErrorPrint(g_szMsg);
		break;
	case 2: // Even
		termios[id].c_cflag |= (PARENB);
		sprintf(g_szMsg,"%s: Parity bit set to Even\n", PROGNAME);
		ErrorPrint(g_szMsg);
		break;
	case 3: // Odd (ie: PARENB | PARODD)
		termios[id].c_cflag |= (PARODD | PARENB);
		sprintf(g_szMsg,"%s: Parity bit set to Odd\n", PROGNAME);
		ErrorPrint(g_szMsg);
		break;
	}

	/* Set terminal stop bit */
	termios[id].c_cflag &= ~(CSTOPB);
	if (ld->stop)
	{
		termios[id].c_cflag |= (CSTOPB);
		sprintf(g_szMsg,"%s: Stop bit set to 2\n", PROGNAME);
		ErrorPrint(g_szMsg);
	} else {
		sprintf(g_szMsg,"%s: Stop bit set to 1\n", PROGNAME);
		ErrorPrint(g_szMsg);
	}

	/* Clear terminal output flow control */
#if defined(__QNXNTO__)
	termios.c_iflag &= ~(IXON | IXOFF);          // set -isflow  & -osflow
	termios.c_cflag &= ~(IHFLOW | OHFLOW);       // set -ihflow  & -ohflow
	tcflow (fd_ser, TCION);                      // set -ispaged
	tcflow (fd_ser, TCOON);                      // set -ospaged
	tcflow (fd_ser, TCIONHW);                    // set -ihpaged
	tcflow (fd_ser, TCOONHW);                    // set -ohpaged
#endif

	/* Set terminal attributes */
	switch (ld->flow)
	{
	case 1:
		sprintf(g_szMsg,"%s: Flow control set to None\n", PROGNAME);
		ErrorPrint(g_szMsg);
		break;

	case HW_FLOW_CONTROL:
#if defined(__QNXNTO__)
		termios.c_lflag |= (IEXTEN);
		termios.c_cflag |= (IHFLOW | OHFLOW);	// set +ihflow & +ohflow
		tcflow (fd_ser, TCIOFFHW);				// set +ihpaged
		tcflow (fd_ser, TCOOFFHW);				// set +ohpaged
		serCfg = _CTL_RTS_CHG | _CTL_RTS;		// set RTS and DTR
		serCfg |= _CTL_DTR_CHG | _CTL_DTR;
#endif
		printf("%s: Flow control set to Hardware (RTS/CTS)\n", PROGNAME);
		break;

	case SW_FLOW_CONTROL:
		printf("%s: Software flow control not supported! Exiting\n", PROGNAME);
		close (fd_ser[id]); // release the file descriptor
		return (-1);
		break;
	}

	/* Set Buffer Size */
	// This mode is used for non blocking reading. The read call returned what is the input queue
	termios[id].c_cc[VMIN] 	= 0;
	termios[id].c_cc[VTIME] = 0;
	
	/*
	// This mode is used when the input buffer size is constant
	termios.c_cc[VMIN] 	= input_buffer_size;
	termios.c_cc[VTIME] = 0;

	// This mode is used for periodic reading. Ts  should be in tenths of second
	termios.c_cc[VMIN] 	= 0;
	termios.c_cc[VTIME] = Ts;

	// This mode is used for retrieve a frame with constant size before Ts expires.
	termios.c_cc[VMIN] 	= input_buffer_size;
	termios.c_cc[VTIME] = Ts;
	*/	


#if defined(__QNXNTO__)
	if (-1 == tcsetattr (fd_ser, TCSADRAIN, &termios))
		printf("%s: Error with device settings, errno %d\n", PROGNAME, errno);
#elif defined(__linux__)
	if (-1 == tcsetattr (fd_ser[id], TCSANOW, &termios[id]))
		printf("%s: Error with device settings, errno %d\n", PROGNAME, errno);
#endif

	/* Clear both input and ouput fd_ser's buffers */
	tcflush (fd_ser[id], TCIOFLUSH);

	return (EOK);
}
/************************************************************************/

int WriteSerial(Opal_Serial_Async_Ctrl_Icon *ld,unsigned char *buf, int len)
{
	int id = ld->ctlid;
	
	return write(fd_ser[id],buf,len);
}

int ReadSerialWithTimeout(Opal_Serial_Async_Ctrl_Icon *ld,unsigned char *buf,int RequestLen,int Sec,int uSec)
{
	struct timeval tv;
    fd_set rfd;
	int n;
	int id = ld->ctlid;
	
	// printf("read id = %d\n",id);
	
	FD_ZERO( &rfd );
	FD_SET( fd_ser[id], &rfd );
	tv.tv_sec = Sec;
	tv.tv_usec = uSec;
	switch ( n = select( 1 + fd_ser[id],
		   &rfd, 0, 0, &tv ) ) {
	  case -1:
		// perror( "select" );
		return -1;
	  case  0:
		// printf( "select timed out .\n");
		return 0;
	  default:
		// printf( "descriptors(%d) ready ...\n",fd_ser[id]);
		if( FD_ISSET( fd_ser[id], &rfd ) ) {
		  n = read(fd_ser[id],buf,RequestLen);
		  // printf( "%d data read ...\n", n );
		  return n;
		}
	}
}


/************************************************************************/
int CloseSerial (Opal_Serial_Async_Ctrl_Icon *ld)
{
	
	char	buf [256];
	int		serCfg = 0;
	int id = ld->ctlid;

#if defined(__QNXNTO__)
	sprintf(buf, "/dev/ser%d", ld->portnum);
#elif defined(__linux__)
	sprintf(buf, "/dev/ttyM%d", ld->portnum);
#endif

	// printf("%s: Closing serial device %s\n\n", PROGNAME, buf);

	/* Restore terminal attributes */
	memcpy (&termios[id], &old_termios[id], sizeof (struct termios));


	/* Then restore old values, ie locking flags */
#if defined(__QNXNTO__)

	tcsetattr (fd_ser[id], TCSADRAIN, &old_termios[id]);
	serCfg = _CTL_RTS_CHG | 0;			// clear RTS and DTR
	serCfg |= _CTL_DTR_CHG | 0;

	if ( devctl (fd_ser[id], DCMD_CHR_SERCTL, &serCfg, sizeof(serCfg), NULL) == -1 ){
		// printf("%s: Error devctl, errno %d\n", PROGNAME, errno);
	}

#elif defined(__linux__)

	tcsetattr (fd_ser[id], TCSANOW, &old_termios[id]);

#endif  

	close (fd_ser[id]);

	return(0);
}
/************************************************************************/

// #endif // OPAL_SERIAL_H
