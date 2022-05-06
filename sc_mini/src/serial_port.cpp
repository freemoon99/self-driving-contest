#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h> 
    
int uart_fd = -1;

int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop) 
	{ 
		struct termios newtio,oldtio; 
		/*±£Žæ²âÊÔÏÖÓÐŽ®¿Ú²ÎÊýÉèÖÃ£¬ÔÚÕâÀïÈç¹ûŽ®¿ÚºÅµÈ³öŽí£¬»áÓÐÏà¹ØµÄ³öŽíÐÅÏ¢*/ 
		if  ( tcgetattr( fd,&oldtio)  !=  0) {  
			perror("SetupSerial 1");
			//LOGI("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
			return -1; 
		} 
		bzero( &newtio, sizeof( newtio ) ); 
		/*²œÖèÒ»£¬ÉèÖÃ×Ö·ûŽóÐ¡*/ 
		newtio.c_cflag  |=  CLOCAL | CREAD;  
		newtio.c_cflag &= ~CSIZE;  
		/*ÉèÖÃÍ£Ö¹Î»*/ 
		switch( nBits ) 
		{ 
		case 7: 
			newtio.c_cflag |= CS7; 
			break; 
		case 8: 
			newtio.c_cflag |= CS8; 
			break; 
		} 
		/*ÉèÖÃÆæÅŒÐ£ÑéÎ»*/ 
		switch( nEvent ) 
		{ 
		case 'o':
		case 'O': //ÆæÊý 
			newtio.c_cflag |= PARENB; 
			newtio.c_cflag |= PARODD; 
			newtio.c_iflag |= (INPCK | ISTRIP); 
			break; 
		case 'e':
		case 'E': //ÅŒÊý 
			newtio.c_iflag |= (INPCK | ISTRIP); 
			newtio.c_cflag |= PARENB; 
			newtio.c_cflag &= ~PARODD; 
			break;
		case 'n':
		case 'N':  //ÎÞÆæÅŒÐ£ÑéÎ» 
			newtio.c_cflag &= ~PARENB; 
			break;
		default:
			break;
		} 
		/*ÉèÖÃ²šÌØÂÊ*/ 
		switch( nSpeed ) 
		{ 
		case 2400: 
			cfsetispeed(&newtio, B2400); 
			cfsetospeed(&newtio, B2400); 
			break; 
		case 4800: 
			cfsetispeed(&newtio, B4800); 
			cfsetospeed(&newtio, B4800); 
			break; 
		case 9600: 
			cfsetispeed(&newtio, B9600); 
			cfsetospeed(&newtio, B9600); 
			break; 
		case 115200: 
			cfsetispeed(&newtio, B115200); 
			cfsetospeed(&newtio, B115200); 
			break; 
		case 460800: 
			cfsetispeed(&newtio, B460800); 
			cfsetospeed(&newtio, B460800); 
			break; 
		default: 
			cfsetispeed(&newtio, B9600); 
			cfsetospeed(&newtio, B9600); 
			break; 
		} 
		/*ÉèÖÃÍ£Ö¹Î»*/ 
		if( nStop == 1 ) 
			newtio.c_cflag &=  ~CSTOPB; 
		else if ( nStop == 2 ) 
			newtio.c_cflag |=  CSTOPB; 
		/*ÉèÖÃµÈŽýÊ±ŒäºÍ×îÐ¡œÓÊÕ×Ö·û*/ 
		newtio.c_cc[VTIME]  = 0; 
		newtio.c_cc[VMIN] = 0; 
		/*ŽŠÀíÎŽœÓÊÕ×Ö·û*/ 
		tcflush(fd,TCIFLUSH); 
		/*Œ€»îÐÂÅäÖÃ*/ 
		if((tcsetattr(fd,TCSANOW,&newtio))!=0) 
		{ 
			perror("com set error"); 
			return -1; 
		} 
		//LOGI("set done!\n"); 
		return 0; 
	}

int open_port(const char *port, int baud_rate)
{
	if(access(port, F_OK) != 0) //  /dev/ttyS1
	{
		printf("Can't access port: %s, exit node lidar.\r\n", port);
		return -1;
	}
	uart_fd = open(port,O_RDWR | O_NOCTTY | O_NONBLOCK);//O_NONBLOCKÉèÖÃÎª·Ç×èÈûÄ£Êœ
	if(uart_fd > 0)
	{
		if(0 == set_opt(uart_fd, baud_rate, 8, 'N', 1))
		{
			tcflush(uart_fd, TCIOFLUSH);
			//LOGI("open_port ok.\r\n");
		}
		else
		{
			//LOGI("set baudrate fail, open_port fail.\r\n");
			return -1;
		}
	}
	else
	{
		perror("Can't Open SerialPort");
	}
	return uart_fd;
}


int read_port(unsigned char *pbuf, unsigned int len)
{
	int rlen = 0;
	if(uart_fd > 0)
	{
		rlen = read(uart_fd, pbuf, len);
	}
	else
	{
		perror("Can't Open SerialPort");
	}
	return rlen;
}

int write_port(unsigned char *pbuf, unsigned int len)
{
	int wlen = 0;
	if(uart_fd > 0)
	{
		wlen = write(uart_fd, pbuf, len);
	}
	else
	{
		perror("Can't Open SerialPort");
	}
	return wlen;
}

int close_port()
{
	if(uart_fd > 0)
	{
		close(uart_fd);
		return 0;
	}
	return -1;
}


/*
int main(int argc, char **argv)
{
	return 0;
}
*/

