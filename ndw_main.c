/*
 
Main function of NDW.
 
*/

#ifndef CCN_NDW_DEFINED 
#define CCN_NDW_DEFINED


#define T_NAME_LEN			12
#define T_BUF_LEN			15
#define SEND_COUNT			500
#define SEND_INTERVAL_US	500000


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <netinet/ether.h>
#include <netpacket/packet.h>
#include <string.h>


int arg_count;
int recv_count;
char padding_mark[18];
char databuf[512];
FILE *fp;

char name_req[T_NAME_LEN];
char name_req_list[15][T_NAME_LEN];


struct tinyosndw_data
{

	unsigned char name[T_NAME_LEN];
	unsigned char type;		/**数据类型 */
	unsigned char buf[T_BUF_LEN];	/**数据包内容 */
};


/*数据类型，可扩展*/
enum ndw_data_type
{
	NDW_DATA_TYPE_REQ = 1,	//请求数据包
	NDW_DATA_TYPE_RSP = 2,	//回应数据包
	NDW_DATA_TYPE_ACK = 4,	//for test
	NDW_NEIB_TYPE_REQ = 4,	//
	NDW_NEIB_TYPE_RSP = 8,	//
	NDW_PATH_TYPE_REQ = 16,	//
	NDW_PATH_TYPE_RSP = 32,	//
};



/**
 * Set the USB Port 
 */
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if  ( tcgetattr( fd,&oldtio)  !=  0) { 
		perror("SetupSerial 1");
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag  |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	switch( nBits )
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}

	switch( nEvent )
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E': 
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':  
		newtio.c_cflag &= ~PARENB;
		break;
	}

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
	if( nStop == 1 )
		newtio.c_cflag &=  ~CSTOPB;
	else if ( nStop == 2 )
	newtio.c_cflag |=  CSTOPB;
	newtio.c_cc[VTIME]  = 0;//重要
	newtio.c_cc[VMIN] = 100;//返回的最小值  重要
	tcflush(fd,TCIFLUSH);
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
		return -1;
	}
//	//printf("set done!\n\r");
	return 0;
}

/**
 * Cal the CRC Byte
 */
uint16_t crcByte(uint16_t crc, uint8_t b) {
  crc = (uint8_t)(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t)(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

/**
 * Cal the CRC Val
 */
uint16_t crccal(unsigned char* buf, int len)
{
	uint16_t res = 0;
	int i;

	for ( i = 0; i < len; ++i)
	{
		res = crcByte(res, buf[i]);
	}

	return res;
}

/**
 * Print the data in HEX, for debug
 */
void printhex_macaddr(void *hex, int len, char *tag)
{
	int i;
	unsigned char *p = (unsigned char *)hex;

	if(len < 1)
		return;

	for(i = 0; i < len - 1; i++)
	{
		if(*p < 0x10)
			printf("0%x%s", *p++, tag);
		else
			printf("%2x%s", *p++, tag);
	}

	if(*p < 0x10)
		printf("0%x", *p++);
	else
		printf("%2x", *p++);
}


void
tinyos_recv(int fd1)
{
	int nread;
	 char usbbuf[1024];
	 char *start_p;
	 char *usbbuf_p;
	 char start_mark[] = {0x7e,0x45,0x00};

	struct tinyosndw_data *tinyosndwrecv_data;
	tinyosndwrecv_data = (struct tinyosndw_data*) malloc(sizeof(*tinyosndwrecv_data));

	memset(usbbuf,0,1024);
	nread = read(fd1, usbbuf, 1024);//读USB串口
	//printhex_macaddr(usbbuf, nread, " ");
	//printf("\n");
	usbbuf_p = usbbuf;

	//fprintf(fp, "nodecount loop %d!\n", nodecount);
	//fflush(fp);

	//printf("usbbuf_p %d\n", usbbuf_p);


	while(*usbbuf_p == 0)
		usbbuf_p++;

	//printf("usbbuf_p++ %d\n", usbbuf_p);

	start_p = strstr(usbbuf_p, start_mark);

	//printf("usbbuf_p %d\n start_p %d \n", usbbuf_p, start_p);

	while(start_p)
	{
		//fprintf(fp, "got a pkt! len = %x\n", *(start_p+7));
		//fflush(fp);

		if ((*(start_p+7) == 0x1c)&&(*(start_p+6) == 0x00))	//a data packet coming, resolve it!
		{
			//fprintf(fp, "got a pkt!\n");
			//fflush(fp);
			//printf("Resolve a 26 bytes pkt.\n");
			//printhex_macaddr(start_p, 39, " ");
			//printf("\n");

			memcpy(tinyosndwrecv_data, start_p + 10, sizeof(*tinyosndwrecv_data));

			if((tinyosndwrecv_data->type == NDW_DATA_TYPE_RSP))	//a DATA packet RSP
			{
				printf("\n%s : %s\n\n", name_req, tinyosndwrecv_data->buf);
				recv_count++;
				//return;
			}

			usbbuf_p = usbbuf_p + (start_p - usbbuf_p) +38;	//move the pointer to check next packet
		}

		if (*(start_p+7) == 0x0e)	// beacon packet
		{
			usbbuf_p = usbbuf_p + (start_p - usbbuf_p) + 25;	//move the pointer to check next packet
		}

		usbbuf_p++;

		if(usbbuf_p - 1 == start_p)	//in case a bug
			break;

		//if(usbbuf_p - usbbuf > 256)	//in case a bug
			//break;

		//fprintf(fp, "while(start_p) %d!\n", usbbuf_p - usbbuf);
		//fflush(fp);

		start_p = strstr((char*)usbbuf_p, (char*)start_mark);	//search next start_mark

	}
}


/**
 * Run the main loop of the ndw 
 */
void
ndwd_run(char* name_req)
{

	int fd1,nset1,nread,count;
	uint16_t crcres;

	
	unsigned char test[] = {
		0x7e,0x44,0x26,0x00,0x00,0x00,0x00,0x00,0x1c,0x22,0x03,
		0x01,0x00,0x01,0x00,0x74,0x65,0x73,0x74,0x64,0x00,0x2d,0x01,0x01,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0xff,0xff,0x7e};

	recv_count = 0;

	fd1 = open("/dev/ttyUSB0", O_RDWR|O_NONBLOCK);//打开串口
	if (fd1 == -1)
		exit(3);

	nset1 = set_opt(fd1,115200, 8, 'N', 1);//设置串口属性
	if (nset1 == -1)
		exit(1);

	struct tinyosndw_data *tinyosndwsend_data;
	tinyosndwsend_data = (struct tinyosndw_data*) malloc(sizeof(*tinyosndwsend_data));

	memset(tinyosndwsend_data,0,sizeof(*tinyosndwsend_data));
	memset(tinyosndwsend_data->buf,0xff, 4);

	if(strcmp(name_req, "test/turn") == 0)
	{//turn
		for(count = 1; count <SEND_COUNT; count++)
		{
			strcpy((char*)tinyosndwsend_data->name, (char*)name_req_list[count%15]);	//fill the name
			printf("%s\n", name_req_list[count%15]);

			tinyosndwsend_data->type = NDW_DATA_TYPE_REQ;	//default
			memcpy(test+11,tinyosndwsend_data,sizeof(*tinyosndwsend_data));	//fill the buf ,send to usb
			crcres = crccal(test+1, sizeof(test)-4);	//calculate the crc value of "test"

			test[sizeof(test)-3] = crcres;	//add the crc value to "test" packet
			test[sizeof(test)-2] = crcres>>8; 

			//printhex_macaddr(test, sizeof(test), " ");	//for debug
			//printf("\n");
			
			nread = write(fd1, test, sizeof(test));	// no result in 300ms, send the REQ again.
			usleep(SEND_INTERVAL_US);	//wait for the data  us
			tinyos_recv(fd1);
			printf("Send count: %d\n", count);

			fprintf(fp, "Send count: %d Recv count %d\n", count, recv_count);
			fflush(fp);
		}
	}
	else if(strcmp(name_req, "test/rand") == 0)
	{//random
		//initial the tinyosndwsend_data
		for(count = 1; count <SEND_COUNT; count++)
		{
			strcpy((char*)tinyosndwsend_data->name, (unsigned char*)name_req_list[rand()%15]);	//fill the name
			printf("%s\n", name_req_list[rand()%15]);
			//fflush(stdout);

			tinyosndwsend_data->type = NDW_DATA_TYPE_REQ;	//default
			memcpy(test+11,tinyosndwsend_data,sizeof(*tinyosndwsend_data));	//fill the buf ,send to usb
			crcres = crccal(test+1, sizeof(test)-4);	//calculate the crc value of "test"

			test[sizeof(test)-3] = crcres;	//add the crc value to "test" packet
			test[sizeof(test)-2] = crcres>>8; 

			//printhex_macaddr(test, sizeof(test), " ");	//for debug
			//printf("\n");

			nread = write(fd1, test, sizeof(test));	// no result in 300ms, send the REQ again.
			usleep(SEND_INTERVAL_US);	//wait for the data  us
			tinyos_recv(fd1);
			printf("Send count: %d\n", count);

			fprintf(fp, "Send count: %d Recv count %d\n", count, recv_count);
			fflush(fp);
		}
	}
	else
	{//repeat
		//initial the tinyosndwsend_data
		strcpy((char*)tinyosndwsend_data->name, (char*)name_req);	//fill the name

		tinyosndwsend_data->type = NDW_DATA_TYPE_REQ;	//default
		memcpy(test+11,tinyosndwsend_data,sizeof(*tinyosndwsend_data));	//fill the buf ,send to usb
		crcres = crccal(test+1, sizeof(test)-4);	//calculate the crc value of "test"

		test[sizeof(test)-3] = crcres;	//add the crc value to "test" packet
		test[sizeof(test)-2] = crcres>>8; 

		printhex_macaddr(test, sizeof(test), " ");	//for debug
		printf("\n");

		nread = write(fd1, test, sizeof(test));	//send the first req

		for(count = 1; count <SEND_COUNT; count++)
		{
			usleep(SEND_INTERVAL_US);	//wait for the data  us
			nread = write(fd1, test, sizeof(test));	// no result in 300ms, send the REQ again.
			tinyos_recv(fd1);
			printf("Send count: %d\n", count);

			fprintf(fp, "Send count: %d\tRecv count %d\n", count, recv_count);
			fflush(fp);
		}
	}



}


/**
 * Main function of ndw
 */
int
main(int argc, char **argv)
{

	arg_count = argc;
	memset(databuf, 0 , 512);

	strcpy(name_req_list[0], "t3/6/32/h");
	strcpy(name_req_list[1], "t3/6/32/t");
	strcpy(name_req_list[2], "t3/6/32/l");

	strcpy(name_req_list[3], "t3/6/33/h");
	strcpy(name_req_list[4], "t3/6/33/t");
	strcpy(name_req_list[5], "t3/6/33/l");

	strcpy(name_req_list[6], "t3/9/11/h");
	strcpy(name_req_list[7], "t3/9/11/t");
	strcpy(name_req_list[8], "t3/9/11/l");

	strcpy(name_req_list[9], "d10/3/56/h");
	strcpy(name_req_list[10], "d10/3/56/t");
	strcpy(name_req_list[11], "d10/3/56/l");

	strcpy(name_req_list[12], "d10/5/42/h");
	strcpy(name_req_list[13], "d10/5/42/t");
	strcpy(name_req_list[14], "d10/5/42/l");

	if(argc < 2)
	{
		printf("Please input a name as an parameter.\n");
		exit(0);
	}

	fp = fopen("log.txt","w");
	if (fp== NULL) {
		perror("can not open file");
		
	}

	fprintf(fp, "\n\n%s\nndw_main started!\n", argv[1]);
	fflush(fp);

	strcpy((char*)name_req, (char*)argv[1]);
	//printf("%s\n", name_req);


	ndwd_run(name_req);

	fprintf(fp, "ndw_main stop!\n");
	fflush(fp);

	fclose(fp);

	return 1;

}

#endif