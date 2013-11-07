/*
 ============================================================================
 Name        : ESP-Lab3.c
 Author      : Rohit Khanna
 Version     :
 Copyright   : Your copyright notice
 Description : user-app for ESP Lab3

  	15 address space = 32KB, Page size = 64 B therefore we have 32KB/64B pages = 2^9 = 512 pages
  	 global pointer to maintain the current page position of EEPROM where it can write.
  	 Will vary from 0 to 511

 ============================================================================
 */


#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>

int FILE_DESC = 0;
uint PAGE_SIZE = 64;


/*
 * int seek_EEPROM(int offset): to set the current page position in the EEPROM to offset which is
 * the page number in the range of 0 to 2k-1. Returns the page_number seeked to, or
 * –1 otherwise.
 */

int seek_EEPROM(int offset){				//offset is page number
	int page_number = offset;
	printf("APP: user seek_EEPROM()\n");
	int res;

	if ((res=lseek(FILE_DESC, page_number, SEEK_SET)) != page_number) {
		fprintf(stderr,"APP: Error: seek_EEPROM()- Failed to write to the i2c bus: %s, res=%d\n",strerror(errno), res);
		return -1;
	}
	printf("APP: seek successful at address=%d\n", res);
	return res;
}


void writeOutput(int ret){

	switch (ret){
		case -2 : printf("APP: Write Request submitted to EEPROM !!\n");
		break;

		case -1 : printf("APP: EEPROM Busy Writing !!\n");
		break;

		case 0 : printf("APP: Write to EEPROM successful !!\n");
		break;

		case -3 : printf("APP: Write to EEPROM Failed !!\n");
		break;

		default: printf("APP: Unknown return value from i2c-flash !! \n");
	}
}

void readOutput(int ret, char *rd_buffer, int page_count){

	int k;
	switch (ret){
		case -2 : printf("APP: Read Request submitted to EEPROM !!\n");
		break;

		case -1 : printf("APP: EEPROM Busy Reading!!\n");
		break;

		case 0 : printf("APP: Read from EEPROM successful !!\n");
		printf("APP : Data read from EEPROM -->\n");
		for(k=0; k<page_count*PAGE_SIZE; k++){
			printf("%c", rd_buffer[k]);
		}
		printf("\n");
		break;
		default: printf("APP: Unknown return value from i2c-flash !! \n");
	}
}



int main(){

	int i;
	int page_count=5;
	char wr_buffer[(page_count*PAGE_SIZE)];
	char rd_buffer[page_count*PAGE_SIZE];
	int res=0;

	/*	OPEN the file	*/
	if ((FILE_DESC = open("/dev/i2c-flash",O_RDWR)) < 0) {
		fprintf(stderr,"APP: Error: Failed to open the bus: %s\n",strerror(errno));
		exit(1);
	}
	printf("APP: File opened successfully\n");

	/* Populate the buffer to write	*/
	for(i=0; i<(page_count*PAGE_SIZE); i++){
		wr_buffer[i]='*';
	}

	/*	SEEK to page number	*/
	if(seek_EEPROM(510) ==-1)											//
		return -1;

	/*	write until it returns with 0	*/
	while((res=write(FILE_DESC, wr_buffer, page_count)) !=0){
		writeOutput(res);
		usleep(300000);
	}
	writeOutput(res);

	/*	SEEK to page number	*/
	if(seek_EEPROM(510) ==-1)											//
		return -1;

	while( (res=read(FILE_DESC,rd_buffer, page_count)) != 0)
	{
		readOutput(res, rd_buffer, page_count);
		usleep(300000);
	}
	readOutput(res, rd_buffer, page_count);

	return 0;
}


