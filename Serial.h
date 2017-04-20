/*
 * Serial.h
 *
 *  Created on: Apr 13, 2017
 *      Author: abds
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <string>
#include <termios.h>
#include<stdio.h>
#include <fcntl.h>

class HandControl{
	public:
	std::string serial;
	int nSpeed,nBits,nStop;
	char nEvent;
	int fd;
	HandControl(std::string serial = "/dev/ttyUSB0", int nSpeed=9600,int nBits =8,char nEvent='N',int nStop=1){
		if((fd = open(serial.c_str(),O_RDWR|O_NOCTTY|O_NDELAY)) == -1){
			printf("open %s fail \n",serial);
		}

		if((set_opt(fd,nSpeed,nBits,'N',1)) == -1){
			printf("set %s fail \n",serial);
		}
	};
	int ParrelOpen();
	int ParrelPick();
	int TriangleOpen();
	int TrianglePick();
	int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
	void stop();
};

#endif /* SERIAL_H_ */
