/*
 * socketmove.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: abds
 */

char* wristdown = "movej(p[0.40, 0.0, 0.600000, 3.14000000, 0.00000000, 0.000000000], a=1.3962634015954636, v=1.0471975511965976)\n";
#include <stdlib.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <iostream>
#include "socketmove.h"

using namespace std;

#define COUNTNUM  15

void socketmove(int&csk,float x,float y,float z){

	char command[512];
	double realx,realy,realz;
	realx = x;
	realy = y;
	realz = z;

	sprintf(command,"movej(p[%.3lf,%.3lf,%.3lf,0,3.14,0],a=1.3962634015954636, v=1.0471975511965976)\n",realx,realy,realz);
	send(csk, command, strlen(command), 0);
	cout<<realz<<endl;



}

int socketInit(int& csk){
	char* targetIP = "192.168.99.146";
	struct sockaddr_in server_add;
	unsigned short portnum = 30002;

	if((csk = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP))== -1){
		printf("create socket fail \n");
		return -1;
	}
	else{
		printf ("create socket success \n");
	}
	bzero(&server_add,sizeof(struct sockaddr_in));
//	int flags = fcntl(csk, F_GETFL, 0);
//	fcntl(csk, F_SETFL, flags|O_NONBLOCK);
	server_add.sin_family =AF_INET;
	server_add.sin_port = htons(portnum);
	server_add.sin_addr.s_addr = inet_addr(targetIP);
	printf("server_add = %#x ,port : %#x\r\n",server_add.sin_addr.s_addr,server_add.sin_port);

	if( -1 == (connect(csk,(struct sockaddr*)&server_add,sizeof(struct sockaddr_in)))){
		printf ("connect fail \n");
		return -1;
	}
	else{
		printf("connect success \n");
	}
	return 0;


}
void socketclose(int& csk){
	char* zero = "movej(p[-0.02, -0.19145, 1.0, 0.0, 2.2214, -2.2214], a=1.3962634015954636, v=1.0471975511965976)\n";
	send(csk, zero, strlen(zero), 0);
	close(csk);
}

void prepareformove(int& csk){
	char* wristdown = "movej(p[0.4, 0.0, 0.5, 3.1, 0.0, 0.0], a=1.3962634015954636, v=1.0471975511965976)\n";
	send(csk, wristdown, strlen(wristdown), 0);
}



