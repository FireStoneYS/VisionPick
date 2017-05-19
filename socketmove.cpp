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
#include <math.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Serial.h"
#include "SaveOption.h"
#include "socketmove.h"

using namespace std;

#define COUNTNUM  15



Eigen::Vector3f rotationvector(Eigen::Vector4f dst4f){
	Eigen::Vector3f dst;
	dst << dst4f[0],dst4f[1],dst4f[2];
	cv::Mat vector(3,1,CV_32FC1);
	Eigen::Vector3f out,axis;
	Eigen::Vector3f src(pow(2,0.5)/2,pow(2,0.5)/2,0);
	axis = dst.cross(src);
	float rx = axis[0]/axis.norm(),ry = axis[1]/axis.norm(),rz = axis[2]/axis.norm();
	float len_src = src.norm(),len_dst = dst.norm();
	float theta = acos(src.dot(dst)/len_src/len_dst);
	vector.at<float>(0,0) = rx*theta;
	vector.at<float>(1,0) = ry*theta;
	vector.at<float>(2,0) = rz*theta;
	cv::Mat matrix(3,3,CV_32FC1);
	cv::Rodrigues(vector,matrix);
	Eigen::Matrix3f matrix_eigen;
	for (int m = 0; m < 3; m++)
		for (int n=0; n < 3; n++)
		{
			matrix_eigen(m,n) = matrix.at<float>(m,n);
		}
	Eigen::AngleAxisf rotation_y(M_PI,Eigen::Vector3f::UnitY());
	Eigen::Matrix3f matrix_y = rotation_y.toRotationMatrix();
	matrix_eigen = matrix_eigen*matrix_y;
//	matrix_eigen <<
//		 cos(theta)+(1-cos(theta))*rx*rx,(1-cos(theta))*rx*ry-rz*sin(theta),(1-cos(theta))*rx*rz+ry*sin(theta),
//		 (1-cos(theta))*rx*ry+rz*sin(theta),cos(theta)+(1-cos(theta))*ry*ry,(1-cos(theta))*ry*rz - rx*sin(theta),
//		 (1-cos(theta))*rx*rz - ry*sin(theta),(1-cos(theta))*ry*rz + rx*sin(theta),cos(theta) + (1-cos(theta))*rz*rz;
	for (int m = 0; m < 3; m++)
		for (int n=0; n < 3; n++)
		{
			matrix.at<float>(m,n) = matrix_eigen(m,n) ;
		}
	cv::Rodrigues(matrix,vector);
	out[0] = vector.ptr<float>(0)[0];
	out[1] = vector.ptr<float>(1)[0];
	out[2] = vector.ptr<float>(2)[0];
	out[0] = rx*theta;
	out[1] = ry*theta;
	out[2] = rz*theta;

	return out;
}

void Sorting(int&csk,TargetPose &tp){
	Eigen::Vector3f cuboid_target,cylinder_target;
	float realx,realy,realz;
	cuboid_target<<0.369,0.589,0.21;
	cylinder_target<<0.493,0.439,0.21;
	Eigen::Vector4f Z;
	Z<<0,0,1,0;
	HandControl hc;
	Eigen::Vector4f target_point;
	target_point =Table2Manipulate()*tp.centroid;
	realx = target_point[0];
	realy = target_point[1];
	realz = target_point[2];
	if(tp.number >= 4){
		float theta = acos(Z.dot(tp.z_axis)/tp.z_axis.norm())/M_PI*180.0;
		if(tp.number == 5 && (abs(theta) <20 || abs(theta) >160)){
			socketmove(csk,realx,realy,realz+0.15,0,3.14,0);
			hc.TriangleOpen();
			sleep(3);
			socketmove(csk,realx,realy,realz+0.05,0,3.14,0);
			sleep(1);
			hc.TrianglePick();
			sleep(2);
			socketmove(csk,realx,realy,realz+0.15,0,3.14,0);
			sleep(3);
			socketmove(csk,cylinder_target[0],cylinder_target[1],realz+0.15,0,3.14,0);
			sleep(3);
			socketmove(csk,cylinder_target[0],cylinder_target[1],cylinder_target[2],0,3.14,0);
			sleep(1);
			hc.TriangleOpen();
			sleep(2);
			socketmove(csk,cylinder_target[0],cylinder_target[1],realz+0.15,0,3.14,0);
			sleep(3);
		}
		else if(tp.number == 4){
			socketmove(csk,realx,realy,realz+0.15,0,3.14,0);
			hc.TriangleOpen();
			sleep(3);
			socketmove(csk,realx,realy,realz,0,3.14,0);
			sleep(1);
			hc.TrianglePick();
			sleep(2);
			socketmove(csk,realx,realy,realz+0.15,0,3.14,0);
			sleep(3);
			socketmove(csk,cylinder_target[0],cylinder_target[1],realz+0.15,0,3.14,0);
			sleep(3);
			socketmove(csk,cylinder_target[0],cylinder_target[1],cylinder_target[2],0,3.14,0);
			sleep(1);
			hc.TriangleOpen();
			sleep(2);
			socketmove(csk,cylinder_target[0],cylinder_target[1],realz+0.15,0,3.14,0);
			sleep(3);
		}
		else{
			socketmove(csk,realx,realy,realz+0.15,0,3.14,0);
			hc.ParrelOpen();
			sleep(3);
			socketmove(csk,realx,realy,realz,0,3.14,0);
			sleep(1);
			hc.ParrelPick();
			sleep(2);
			socketmove(csk,realx,realy,realz+0.15,0,3.14,0);
			sleep(3);
			socketmove(csk,cylinder_target[0],cylinder_target[1],realz+0.15,0,3.14,0);
			sleep(3);
			socketmove(csk,cylinder_target[0],cylinder_target[1],cylinder_target[2],0,3.14,0);
			sleep(1);
			hc.ParrelOpen();
			sleep(2);
			socketmove(csk,cylinder_target[0],cylinder_target[1],realz+0.15,0,3.14,0);
			sleep(3);
		}
	}
	else{
		socketmove(csk,realx,realy,realz+0.15,0,3.14,0);
		hc.ParrelOpen();
		sleep(3);
		socketmove(csk,realx,realy,realz,0,3.14,0);
		sleep(1);
		hc.ParrelPick();
		sleep(2);
		socketmove(csk,realx,realy,realz+0.15,0,3.14,0);
		sleep(3);
		socketmove(csk,cuboid_target[0],cuboid_target[1],realz+0.15,0,3.14,0);
		sleep(3);
		socketmove(csk,cuboid_target[0],cuboid_target[1],cuboid_target[2],0,3.14,0);
		sleep(1);
		hc.ParrelOpen();
		sleep(2);
		socketmove(csk,cuboid_target[0],cuboid_target[1],realz+0.15,0,3.14,0);
		sleep(3);
	}
}

void linearpick(int&csk,TargetPose &tp){
	char command[512];
	double realx,realy,realz,rx,ry,rz;
	Eigen::Vector4f corner(-0.362,0.05,0,1);
	corner = Table2Manipulate()*corner;
	Eigen::Vector3f ro_vec;
	Eigen::Vector4f X,Z,target_point;
	X<<1,0,0,0;
	Z<<0,0,1,0;
	float anglez=acos(Z.dot(tp.z_axis)/tp.z_axis.norm())/PI*180;
	HandControl hc;
	if(abs(anglez) <20 || abs(anglez) >160){
		target_point =Table2Manipulate()*tp.centroid;
		ro_vec = rotationvector(Table2Manipulate()*tp.z_axis);
//		cout<<ro_vec<<endl;
		realx = target_point[0];
		realy = target_point[1];
		realz = target_point[2];
		rx = ro_vec[0];
		ry = ro_vec[1];
		rz = ro_vec[2];
		socketmove(csk,realx,realy,realz+0.15,rx,ry,rz);
		hc.ParrelOpen();
		sleep(3);
		socketmove(csk,realx,realy,realz,rx,ry,rz);
		sleep(1);
		hc.ParrelPick();
		sleep(2);
		socketmove(csk,realx,realy,realz+0.15,rx,ry,rz);
		sleep(3);
		socketmove(csk,corner[0],corner[1],realz+0.15,rx,ry,rz);
		sleep(3);
		socketmove(csk,corner[0],corner[1],realz,rx,ry,rz);
		sleep(1);
		hc.ParrelOpen();
		sleep(2);
		socketmove(csk,corner[0],corner[1],realz+0.15,rx,ry,rz);
		sleep(3);
	}
	else{
		target_point =Table2Manipulate()*tp.centroid;
		ro_vec = rotationvector(Table2Manipulate()*tp.z_axis);
		realx = target_point[0];
		realy = target_point[1];
		realz = target_point[2];
		rx = ro_vec[0];
		ry = ro_vec[1];
		rz = ro_vec[2];
		socketmove(csk,realx,realy,realz+0.15,rx,ry,rz);
		hc.ParrelOpen();
		sleep(3);
		socketmove(csk,realx,realy,realz,rx,ry,rz);
		sleep(1);
		hc.ParrelPick();
		sleep(2);
		socketmove(csk,realx,realy,realz+0.15,rx,ry,rz);
		sleep(3);
		socketmove(csk,corner[0],corner[1],realz+0.15,rx,ry,rz);
		sleep(3);
		socketmove(csk,corner[0],corner[1],realz,rx,ry,rz);
		sleep(1);
		hc.ParrelOpen();
		sleep(2);
		socketmove(csk,corner[0],corner[1],realz+0.15,rx,ry,rz);
		sleep(3);
	}
}
void symcenterpick(int&csk,TargetPose &tp){
	float realx,realy,realz;
	Eigen::Vector4f corner(-0.362,0.05,0,1);
	corner = Table2Manipulate()*corner;
	HandControl hc;
	Eigen::Vector4f target_point;
	target_point =Table2Manipulate()*tp.centroid;
	realx = target_point[0];
	realy = target_point[1];
	realz = target_point[2];
	socketmove(csk,realx,realy,realz+0.15,0,3.14,0);
	hc.TriangleOpen();
	sleep(3);
	socketmove(csk,realx,realy,realz,0,3.14,0);
	sleep(1);
	hc.TrianglePick();
	sleep(2);
	socketmove(csk,realx,realy,realz+0.15,0,3.14,0);
	sleep(3);
	socketmove(csk,corner[0],corner[1],realz+0.15,0,3.14,0);
	sleep(3);
	socketmove(csk,corner[0],corner[1],realz,0,3.14,0);
	sleep(1);
	hc.TriangleOpen();
	sleep(2);
	socketmove(csk,corner[0],corner[1],realz+0.15,0,3.14,0);
	sleep(3);
}
void socketmove(int&csk,float x,float y,float z,float rx,float ry,float rz){
	char command[512];
	sprintf(command,"movej(p[%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf],a=1.396, v=1.047)\n",x,y,z,rx,ry,rz);
	send(csk, command, strlen(command), 0);
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
//	prepareformove(csk);
//	sleep(5);
	return 0;


}
void socketclose(int& csk){
	char* zero = "movej(p[-0.02, -0.19145, 1.0, 0.0, 2.2214, -2.2214], a=1.3962634015954636, v=1.0471975511965976)\n";
	send(csk, zero, strlen(zero), 0);
	close(csk);
}

void prepareformove(int& csk){
	char* wristdown = "movej(p[0.4, 0.0, 0.5, 0, 3.14, 0.0], a=1.3962634015954636, v=1.0471975511965976)\n";
	send(csk, wristdown, strlen(wristdown), 0);
}



