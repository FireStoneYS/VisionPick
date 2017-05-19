/*
 * socketmove.h
 *
 *  Created on: Apr 11, 2017
 *      Author: abds
 */

#ifndef SOCKETMOVE_H_
#define SOCKETMOVE_H_

Eigen::Vector3f rotationvector(Eigen::Vector4f dst4f);
void Sorting(int&csk,TargetPose &tp);
void linearpick(int&csk,TargetPose &tp);
void symcenterpick(int&csk,TargetPose &tp);

void socketmove(int&csk,float x,float y,float z,float rx,float ry,float rz);
int socketInit(int& csk);
void socketclose(int& csk);
void prepareformove(int& csk);


#endif /* SOCKETMOVE_H_ */
