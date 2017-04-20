/*
 * socketmove.h
 *
 *  Created on: Apr 11, 2017
 *      Author: abds
 */

#ifndef SOCKETMOVE_H_
#define SOCKETMOVE_H_


void socketmove(int&csk,float x,float y,float z);
int socketInit(int& csk);
void socketclose(int& csk);
void prepareformove(int& csk);


#endif /* SOCKETMOVE_H_ */
