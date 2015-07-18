/*
 * TurnRight.h
 *
 *  Created on: Dec 14, 2014
 *      Author: user
 */

#ifndef TURNRIGHT_H_
#define TURNRIGHT_H_

#include "Behavior.h"
#include <iostream>

class TurnRight: public Behavior {
public:

	TurnRight(Robot* robot);
	bool startCond();
	bool stopCond();
	void action();
	void print();
	virtual ~TurnRight();
};

#endif /* TURNRIGHT_H_ */
