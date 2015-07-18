
#ifndef GOLeft_H_
#define GOLeft_H_

#include "Behavior.h"
#include <iostream>
#include <math.h>

class TurnLeft: public Behavior {
public:

	TurnLeft(Robot* robot);
	bool startCond();
	bool stopCond();
	void action();
	void print();
	virtual ~TurnLeft();
};

#endif /* TURNLEFT_H_ */
