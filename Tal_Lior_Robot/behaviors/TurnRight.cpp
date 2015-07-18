
#include "TurnRight.h"

bool TurnRight::startCond(){
	return (_robot->isRightFree());
}

bool TurnRight::stopCond(){
	return (_robot->isForwardFree());
}

void TurnRight::action(){
	_robot->setSpeed(0.0, TURN_RIGHT_SPEED);
}

TurnRight::TurnRight(Robot* robot):Behavior(robot) {
	// TODO Auto-generated constructor stub
}

TurnRight::~TurnRight() {
	// TODO Auto-generated destructor stub
}

void TurnRight::print(){
	//cout << "TurnRight..." << endl;
}
