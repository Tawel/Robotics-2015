
#include "TurnLeft.h"

bool TurnLeft::startCond(){
	return (!_robot->isRightFree() && !_robot->isForwardFree());
}

bool TurnLeft::stopCond(){
	return (_robot->isForwardFree());
}

void TurnLeft::action(){
	_robot->setSpeed(0.0, TURN_LEFT_SPEED);
}

TurnLeft::TurnLeft(Robot* robot):Behavior(robot) {
	// TODO Auto-generated constructor stub
}

TurnLeft::~TurnLeft() {
	// TODO Auto-generated destructor stub
}

void TurnLeft::print(){
	//cout << "TurnLeft..." << endl;
}
