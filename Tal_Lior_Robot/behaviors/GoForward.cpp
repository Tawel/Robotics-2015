/*
 * GoForward.cpp
 *
 *  Created on: Jun 14, 2015
 *      Author: user
 */

#include "GoForward.h"
#include <iostream>
#include <math.h>
#include "../Defines.h"

bool GoForward::startCond()
{
	return (_robot->checkRange(_robot->getLaserSpec() / 2 - LASER_SPEC, _robot->getLaserSpec() / 2 + LASER_SPEC));
}

bool GoForward::stopCond()
{
	_robot->Read();
	return (!startCond() || (_wpm->isInWayPoint(_robot->getXpos(), _robot->getYpos())));
}

void GoForward::action()
{
	_robot->setSpeed(MOVE_SPEED, 0.0);
}

GoForward::GoForward(Robot* robot, WaypointsManager* wpm):Behavior(robot) {
	_wpm = wpm;
}

GoForward::~GoForward() {
	// TODO Auto-generated destructor stub
}

void GoForward::print()
{

}
