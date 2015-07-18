/*
 * GoToWayPoint.cpp
 *
 *  Created on: Jun 20, 2015
 *      Author: colman
 */

#include "GoToWayPoint.h"

GoToWayPoint::GoToWayPoint(Robot* robot, WaypointsManager* wpm):Behavior(robot) {
	_wpm = wpm;
}

GoToWayPoint::~GoToWayPoint() {
	// TODO Auto-generated destructor stub
}

// here we set if the robot should move right or left towards the nearest waypoint
bool GoToWayPoint::startCond()
{
	double yaw = _robot->getYaw();
	diffrence = _wpm->currWayPoint.yaw - yaw;

	if(diffrence == 0)
	{
		return false;
	}
	else if(diffrence >= 180)
	{
		isRight =  1;
		diffrence = 360 - diffrence;
	}
	else if (diffrence < -180)
	{
		isRight = 0;
		diffrence = 360 + diffrence;
	}
	else if (diffrence > 0)
	{
		isRight = 0;
	}
	else
	{
		isRight = 1;
		diffrence = abs(diffrence);
	}

	if(isRight)
	{
		return (_robot->checkRange(0, diffrence * ONE_DEGREE_INDEXES));
	}
	else
	{
		return(_robot->checkRange( _robot->getLaserSpec() - (diffrence * ONE_DEGREE_INDEXES),_robot->getLaserSpec() ));
	}
}
bool GoToWayPoint::stopCond()
{
	if(isRight)
	{
		return (!(_robot->checkRange(0, diffrence * ONE_DEGREE_INDEXES)) || (abs(_robot->getYaw() - _wpm->currWayPoint.yaw) <= MAX_YAW_DIFF));
	}
	else
	{
		return(!(_robot->checkRange( _robot->getLaserSpec() - (diffrence * ONE_DEGREE_INDEXES),_robot->getLaserSpec())) || (abs(_robot->getYaw() - _wpm->currWayPoint.yaw) <= MAX_YAW_DIFF)) ;
	}

}
void GoToWayPoint::action()
{
	if(isRight)
	{
		_robot->setSpeed(0.0, TURN_RIGHT_SPEED);
	}
	else
	{
		_robot->setSpeed(0.0, TURN_LEFT_SPEED);
	}
}
