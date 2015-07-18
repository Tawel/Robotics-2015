/*
 * GoToWayPoint.h
 *
 *  Created on: Jun 20, 2015
 *      Author: colman
 */

#ifndef GOTOWAYPOINT_H_
#define GOTOWAYPOINT_H_

#include "Behavior.h"
#include "../WaypointsManager.h"

class GoToWayPoint: public Behavior {
public:
	GoToWayPoint(Robot* robot, WaypointsManager* Wpm);
	bool startCond();
	bool stopCond();
	void action();
	void print();
	virtual ~GoToWayPoint();

private:
	WaypointsManager* _wpm;
	bool isRight;
	double diffrence;

};

#endif /* GOTOWAYPOINT_H_ */
