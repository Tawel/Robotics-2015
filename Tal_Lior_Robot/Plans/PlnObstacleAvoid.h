/*
 * PlnObstacleAvoid.h
 *
 *  Created on: Dec 21, 2014
 *      Author: user
 */

#ifndef PLNOBSTACLEAVOID_H_
#define PLNOBSTACLEAVOID_H_

#include "Plan.h"
#include "../WaypointsManager.h"

class PlnObstacleAvoid: public Plan {

public:
	PlnObstacleAvoid(Robot* robot,WaypointsManager* wpm);
	Behavior* getStartPoint();
	virtual ~PlnObstacleAvoid();
};

#endif /* PLNOBSTACLEAVOID_H_ */


