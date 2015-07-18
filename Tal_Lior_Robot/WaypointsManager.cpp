/*
 * WaypointsManager.cpp
 *
 *  Created on: Jun 14, 2015
 *      Author: colman
 */

#include "WaypointsManager.h"
#include <math.h>
using namespace std;

WaypointsManager::WaypointsManager(vector<cell> path, double gridResolution, double mapResolution)
{
	astar_path = path;
	_gridResolution = gridResolution;
	_mapResolution = mapResolution;
}

// Creating way point every "num_of_cells" item on the path
void WaypointsManager::build_way_point_vector(int num_of_cells)
{
	set<wayPoint>::iterator iter = wayPoints.begin();
	int counter = 0;
	double m;

	for (int i = 0; i < astar_path.size(); i++)
	{
		if(i == 0)
		{
			m = calc_incline(astar_path[i],astar_path[i+1]);
			wayPoint wp(astar_path[i].x_Coordinate, astar_path[i].y_Coordinate , calc_yaw(m, astar_path[i], astar_path[i+1]));
			wayPoints.insert(iter,wp);
			++iter;

			currWayPoint.x_Coordinate = wp.x_Coordinate;
			currWayPoint.y_Coordinate = wp.y_Coordinate;
			currWayPoint.yaw = wp.yaw;

			nextWayPoint.x_Coordinate = wp.x_Coordinate;
			nextWayPoint.y_Coordinate = wp.y_Coordinate;
			nextWayPoint.yaw = wp.yaw;

		}
		else if ( i== (astar_path.size() - 1))
		{
			wayPoint wp(astar_path[i].x_Coordinate, astar_path[i].y_Coordinate , calc_yaw(m, astar_path[i-1], astar_path[i]));
			wayPoints.insert(iter,wp);
			++iter;
		}
		else if (counter == num_of_cells)
		{
			m = calc_incline(astar_path[i], astar_path[i+1]);

			wayPoint wp(astar_path[i].x_Coordinate, astar_path[i].y_Coordinate , calc_yaw(m, astar_path[i], astar_path[i+1]));
			wayPoints.insert(iter,wp);
			++iter;
			counter = 0;
		}
		else
		{
			double new_m;
			bool is_old_varticle;
			is_old_varticle = is_verticle;
			new_m = calc_incline(astar_path[i], astar_path[i+1]);
			if((new_m != m)||(is_old_varticle != is_verticle))
			{
				wayPoint wp(astar_path[i].x_Coordinate, astar_path[i].y_Coordinate, calc_yaw(new_m, astar_path[i], astar_path[i+1]));
				wayPoints.insert(iter,wp);
				++iter;
				counter = 0;
			}
			m = new_m;
		}

		counter++;
	}
}

double WaypointsManager::calc_yaw(double m, cell cell_from, cell cell_to)
{
	double angle;

	if(!is_verticle)
	{
		angle = 180 * atan(m) / M_PI;
	}

	if (is_verticle)
	{
		if (cell_to.y_Coordinate > cell_from.y_Coordinate)
		{
			return (270);
		}
		else
		{
			return (90);
		}
	}
	else if ( m == 0)
	{
		if (cell_to.x_Coordinate > cell_from.x_Coordinate)
		{
			return (angle);
		}
		else
		{
			return (180 + angle);
		}
	}
	else if (m > 0)
	{
		if (cell_to.y_Coordinate > cell_from.y_Coordinate)
		{
			return (360 - angle);
		}
		else
		{
			return (180 - angle);
		}
	}
	else
	{
		if (cell_to.y_Coordinate > cell_from.y_Coordinate)
		{
			return (180 + angle);
		}
		else
		{
			return (angle);
		}
	}

//	double angle;
//
//	if(!is_verticle)
//	{
//		angle = 180 * atan(m) / M_PI;
//	}
//
//	if (is_verticle)
//	{
//		if (cell_to.y_Coordinate > cell_from.y_Coordinate)
//		{
//			return (90);
//		}
//		else
//		{
//			return (270);
//		}
//	}
//	else if ( m == 0)
//	{
//		if (cell_to.x_Coordinate > cell_from.x_Coordinate)
//		{
//			return (angle);
//		}
//		else
//		{
//			return (180 + angle);
//		}
//	}
//	else if (m > 0)
//	{
//		if (cell_to.y_Coordinate > cell_from.y_Coordinate)
//		{
//			return (angle);
//		}
//		else
//		{
//			return (180 + angle);
//		}
//	}
//	else
//	{
//		if (cell_to.y_Coordinate > cell_from.y_Coordinate)
//		{
//			return (90 + angle);
//		}
//		else
//		{
//			return (360 - angle);
//		}
//	}
}

double WaypointsManager::calc_incline(cell cell_from, cell cell_to)
{
	is_verticle = 0;
	if(cell_from.x_Coordinate == cell_to.x_Coordinate)
	{
		is_verticle = 1;
		// TODO- CHECK if ok
		return(9999);
	}
	else
	{
		return((cell_to.y_Coordinate - cell_from.y_Coordinate) / (cell_to.x_Coordinate - cell_from.x_Coordinate));
	}
}

void WaypointsManager::setNextWayPoint(wayPoint Next)
{
	currWayPoint.x_Coordinate = nextWayPoint.x_Coordinate;
	currWayPoint.y_Coordinate = nextWayPoint.y_Coordinate;
	currWayPoint.yaw = nextWayPoint.yaw;

	nextWayPoint.x_Coordinate = Next.x_Coordinate;
	nextWayPoint.y_Coordinate = Next.y_Coordinate;
	nextWayPoint.yaw = Next.yaw;
}


bool WaypointsManager::isInWayPoint(double x,double y)
{
	double dx = nextWayPoint.x_Coordinate - x;
	double dy = nextWayPoint.y_Coordinate - y;
	double distance = sqrt(pow(dx, 2) + pow(dy, 2));

	cout << "way point x"<< " " << nextWayPoint.x_Coordinate << " " << "x" << " " << x << endl;
	cout << "way point y" << " " << nextWayPoint.y_Coordinate << " " << "y" << " "  << y << endl;
	cout << "yaw" << nextWayPoint.yaw <<  endl;
	cout << (distance) << endl;

	//if (distance*_gridResolution <= TOLLERANCE)
	if (distance*_gridResolution <= TOLLERANCE)
	{
		return true;
	}
	return false;
}

WaypointsManager::~WaypointsManager() {
	// TODO Auto-generated destructor stub
}
