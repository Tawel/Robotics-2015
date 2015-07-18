/*
 * cellcoordinate.cpp
 *
 *  Created on: Jun 13, 2015
 *      Author: colman
 */

#include "cellcoordinate.h"


cell_coordinate::cell_coordinate(double x, double y){
	x_Coordinate = x;
	y_Coordinate = y;
}

cell_coordinate::cell_coordinate()
{
	x_Coordinate = 0;
	y_Coordinate = 0;
}

cell_coordinate::~cell_coordinate() {
	// TODO Auto-generated destructor stub
}

