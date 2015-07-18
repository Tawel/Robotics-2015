/*
 * cell.cpp
 *
 *  Created on: Jun 13, 2015
 *      Author: colman
 */

#include "cell.h"


cell::cell(double x, double y){
	x_Coordinate = x;
	y_Coordinate = y;
}

cell::cell()
{
	x_Coordinate = 0;
	y_Coordinate = 0;
}

cell::~cell() {
	// TODO Auto-generated destructor stub
}

