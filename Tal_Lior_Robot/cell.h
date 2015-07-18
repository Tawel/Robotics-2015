/*
 * cell.h
 *
 *  Created on: Jun 13, 2015
 *      Author: colman
 */

#ifndef CELL_H_
#define CELL_H_

#include "limits.h"

class cell {
public:
	cell();
	cell(double x, double y);

	double x_Coordinate;
	double y_Coordinate;

	virtual ~cell();
	bool operator<(const cell& cel) const
	{
		return  (y_Coordinate*INT_MAX + x_Coordinate < cel.y_Coordinate*INT_MAX + cel.x_Coordinate);
	}
};

#endif /* CELL_H_ */

