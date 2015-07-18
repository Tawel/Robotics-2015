/*
 * Map.h
 *
 *  Created on: Jun 2, 2015
 *      Author: colman
 */

#ifndef MAP_H_
#define MAP_H_

#include "Defines.h"
#include <iostream>
#include <vector>
#include <set>
#include "cell.h"
#include "ConfigurationManager.h"
#include "wayPoint.h"

using namespace std;

using std::vector;
using std::set;

struct grid_data
{
	char cell_color;
	double h_val;
	double g_val;
	double f_val;
	cell parent;
//	cell_coordinate current;
};


class Map {
private:
	void encodeOneStep(const char* filename, std::vector<unsigned char> image, unsigned width, unsigned height);
	void decodeOneStep(const char* filename);

//	vector<vector<grid_data> > convertMapToGrid(const char* filename, double map_resolution, double grid_resolution);

public:
	vector<vector<grid_data> > _original_grid;
	vector<vector<grid_data> > _thickened_grid;

	Map(){}
	vector<vector<grid_data> > convertMapToGrid(const char* filename, double map_resolution, double grid_resolution);
	void thickenMap(const char* filename, int thickenSizeCM);
	void createGrids(const char* originalMapFile, double map_resolution, double grid_resolution);
	double checkNearestObs(cell point, double angle);

	virtual ~Map();
};

#endif /* MAP_H_ */
