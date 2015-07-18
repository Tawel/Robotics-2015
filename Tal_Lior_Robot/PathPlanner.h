/*
 * PathPlanner.h
 *
 *  Created on: Jun 7, 2015
 *      Author: user
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include "Map.h";
#include "cellcoordinate.h";

#include <set>

using std::set;

class PathPlanner {
public:

	vector<vector<grid_data> > _grid;
	cell_coordinate _start;
	cell_coordinate _goal;
	vector<cell_coordinate> _path;
	set<cell_coordinate> _open_list;
	set<cell_coordinate> _close_list;

	PathPlanner(vector<vector<grid_data> > grid, cell_coordinate start, cell_coordinate goal);
	vector<cell_coordinate> astar();

	virtual ~PathPlanner();

private:
	double heuristic_cost_estimate(cell_coordinate cell_from);
	double g_cost(cell_coordinate cell_from, cell_coordinate cell_to);
	void fill_heuristic();
	void fill_g_f(cell_coordinate cell_from);
	void reconstruct_path();
	cell_coordinate find_lowest_f_score();
	bool check_in_set(set<cell_coordinate> nodes_set, int row_index, int cols_index);
};

#endif /* PATHPLANNER_H_ */
