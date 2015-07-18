/*
 * PathPlanner.h
 *
 *  Created on: Jun 7, 2015
 *      Author: user
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include "Map.h";
#include "cell.h";

#include <set>

using std::set;

class PathPlanner {
public:

	vector<vector<grid_data> > _grid;
	cell _start;
	cell _goal;
	vector<cell> _path;
	set<cell> _open_list;
	set<cell> _close_list;

	PathPlanner(vector<vector<grid_data> > grid, cell start, cell goal);
	vector<cell> astar();

	virtual ~PathPlanner();

private:
	double heuristic_cost_estimate(cell cell_from);
	double g_cost(cell cell_from, cell cell_to);
	void fill_heuristic();
	void fill_g_f(cell cell_from);
	void reconstruct_path();
	cell find_lowest_f_score();
	bool check_in_set(set<cell> nodes_set, int row_index, int cols_index);
};

#endif /* PATHPLANNER_H_ */
