/*
 * PathPlanner.cpp
 *
 *  Created on: Jun 7, 2015
 *      Author: user
 */

#include "PathPlanner.h"
#include <stdlib.h>
#include <math.h>

PathPlanner::PathPlanner(vector<vector<grid_data> > grid, cell start, cell goal){
	_grid = grid;
	_start = start;
	_goal = goal;
//	_open_list.resize( _grid.size() * (_grid[0]).size());
}

double PathPlanner::heuristic_cost_estimate(cell cell_from){
	int dx = abs(cell_from.x_Coordinate - _goal.x_Coordinate);
	int dy = abs(cell_from.y_Coordinate - _goal.y_Coordinate);
	double multiple_coordinates = dx^2 + dy^2;
    return sqrt (multiple_coordinates);
}

double PathPlanner::g_cost(cell cell_from, cell cell_to)
{
	if(( cell_from.x_Coordinate != cell_to.x_Coordinate) && ( cell_from.y_Coordinate != cell_to.y_Coordinate))
		{
			return (_grid[cell_from.y_Coordinate][cell_from.x_Coordinate].g_val + 14);
		}
		else
		{
			return (_grid[cell_from.y_Coordinate][cell_from.x_Coordinate].g_val + 10);
		}
}


void PathPlanner::fill_heuristic( )
{
	double hVal;
	for (int i = 0; i < _grid.size(); i++)
	{
		for(int j = 0; j< _grid[i].size(); j++)
		{
			if(_grid[i][j].cell_color == 0){
				cell cl(j, i);
				hVal = heuristic_cost_estimate(cl);
				_grid[i][j].h_val = hVal;
			}
		}
	}
}

void PathPlanner::fill_g_f(cell cell_from){
	int i,j;
	double currentGval;
	bool in_open_list = false;

	_close_list.insert(cell_from);
	for(i=cell_from.y_Coordinate-1; i <=cell_from.y_Coordinate+1; i++)
	{
		for(j=cell_from.x_Coordinate-1; j <= cell_from.x_Coordinate+1; j++)
		{
			if((i>=0)&&(j>=0)&&(i<_grid.size())&&(j<_grid[0].size()))
			{
				if (i == _goal.y_Coordinate && j == _goal.x_Coordinate )
				{
					cell cl2(j, i);
					_grid[i][j].parent.x_Coordinate = cell_from.x_Coordinate;
					_grid[i][j].parent.y_Coordinate = cell_from.y_Coordinate;
					_grid[i][j].g_val = 0;
					_grid[i][j].f_val = 0;
					_open_list.insert(cl2);
				}
				else {

					bool is_close = check_in_set(_close_list,i,j);
					if((_grid[i][j].cell_color == 0)&&(((j != cell_from.x_Coordinate)||(i != cell_from.y_Coordinate)))&&(!is_close))
					{
					    cell cl(j, i);
						currentGval = g_cost(cell_from, cl);
						in_open_list = check_in_set(_open_list,i,j);
						if(in_open_list){
							if (_grid[i][j].g_val > currentGval){
								_grid[i][j].g_val = currentGval;
								_grid[i][j].f_val = _grid[i][j].g_val + _grid[i][j].h_val;
								_grid[i][j].parent.x_Coordinate = cell_from.x_Coordinate;
								_grid[i][j].parent.y_Coordinate = cell_from.y_Coordinate;
							}
						}
						else{
							_grid[i][j].g_val = currentGval;
							_grid[i][j].f_val = _grid[i][j].g_val + _grid[i][j].h_val;
							_grid[i][j].parent.x_Coordinate = cell_from.x_Coordinate;
							_grid[i][j].parent.y_Coordinate = cell_from.y_Coordinate;
							_open_list.insert(cl);
						}
					}
				}
			}
		}
	}
}


bool PathPlanner::check_in_set(set<cell> nodes_set, int row_index, int cols_index){
	cell current_cell;
	set<cell>::iterator it;

	if (!nodes_set.empty()) {
		for (it = nodes_set.begin(); it != nodes_set.end(); ++it) {
			current_cell = *it;

			if ((current_cell.x_Coordinate == cols_index ) &&
				(current_cell.y_Coordinate == row_index )){
				return true;
		   }
		}
	}

	return false;
}

PathPlanner::~PathPlanner() {
	// TODO Auto-generated destructor stub
}

vector<cell> PathPlanner::astar()
{
	this->fill_heuristic();
	this->fill_g_f(_start);
	int size = _open_list.size();
	while (!_open_list.empty())
	{
		cell lowest_f_cell = find_lowest_f_score();
		if (lowest_f_cell.x_Coordinate == _goal.x_Coordinate &&
		    (lowest_f_cell.y_Coordinate == _goal.y_Coordinate))
		{
			reconstruct_path();
			return _path;
		}

		std::set<cell>::iterator it_openlist;
		std::set<cell>::iterator it_closelist;
		_open_list.erase(lowest_f_cell);
		fill_g_f(lowest_f_cell);
	}

	return _path;
}
cell PathPlanner::find_lowest_f_score(){
	set<cell>::iterator it;
	cell min_cell = *_open_list.begin();
	cell current_cell;

	for (it = _open_list.begin(); it != _open_list.end(); ++it) {
		current_cell = *it;

		if((_grid[current_cell.y_Coordinate][current_cell.x_Coordinate]).f_val <
		   (_grid[min_cell.y_Coordinate][min_cell.x_Coordinate]).f_val)
		{
			min_cell = current_cell;
		}
	}

	return min_cell;
}

void PathPlanner::reconstruct_path()
{
	double current_x_coordinate, current_y_coordinate, pre_x_coordinate, pre_y_coordinate;
	int i = 1;
	vector<cell> path;

	current_x_coordinate = _goal.x_Coordinate;
	current_y_coordinate = _goal.y_Coordinate;

	cell currCell(current_x_coordinate, current_y_coordinate);

	while ((current_x_coordinate != _start.x_Coordinate)||(current_y_coordinate != _start.y_Coordinate)){
		i++;
		pre_x_coordinate = current_x_coordinate;
		pre_y_coordinate = current_y_coordinate;

		current_x_coordinate = _grid[pre_y_coordinate][pre_x_coordinate].parent.x_Coordinate;
		current_y_coordinate = _grid[pre_y_coordinate][pre_x_coordinate].parent.y_Coordinate;

	}

	path.resize(i);
	_path.resize(i);

	i = 0;

	current_x_coordinate = _goal.x_Coordinate;
	current_y_coordinate = _goal.y_Coordinate;

	cell currCell2(current_x_coordinate, current_y_coordinate);

	path[0] = currCell2;

	while ((current_x_coordinate != _start.x_Coordinate)||(current_y_coordinate != _start.y_Coordinate)){
		i++;
		pre_x_coordinate = current_x_coordinate;
		pre_y_coordinate = current_y_coordinate;

		current_x_coordinate = _grid[pre_y_coordinate][pre_x_coordinate].parent.x_Coordinate;
		current_y_coordinate = _grid[pre_y_coordinate][pre_x_coordinate].parent.y_Coordinate;
		cell currCell3(current_x_coordinate, current_y_coordinate);
		path[i] = currCell3;
	}

	for (int j = path.size() - 1; j >= 0; j--){
		_path[path.size() -1 - j] = path[j];
	}
}
