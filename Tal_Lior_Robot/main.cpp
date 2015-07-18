/*
 * main.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: user
 */

#include <iostream>
#include "Robot.h"
#include "Manager.h"
#include "Plans/ObstacleAvoidPlan.h"
#include "Map.h"
#include "PathPlanner.h"
#include "ConfigurationManager.h"
#include <vector>
#include "WaypointsManager.h"

using namespace std;

void printMatrix(vector<vector<grid_data> > grid);

int main()
{
	ConfigurationManager ConfigMgr(CONFIGURATION_PATH);

	Map map;
	map.thickenMap(ConfigMgr.map_path, ConfigMgr.robot_width);
	map.createGrids(ConfigMgr.map_path, ConfigMgr.map_resolution, ConfigMgr.grid_resolution);

	cout << map._original_grid.size() <<  "Grid Row" << endl;

	Robot robot("localhost",6665, &ConfigMgr, map._original_grid.size());
	//Robot robot("10.10.245.64",6665, &cm, map._original_grid.size());


	// printing
	printMatrix(map._original_grid);
	for (int var = 0; var < 10; var++) {
		cout << endl << "";
	}
	printMatrix(map._thickened_grid);


	// Getting the start and end point of the path
	double resolution_relation = ConfigMgr.grid_resolution / ConfigMgr.map_resolution;
	int start_x = ConfigMgr.start_x / resolution_relation;
	int start_y = ConfigMgr.start_y / resolution_relation;
	int end_x = ConfigMgr.target_x / resolution_relation;
	int end_y = ConfigMgr.target_y / resolution_relation;
	cell start_point(start_x, start_y);
	cell end_point(end_x, end_y);

	// Running A_star algorithm on the thickened map
	PathPlanner pathPlanner(map._thickened_grid, start_point, end_point);

	vector<cell> a_star_result = pathPlanner.astar();

	// Printing the A star path
	cout << "THE RESULT IS: " << a_star_result.size() << endl;
	for (int i = 0; i < a_star_result.size(); ++i) {
		cout << a_star_result[i].x_Coordinate << " " << a_star_result[i].y_Coordinate << endl;
		(map._original_grid[a_star_result[i].y_Coordinate][a_star_result[i].x_Coordinate]).cell_color = '2';
		(map._thickened_grid[a_star_result[i].y_Coordinate][a_star_result[i].x_Coordinate]).cell_color = '2';
	}

	// Creating initial way points
	WaypointsManager wp(a_star_result, ConfigMgr.grid_resolution, ConfigMgr.map_resolution);
	wp.build_way_point_vector(3);

	wayPoint wpm;
	set<wayPoint>::iterator it;

	// Printing the way points
	for (it = (wp.wayPoints).begin(); it != (wp.wayPoints).end(); ++it) {
		wpm = *it;
		cout << wpm.x_Coordinate << " " << wpm.y_Coordinate << " " << wpm.yaw << endl;
		(map._original_grid[wpm.y_Coordinate][wpm.x_Coordinate]).cell_color = '3';
		(map._thickened_grid[wpm.y_Coordinate][wpm.x_Coordinate]).cell_color = '3';
	}

	// Printing the map including path and way points
	for (int var = 0; var < 10; var++) {
		cout << endl << "";
	}
	cout << "Our thick map with path and waypoints" << endl;
	printMatrix(map._thickened_grid);
	for (int var = 0; var < 10; var++) {
		cout << endl << "";
	}

	ObstacleAvoidPlan plnOA(&robot, &wp);
	LocalizationManager lm;

	// Start moving in the path
	Manager manager(&robot, &plnOA, &lm, &ConfigMgr, &wp);
	manager.run();

 	return 0;
}

void printMatrix(vector<vector<grid_data> > grid){
	for (int var = 0; var < grid.size(); ++var) {
		for (int var2 = 0; var2 < grid[var].size(); ++var2) {
			cout << (int)grid[var][var2].cell_color;
			cout << " ";
		}
		cout << endl;
	}
}
