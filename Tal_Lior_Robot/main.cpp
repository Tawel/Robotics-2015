/*
 * main.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: user
 */

#include <iostream>
#include "Robot.h"
#include "Manager.h"
#include "Plans/PlnObstacleAvoid.h"
#include "Map.h"
#include "PathPlanner.h"
#include "ConfigurationManager.h"
#include <vector>
#include "WaypointsManager.h"

using namespace std;

void printMatrix(vector<vector<grid_data> > grid);

int main()
{
	ConfigurationManager cm(CONFIGURATION_PATH);

	Map map;
	map.thickenMap(cm.map_path, cm.robot_width);
	map.createGrids(cm.map_path, cm.map_resolution, cm.grid_resolution);

	cout << map._original_grid.size() <<  "Grid Row" << endl;

	Robot robot("localhost",6665, &cm, map._original_grid.size());
	//Robot robot("10.10.245.64",6665, &cm, map._original_grid.size());

	// printing
	printMatrix(map._original_grid);
	for (int var = 0; var < 10; var++) {
		cout << endl << "";
	}
	printMatrix(map._thickened_grid);

	// Running the a* algorithm on the thickened map
	// calculating robot location on grid...
	double resolution_relation = cm.grid_resolution / cm.map_resolution;
	// Location is ROUNDED!!!
	int start_x = cm.start_x / resolution_relation;
	int start_y = cm.start_y / resolution_relation;
	int end_x = cm.target_x / resolution_relation;
	int end_y = cm.target_y / resolution_relation;
	cell start_point(start_x, start_y);
	cell end_point(end_x, end_y);
	PathPlanner pathPlanner(map._thickened_grid, start_point, end_point);

	vector<cell> ass_star_result = pathPlanner.astar();

	cout << "THE RESULT IS: " << ass_star_result.size() << endl;

	for (int i = 0; i < ass_star_result.size(); ++i) {
		cout << ass_star_result[i].x_Coordinate << " " << ass_star_result[i].y_Coordinate << endl;
		(map._original_grid[ass_star_result[i].y_Coordinate][ass_star_result[i].x_Coordinate]).cell_color = 'k';
		(map._thickened_grid[ass_star_result[i].y_Coordinate][ass_star_result[i].x_Coordinate]).cell_color = 'k';
	}

	for (int var = 0; var < 10; var++) {
		cout << endl << "";
	}
	cout << "Tawels path planner" << endl;
	printMatrix(map._thickened_grid);
	for (int var = 0; var < 10; var++) {
		cout << endl << "";
	}


	WaypointsManager wp(ass_star_result, cm.grid_resolution, cm.map_resolution);
	wp.build_way_point_vector(3);

	wayPoint wpm;
	set<wayPoint>::iterator it;

	for (it = (wp.wayPoints).begin(); it != (wp.wayPoints).end(); ++it) {
		wpm = *it;
		cout << wpm.x_Coordinate << " " << wpm.y_Coordinate << " " << wpm.yaw << endl;
		(map._original_grid[wpm.y_Coordinate][wpm.x_Coordinate]).cell_color = 'l';
		(map._thickened_grid[wpm.y_Coordinate][wpm.x_Coordinate]).cell_color = 'l';
	}

	for (int var = 0; var < 10; var++) {
		cout << endl << "";
	}
	cout << "Tawels way points" << endl;
	printMatrix(map._thickened_grid);
	for (int var = 0; var < 10; var++) {
		cout << endl << "";
	}

	PlnObstacleAvoid plnOA(&robot, &wp);
	LocalizationManager lm;

	Manager manager(&robot, &plnOA, &lm, &cm, &wp);
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
