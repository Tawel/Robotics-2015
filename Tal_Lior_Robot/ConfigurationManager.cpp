/*
 * ConfigurationManager.cpp
 *
 *  Created on: May 29, 2015
 *      Author: user
 */

#include "ConfigurationManager.h"
#include<iostream>
#include<fstream>
#include <stdlib.h>

using namespace std;
ConfigurationManager::ConfigurationManager(const char* configurationPath) {
	ReadConfigurationData(configurationPath);
	convertSizeToCM();

}


void ConfigurationManager::ReadConfigurationData(const char* configurationPath) {
	 ifstream myReadFile;
	 int counter = 0;
	 char attribute_name[MAX_LINE_LENGTH];
	 myReadFile.open(configurationPath);

	 // Reading all the configurations
	 while (!myReadFile.eof() && (counter < NUM_OF_PROPERTIES)) {

		 myReadFile.getline(attribute_name, MAX_LINE_LENGTH, ' ');
		 myReadFile.getline(this->file_data[counter], MAX_LINE_LENGTH, '\n');
		 counter++;
	}


	this->map_path = this->file_data[0];
	this->start_x = atoi(this->file_data[1]);
	this->start_y = atoi(this->file_data[2]);
	this->yaw = atoi(this->file_data[3]);
	this->target_x = atoi(this->file_data[4]);
	this->target_y = atoi(this->file_data[5]);
	this->robot_length = atof(this->file_data[6]);
	this->robot_width = atof(this->file_data[7]);
	this->map_resolution = atof(this->file_data[8]);
	this->grid_resolution = atof(this->file_data[9]);
}


void ConfigurationManager::convertSizeToCM(){
	this->robot_length = this->robot_length / this->map_resolution;
	this->robot_width = this->robot_width / this->map_resolution;
}

ConfigurationManager::~ConfigurationManager() {
	// TODO Auto-generated destructor stub
}

