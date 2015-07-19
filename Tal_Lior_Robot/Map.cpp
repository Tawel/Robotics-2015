/*
 * Map.cpp
 *
 *  Created on: Jun 2, 2015
 *      Author: colman
 */

#include "Map.h"
#include "lodepng.h"
#include <iostream>
#include <string>

//Encode from raw pixels to disk with a single function call
//The image argument has width * height RGBA pixels or width * height * 4 bytes
void Map::encodeOneStep(string filename, std::vector<unsigned char> image, unsigned width, unsigned height) {
	//Encode the image
	unsigned error = lodepng::encode(filename, image, width, height);

	//if there's an error, display it
	if (error)
		std::cout << "encoder error " << error << ": "
		<< lodepng_error_text(error) << std::endl;
}

void Map::decodeOneStep(string filename) {
	std::vector<unsigned char> image; //the raw pixels
	unsigned width, height;

	//decode
	unsigned error = lodepng::decode(image, width, height, filename);

	//if there's an error, display it
	if (error)
		std::cout << "decoder error " << error << ": "
		<< lodepng_error_text(error) << std::endl;

	// image displayed as vector (RGBA..)
}
void Map::thickenMap(string filename, int thickenSizeCM) {

	std::vector<unsigned char> image; //the raw pixels
	unsigned width, height;
	int x, y;
	int i, j;
	//decode
	unsigned error = lodepng::decode(image, width, height, filename);

	//if there's an error, display it
	if (error)
		std::cout << "decoder error " << error << ": "
		<< lodepng_error_text(error) << std::endl;

	std::vector<unsigned char> newImage; //the raw pixels

	newImage.resize(width * height * 4);

	// Initializing thick map
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++) {
			newImage[y * width * 4 + x * 4 + 0] = 255;
			newImage[y * width * 4 + x * 4 + 1] = 255;
			newImage[y * width * 4 + x * 4 + 2] = 255;
			newImage[y * width * 4 + x * 4 + 3] = 255;
		}

	// thickening map
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++) {
			if (!(image[y * width * 4 + x * 4 + 0]
			            || image[y * width * 4 + x * 4 + 1]
			                     || image[y * width * 4 + x * 4 + 2])){
				for (i = y - thickenSizeCM; i <= y + thickenSizeCM; i++){
					for (j = x - thickenSizeCM; j <= x + thickenSizeCM; j++){
						if ((i>=0)&&(j>=0)&&(i<height)&&(j<width)){
							newImage[i * width * 4 + j * 4 + 0] = 0;
							newImage[i * width * 4 + j * 4 + 1] = 0;
							newImage[i * width * 4 + j * 4 + 2] = 0;
						}
					}
				}
			}
		}

	// saving pic
	encodeOneStep(THICKENED_MAP_NAME, newImage, width, height);
}

vector<vector<grid_data> > Map::convertMapToGrid(string filename, double map_resolution, double grid_resolution){

	std::vector<unsigned char> image;
	unsigned width, height;
	unsigned error = lodepng::decode(image, width, height, filename);
	int x,y, i, j;
	int resolution_relation = grid_resolution / map_resolution;
	int grid_rows = height * map_resolution / grid_resolution;
	int grid_columns = width * map_resolution / grid_resolution;
	char is_black_found;

	vector<vector<grid_data> > grid (grid_rows);

	for (int c = 0; c < grid_rows; c++)
	{
		grid[c].resize(grid_columns);
	}

	for (y = 0; y < (grid_rows * resolution_relation); y += resolution_relation)
		for (x = 0; x < (grid_columns * resolution_relation); x += resolution_relation) {
			is_black_found = 0;
			for (i = y ; (i < y + resolution_relation)&&(is_black_found == 0); i++){
				for (j = x ; (j < x + resolution_relation)&&(is_black_found == 0); j++){
					if (!(image[i * width * 4 + j * 4 + 0]
					            || image[i * width * 4 + j * 4 + 1]
					                     || image[i * width * 4 + j * 4 + 2])){
						is_black_found = 1;
						(grid[y / resolution_relation][x / resolution_relation]).cell_color = 1;
					}
				}
			}
			if (is_black_found == 0){
				(grid[y / resolution_relation][x / resolution_relation]).cell_color = 0;
			}

			(grid[y / resolution_relation][x / resolution_relation]).f_val = 0;
			(grid[y / resolution_relation][x / resolution_relation]).g_val = 0;
			(grid[y / resolution_relation][x / resolution_relation]).h_val = 0;
			(grid[y / resolution_relation][x / resolution_relation]).parent.x_Coordinate = 0;
			(grid[y / resolution_relation][x / resolution_relation]).parent.y_Coordinate = 0;
		}

	//_original_grid = grid;
	return grid;
}

void Map::createGrids(string originalMapFile, double map_resolution, double grid_resolution){
	_original_grid = convertMapToGrid(originalMapFile,
			map_resolution,
			grid_resolution);
	_thickened_grid = convertMapToGrid(THICKENED_MAP_NAME,
			map_resolution,
			grid_resolution);
}


Map::~Map() {
	// TODO Auto-generated destructor stub
}


