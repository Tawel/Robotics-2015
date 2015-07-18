/*
 * Map.cpp
 *
 *  Created on: Jun 2, 2015
 *      Author: colman
 */

#include "Map.h"
#include "lodepng.h"
#include <iostream>


//Encode from raw pixels to disk with a single function call
//The image argument has width * height RGBA pixels or width * height * 4 bytes
void Map::encodeOneStep(const char* filename, std::vector<unsigned char> image, unsigned width, unsigned height) {
	//Encode the image
	unsigned error = lodepng::encode(filename, image, width, height);

	//if there's an error, display it
	if (error)
		std::cout << "encoder error " << error << ": "
				<< lodepng_error_text(error) << std::endl;
}

void Map::decodeOneStep(const char* filename) {
	std::vector<unsigned char> image; //the raw pixels
	unsigned width, height;

	//decode
	unsigned error = lodepng::decode(image, width, height, filename);

	//if there's an error, display it
	if (error)
		std::cout << "decoder error " << error << ": "
				<< lodepng_error_text(error) << std::endl;

	//the pixels are now in the vector "image", 4 bytes per pixel, ordered RGBARGBA..., use it as texture, draw it, ...
}
void Map::thickenMap(const char* filename, int thickenSizeCM) {

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
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++) {
			 newImage[y * width * 4 + x * 4 + 0] = 255;
			 newImage[y * width * 4 + x * 4 + 1] = 255;
			 newImage[y * width * 4 + x * 4 + 2] = 255;
			 newImage[y * width * 4 + x * 4 + 3] = 255;
		}

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
	encodeOneStep(THICKENED_MAP_NAME, newImage, width, height);
}

vector<vector<grid_data> > Map::convertMapToGrid(const char* filename, double map_resolution, double grid_resolution){

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

void Map::createGrids(const char* originalMapFile, double map_resolution, double grid_resolution){
	_original_grid = convertMapToGrid(originalMapFile,
										map_resolution,
										grid_resolution);
	_thickened_grid = convertMapToGrid(THICKENED_MAP_NAME,
										map_resolution,
										grid_resolution);
}

double Map::checkNearestObs(cell_coordinate point, double angle){

	int roundedAngle = angle;
	if (roundedAngle % 45 < 22.5) {
		roundedAngle = (roundedAngle/45)*45;
	} else {
		roundedAngle = (roundedAngle/45 + 1)*45;
	}

	int xDirection;
	int yDirection;

	switch (roundedAngle%360) {
		case 0:
			xDirection = 1;
			yDirection = 0;
			break;
		case 45:
			xDirection = 1;
			yDirection = 1;
			break;
		case 90:
			xDirection = 0;
			yDirection = 1;
			break;
		case 135:
			xDirection = -1;
			yDirection = 1;
			break;
		case 180:
			xDirection = -1;
			yDirection = 0;
			break;
		case 225:
			xDirection = -1;
			yDirection = -1;
			break;
		case 270:
			xDirection = 0;
			yDirection = -1;
			break;
		case 315:
			xDirection = 1;
			yDirection = -1;
			break;
		default:
			return -1; // wrong angle
	}

	int x = point.x_Coordinate;
	int y = point.y_Coordinate;
	int counter = 0;

	while (y > 0 && x > 0 && y < _original_grid.size() && x < _original_grid[0].size()){
		if (_original_grid[y][x].cell_color == 1) {
			break;
		}
		counter++;
		x += xDirection;
		y += yDirection;
	}

	if (xDirection == 0 || yDirection == 0) {
		return counter*10;
	} else {
		return counter*14.14215; // sqrt(200)
	}
}

//Map::Map() {
//	// TODO Auto-generated constructor stub
//
//}

Map::~Map() {
	// TODO Auto-generated destructor stub
}


