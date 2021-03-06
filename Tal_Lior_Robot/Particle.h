/*
 * Particle.h
 *
 *  Created on: Jun 17, 2015
 *      Author: colman
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_
#include "Map.h"
#include "Robot.h"

#define NORMALIZE_FACTOR 1.2
#define TOP_DISTANCE 0.2
#define TOP_DELTA_TETA 0.01
#define TRH_DIS 2
#define TRH_YAW 2
class Particle {

private:
    // Variables
	cell _cell;
    double _Teta;
    double _Belief;


public:
    Particle(cell cell, double dYaw);

    // Properties
    double getBelief();

    // Methods
    double update(double deltaX, double deltaY, double deltaTeta , LaserProxy* laser);
    double probMov(double deltaX, double deltaY, double deltaTeta);
	virtual ~Particle();
};

#endif /* PARTICLE_H_ */
