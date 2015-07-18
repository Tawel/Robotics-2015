
#include "Robot.h"
#include <math.h>

Robot::Robot(char* ip, int port, ConfigurationManager* cm, int grid_rows) {

	_pc = new PlayerClient(ip, port);
	_pp = new Position2dProxy(_pc);
	_lp = new LaserProxy(_pc);
	_cm = cm;
	_grid_rows = grid_rows;

	_pp->SetMotorEnable(true);
	//For fixing Player's reading BUG
	for(int i=0;i<15;i++)
		Read();
	//_pp->SetOdometry(cm->grid_resolution * ((double)cm->start_x / 100), cm->grid_resolution * ((double)cm->start_y / 100), cm->yaw * M_PI / 180 );
	//_pp->SetOdometry(((double)cm->start_x / 100) / (cm->grid_resolution / cm->map_resolution) , ((double)cm->start_y / 100) / (cm->grid_resolution / cm->map_resolution), 90*M_PI/180); //cm->yaw * M_PI / 180 );

	// (_cm->grid_resolution)
	//_pp->SetOdometry(((double)cm->start_x / 100), 0, 90*M_PI/180);
	//final
	//_pp->SetOdometry(((double)cm->start_x / _cm->map_resolution), ((double)cm->start_y / _cm->map_resolution), cm->yaw*M_PI/180);
	_pp->SetOdometry(((double)cm->start_x / (_cm->grid_resolution / _cm->map_resolution)/ (_cm->grid_resolution)), ((_grid_rows / _cm->grid_resolution) - (((double)cm->start_y) / (_cm->grid_resolution / _cm->map_resolution)/ (_cm->grid_resolution))) ,cm->yaw*M_PI/180);
	cout << " x " << getXpos() << " y " << getYpos() << "yaw" << getYaw() << endl;
//	for(int i=0;i<100;i++)
//			getXpos();
//			getYpos();
	_pp->SetOdometry(((double)cm->start_x / (_cm->grid_resolution / _cm->map_resolution)/ (_cm->grid_resolution)), ((_grid_rows / _cm->grid_resolution) - (((double)cm->start_y) / (_cm->grid_resolution / _cm->map_resolution)/ (_cm->grid_resolution))) ,cm->yaw*M_PI/180);
	cout << " x " << getXpos() << " y " << getYpos() << "yaw" << getYaw() << endl;
	//_pp->SetOdometry(((double)cm->start_x / _cm->map_resolution), ((double)cm->start_y / _cm->map_resolution), 90*M_PI/180);
			//cm->yaw * M_PI / 180 );





	//((((_grid_rows / _cm->grid_resolution)) - ((double)cm->start_y) / (_cm->grid_resolution / _cm->map_resolution)/ (_cm->grid_resolution)))

}

void Robot::Read()
{
	_pc->Read();
}

void Robot::setSpeed(float xSpeed, float angularSpeed) {
	_pp->SetSpeed(xSpeed, angularSpeed);
}

bool Robot::isRightFree() {
	if ((*_lp)[RIGHT_LASER_PROXY_VALUE] > DISTANCE_TOLERANCE)
		return true;
	else
		return false;
}

bool Robot::isLeftFree() {
	if ((*_lp)[LEFT_LASER_PROXY_VALUE] > DISTANCE_TOLERANCE)
		return true;
	else
		return false;
}

bool Robot::isForwardFree() {
	if ((*_lp)[FORWARD_LASER_PROXY_VALUE] > DISTANCE_TOLERANCE)
		return true;
	else
		return false;
}

double Robot::getXpos()
{
	return ((_pp->GetXPos()) * _cm->grid_resolution);
	//return ((_pp->GetXPos()) / _cm->map_resolution);
}

double Robot::getYpos()
{
	return (((_grid_rows / _cm->grid_resolution) - _pp->GetYPos()) * _cm->grid_resolution);
//	return ((_grid_rows / _cm->grid_resolution -_pp->GetYPos()) * _cm->grid_resolution);
	//return ((_grid_rows / _cm->grid_resolution -_pp->GetYPos()) / _cm->map_resolution);
}

double Robot::getYaw()
{
	double yaw = 180*(_pp->GetYaw())/M_PI;
	if(yaw >= 0)
	{
		return (yaw);
	}
	else
	{
		return (360+yaw);
	}
}

LaserProxy* Robot::getLaser()
{
	return (_lp);
}

float Robot::getLaserDistance(int index)
{
    return _lp->GetRange(index);
}

bool Robot::checkRange(int nStart, int nEnd)
{
    bool is_dis_Good = true;

    for (int index = nStart; (index <= nEnd) && (is_dis_Good); index++)
    {
    	is_dis_Good = (this->getLaserDistance(index) > DISTANCE_TOLERANCE);
    }

    return (is_dis_Good);
}


double Robot::getLaserSpec()
{
	return(((_lp->GetMaxAngle() * 180 / M_PI) + 120 ) / 0.36);
}

void Robot::drive(int cmToMove)
{
	_pc->Read();

	double radYaw = _pp->GetYaw();
	double locationX = _pp->GetXPos();
	double locationY = _pp->GetYPos();

	locationX += (cos(radYaw) * cmToMove);
	locationY += (sin(radYaw) * cmToMove);

	double currX = getXpos();
	double currY = getYpos();

	while(currX < locationX)
	{
		_pc->Read();
		_pp->SetSpeed(0.2, 0.0);
		currX = getXpos();
		currY = getYpos();
		cout << currX << endl;
	}

		_pp->SetSpeed(0.0,0.0);

}

Robot::~Robot() {
	delete _pc;
	delete _pp;
	delete _lp;
}
