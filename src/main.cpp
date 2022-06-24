#include "main.h"




void initialize() {
	Robot::reset_sensors();
	pros::delay(3000);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	Robot::start_task("TRACKING",Robot::raestro);

	//Robot::move_to(0 ,55 ,90 ,13 ,5.12 ,.5 ,1.5 ,5.12 ,.5 ,1 ,2);
	//Robot::move_to(55 ,55 ,180 ,8 ,5.12 ,.5 ,1.5 ,5.12 ,.5 ,1 ,2);
	//Robot::move_to(55 ,0 ,270 ,13 ,5.12 ,.5 ,1.5 ,5.12 ,.5 ,1 ,2);
	//Robot::move_to(0,0 ,360 ,13 ,5.12 ,.5 ,1.5 ,5.12 ,.5 ,1 ,2);
}

void opcontrol() {
	
	Robot::start_task("DRIVE",Robot::drive);	

	
}
