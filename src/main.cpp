#include "main.h"




void initialize() {
	Robot::reset_sensors();
	pros::delay(3000);
}

void disabled() {}

void competition_initialize() {}

/*
Robot::controlador_giro(-90, 1.25, 0, .4,3);
Robot::controlador_chassis({0,24}, 10, 5.12, .46, 2); 
*/

void autonomous() {
	Robot::start_task("TRACKING",Robot::raestro);
	Robot::test_odom();
	pros::delay(250);
	Robot::get_data();
    
	//Robot::move_to_pure_pursuit({{0,55},{55,55}}, {55,55}, 9);
}


void opcontrol() {
	//Robot::start_task("TRACKING",Robot::raestro);
	Robot::start_task("DRIVE", Robot::drive);
}
