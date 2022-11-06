#include "main.h"
#include <vector>

void initialize() {
	//Reinicio de sensores antes de match
	Robot::reset_sensors();
	pros::delay(3000);
	std::cout<<"oki doki"<<std::endl;
}

void disabled() {}

void competition_initialize() {}


void autonomous() {
	Robot::start_task("TRACKING",Robot::raestro);
    
	Robot::Odom_Movement(Control_move_to, {0,24,0}, {8, .01, 100}, Turn_Constant, 5,0,0);
    Robot::get_data();


	//Robot::python_movement(Control_move_facing_to, X_python, Y_python, 4);

	

}

void opcontrol() {
	//Robot::start_task("TRACKING",Robot::raestro);
	Robot::start_task("DRIVE", Robot::drive);
	//Robot::start_task("PID_DRIFTING", Robot::PID_drift);

}
