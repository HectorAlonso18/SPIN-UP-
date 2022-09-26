#include "main.h"
#include <vector>

void initialize() {
	Robot::reset_sensors();
	pros::delay(3000);
	std::cout<<"oki doki"<<std::endl;
}

void disabled() {}

void competition_initialize() {}


void autonomous() {
	Robot::start_task("TRACKING",Robot::raestro);

	Robot::python_movement(Control_move_facing_to, X_python, Y_python, 4);

	//Robot::Odom_Movement(Control_move_to, std::vector<double> posicion, std::vector<double> DrivePID, std::vector<double> TurnPID, double tiempo, double TargetX, double TargetY)

	//Robot::Odom_Movement(Control_move_facing_to)

}

void opcontrol() {
	Robot::start_task("TRACKING",Robot::raestro);
	Robot::start_task("DRIVE", Robot::drive);
	Robot::start_task("PID_DRIFTING", Robot::PID_drift);


}
