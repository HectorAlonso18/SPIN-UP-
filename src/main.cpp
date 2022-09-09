#include "main.h"
#include <vector>




void initialize() {
	Robot::reset_sensors();
	pros::delay(3000);
}

void disabled() {}

void competition_initialize() {}





std::vector<double> X_python{-89.530298, -67.30424500000001, -45.078192, -21.286924000000003, 1.252172, -21.91301, -45.078192, -67.617288, -21.286924000000003}; 

std::vector<double> Y_python{66.05207300000001, 44.452106, 21.91301, -0.626086, 21.599967, 45.391235, 68.55641700000001, 90.78247, 0.0};

//Robot::controlador_chassis({0,24}, 8, 0.037, 60, 2.5);
//Robot::controlador_giro(90,1.56, .015, 15, 2);
//Robot::move_to({X_python[i],Y_python[i],0}, 8, 0.037, 60,1.56, .015, 15, 3.5);

void autonomous() {
	Robot::start_task("TRACKING",Robot::raestro);
	
/*
	for(auto i=0; i<X_python.size(); i++){
		Robot::move_to({X_python[i],Y_python[i],0}, 8, 0.037, 60,1.56, .015, 15, 10);
	}
*/
   
   //Robot::move_to({0,24,180}, 8, 0.037, 60,1.56, .015, 15,3);
   //Robot::move_to({24,24,0}, 8, 0.037, 60,1.56, .015, 15, 3);
   //Robot::move_to({24,0,270}, 8, 0.037, 60,1.56, .015, 15, 3);
   //Robot::move_to({0,0,0}, 8, 0.037, 60,1.56, .015, 15, 3);

  
   /*for(auto i=0; i<X_python.size(); i++){
	 Robot::move_facing_to({X_python[i], Y_python[i]}, Robot::High_GoalX, Robot::High_GoalY, 8, 0.037, 60,1.56, .015, 15, 3);
   }*/

   Robot::move_facing_to({-91.721599, 90.46942700000001}, Robot::High_GoalX, Robot::High_GoalY, 8, 0.037, 60,1.56, .015, 15, 3);
	 
   for(auto i=0; i<X_python.size(); i++){
	 Robot::move_facing_to({X_python[i], Y_python[i]}, Robot::High_GoalX, Robot::High_GoalY, 8, 0.037, 60,1.56, .015, 15, 3);
   }

   Robot::get_data();

	//Robot::move_to_pure_pursuit({{0,55},{55,55}}, {55,55}, 9);
}


void opcontrol() {
	//Robot::start_task("TRACKING",Robot::raestro);
	Robot::start_task("DRIVE", Robot::drive);

}
