#include "main.h"
#include "parametros.h"
#include "pros/rtos.hpp"
#include <iostream>
#include <ostream>
#include <vector>


void initialize() {
	Robot::reset_sensors();
	pros::delay(3000);
	std::cout<<"OKI DOKI"<<std::endl; 
}


void disabled() {}


void competition_initialize() {}


void autonomous() {
	Robot::start_task("TRACKING", Robot::raestro); 
	Robot::eat(true,10000); 
    
	//Robot avanza y pesca el primer disco
	Robot::PID_chassis(14,0, .5, 2); 
    pros::delay(500); 

	Robot::PID_chassis(11,0, .25, 2); 

	pros::delay(500); 

	//Se prepara a tirar

    Robot::Turning(fuctPtr_move_to,180,.5, Turn_Constant, 3, 0,0,0);	
    
    Robot::Turning(fuctPtr_move_to,207,1, Turn_Constant, 3, 0,0,0);	
    
	pros::delay(100);
	
	//Tira el primer disco
	Robot::Flywheel_pid_shoot(2780, Flywheel_Constant,1, 3,0);

	Robot::eat(false,0); 

	pros::delay(200); 
    
	//El robot se prepara para agarrar el segundo disco
	Robot::eat(true,10000); 
  
	Robot::Turning(fuctPtr_move_to,47,.5, Turn_Constant, 3, 0,0,0);	
    
    Robot::PID_chassis(8,47, .5, 2); 

    //El robot retrocede para ponerse con la diagonal y comerselos 
	Robot::PID_chassis(-15,47, .3, 2); 

    pros::delay(1250); 

	Robot::Turning(fuctPtr_move_to,200,.5, Turn_Constant, 3, 0,0,0);

	//Se perfila con la diagonal 320
	Robot::Turning(fuctPtr_move_to,315,.5, Turn_Constant, 3, 0,0,0);	
	pros::delay(200); 
    
	//Avanza y se come los dis discos 
	Robot::PID_chassis(80,320, .1, 8); 

	//Apunta para el segundo tiro
	Robot::Turning(fuctPtr_move_to,242,.5, Turn_Constant, 3, 0,0,0);
    Robot::Flywheel_pid_shoot(1800, Flywheel_Constant, .5,3,750);

}

//184

void opcontrol() {
	Robot::start_task("DRIVE", Robot::drive);

}
