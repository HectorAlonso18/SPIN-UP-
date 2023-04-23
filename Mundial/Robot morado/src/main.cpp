#include "main.h"
#include "parametros.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <iostream>
#include <ostream>
#include <regex>
#include <vector>



void initialize() {
	Robot::reset_sensors();
	pros::delay(3000);
	std::cout<<"SENSORES OK"<<std::endl;

	screen::init();  
}





void disabled() {}


void competition_initialize() {}




void autonomous() {
	   
    //Robot::start_task("PRINTING", screen::odom_stats);
	//Robot::start_task("GPS", Robot::rastreo);

  




	if(screen::autonomo==1){
		
		Robot::start_task("GPS", Robot::rastreo);
		Robot::start_task("PRINTING", screen::odom_stats);
        Robot::eat(200); 

		//Agarra disco en la linea
		Robot::Move_to_point(Control_move_to, 1, {-1.99-1,7.76-3,313.25}, Drive_Constant, Turn_Constant, 2); 
		
		pros::delay(1000); 
		Robot::eat(0); 

		//Gira el roller
		Robot::Move_to_point(Control_move_to, 1, {-2.77,.81-1.25,180.7}, Drive_Constant, Turn_Constant, 3); 
		Robot::Move_Roller_pos(-150, 100); 
        pros::delay(1000); 
		Robot::Move_to_point(Control_move_to, 1, {-2.77,5,180.7}, Drive_Constant, Turn_Constant, 3.5); 
        

		//Posicion de tiro 
		//Robot::move_Flywheel(10000); 
		Robot::eat(200); 
		Robot::move_Flywheel(10000); 
	
		Robot::Move_to_point(Control_move_to, 1, {17.01,8.72+12,168}, Drive_Constant, Turn_Constant, 6); 
        
	
		//Primer tiro 
		pros::delay(3000); 
		Robot::Shoot_normal(20);
		pros::delay(4000);  
		Robot::Shoot_normal(20);
		pros::delay(4000);  
		Robot::Shoot_normal(20);
      
	  /*
		//Tira torre de tres 
		Robot::Move_to_point(Control_move_to, 1, {24.11,4.49,60.95}, Drive_Constant, Turn_Constant, 5); 
		//Se la come
		Robot::Move_to_point(Control_move_to, 1, {43.8,19.71,44.8}, Drive_Constant, Turn_Constant, 5); 
        
		//Posicioin de tiro 


        */
		
		  
    } 

	

}



void opcontrol() {



//Robot::RPM=400; 

//Robot::start_task("FLYWHEEL", Robot::Flywheel_PID_motor); 


	Robot::start_task("get_orientation", Robot::track_orientation);
	Robot::start_task("Drive", Robot::drive);
	Robot::start_task("PID_FLYWHEEL", Robot::Flywheel_PID_motor); 
	Robot::start_task("drifting", Robot::PID_drift_Alex_double); 	



}