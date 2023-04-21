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
		Robot::start_task("PID", Robot::Flywheel_PID_a);
		    
	   // Robot::start_task("PRINTING", screen::odom_stats); 

		
		Robot::RPM=2300; 
        
		Robot::eat(200); 

		//Va por el tercer disco y encesta 
		Robot::Move_to_point(Control_move_to, 1, {-12,-5,270}, Drive_Constant, Turn_Constant, 3);
		Robot::Turning(Control_move_to, 90+20, 1, Turn_Constant, 2.5);  
	

		//Se realiza la primera tanda
	 
		pros::delay(750); 
		Robot::Shoot_normal(20);
		pros::delay(1500); 
		Robot::Shoot_normal(20); 
		pros::delay(1500); 
		Robot::Shoot_normal(20); 
		pros::delay(1500); 


		Robot::RPM=0; 
	    Robot::eat(200); 

		//Tira torre de tres en la linea 
		Robot::Move_to_point(Control_move_to, .5, {-6,8,360-40}, Drive_Constant, Turn_Constant, 1.75);
		Robot::Move_to_point(Control_move_to, .25, {-6-10,8+10,360-40}, Drive_Constant, Turn_Constant, 1);
		
		//Retrocede 
		Robot::Move_to_point(Control_move_to, 1, {-6-8,8+8,360-40}, Drive_Constant, Turn_Constant, 1);
        pros::delay(1000);

		Robot::eat(0); 
        
		//Se perfila para poder agarrar el tercer disco de la segunda tanda
		Robot::Move_to_point(Control_move_to, .75, {2,28,0}, Drive_Constant, Turn_Constant, 3.5);
        Robot::eat(200);

		Robot::RPM=2380;
		
		//Toma el tercer disco de la sgunda tanda 
		Robot::Move_to_point(Control_move_to, .75, {4,35,0}, Drive_Constant, Turn_Constant, 2);
		
		//Retrocede y se prepará para poder tirar
		Robot::Move_to_point(Control_move_to, .75, {0,25,96}, Drive_Constant, Turn_Constant, 3);

		
     
		//Realiza la segunda tanda de tiro 
		//Robot::Shoot_PID(1, "LONG", 100); 
		pros::delay(750); 
		Robot::Shoot_normal(20);
		pros::delay(1500);  
		Robot::Shoot_normal(20); 
		pros::delay(1500); 
		Robot::Shoot_normal(20); 
		pros::delay(1000); 
       
        //Cambio de roller
		Robot::eat(-70);
	    Robot::Move_to_point(Control_move_to, 1, {3,25,98}, Drive_Constant, Turn_Constant, 1);
        Robot::eat(-70); 
		Robot::Move_to_point(Control_move_to, 1, {0,28,90}, Drive_Constant, Turn_Constant, 1);
        
		    
        
		//Se prepara para ir por la tercera tanda 
		Robot::eat(200); 

        
		Robot::RPM=2080;

		//Toma el primer disco 
		Robot::Move_to_point(Control_move_to, .75, {-24.90,-6.02,220.9}, Drive_Constant, Turn_Constant,6);
		//Segundo disco
		Robot::Move_to_point(Control_move_to, .8, {-39.81,-23.12,226.80}, Drive_Constant, Turn_Constant,3);
		//Tercer disco
		Robot::Move_to_point(Control_move_to, .8, {-39.57,-14.12,311.42}, Drive_Constant, Turn_Constant,3);

		//Posision para tercer tiro 
		Robot::Move_to_point(Control_move_to, .8, {-41.92,-7.64,126.9-8}, Drive_Constant, Turn_Constant,3);
		
		//Tercera tanda
		pros::delay(750); 
		Robot::Shoot_normal(20);
		pros::delay(1500); 
		Robot::Shoot_normal(20); 
		pros::delay(1500); 
		Robot::Shoot_normal(20); 
		pros::delay(1500); 
        		
        
		Robot::RPM= 2150; 
		//EScuadra
		Robot::Move_to_point(Control_move_to, 1, {-40.97,-28.48,129.75}, Drive_Constant, Turn_Constant,6);
		Robot::Move_to_point(Control_move_to, 1, {-17.17,-29.19,129.77}, Drive_Constant, Turn_Constant,6);
		
      
		//Cuarta tanda
		Robot::Move_to_point(Control_move_to, 1, {-12,-5,270}, Drive_Constant, Turn_Constant, 3);
		Robot::Turning(Control_move_to, 90+20, 1, Turn_Constant, 2.5);  
		Robot::eat(0);
       
	   /*
		//Robot::Shoot_PID(1, "LONG", 100); 
		pros::delay(2000); 
		Robot::Shoot_normal(20);
		pros::delay(1500); 
		Robot::Shoot_normal(20); 
		pros::delay(1500); 
		Robot::Shoot_normal(20); 
		pros::delay(1500); 
	*/ 

	  


  } 

	

}



void opcontrol() {
   
	Robot::start_task("get_orientation", Robot::track_orientation);
	Robot::start_task("Drive", Robot::drive);
	Robot::start_task("PID_FLYWHEEL", Robot::Flywheel_PID_a); 
	Robot::start_task("drifting", Robot::PID_drift_Cesar); 	
}