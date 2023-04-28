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

    int cesar_constant=1; 




	if(screen::autonomo==1){
		
		Robot::start_task("GPS", Robot::rastreo);
		Robot::start_task("PRINTING", screen::odom_stats);
		Robot::start_task("PID_FLYWHEEL", Robot::Flywheel_PID_motor); 
        
		
		Robot::eat(200); 
		Robot::RPM=430*cesar_constant; 

		//Agarra disco en la linea

		Robot::Move_to_point(Control_move_to, 1, {-2.1,7.59,319.40}, Drive_Constant, Turn_Constant, 2); 
		
		pros::delay(1000); 
		Robot::eat(0); 
       
	   
		//Gira el roller
		Robot::Move_to_point(Control_move_to, 1, {-2.77,.81-1.25,180.7}, Drive_Constant, Turn_Constant, 3); 
		Robot::Move_Roller_pos(-150, 100); 
        pros::delay(1000); 

		Robot::Move_to_point(Control_move_to, 1, {-2.77,5,180.7}, Drive_Constant, Turn_Constant, 3); 
        

		Robot::eat(200); 
        
		//Tumba torre de tres y tira 
		Robot::Move_to_point(Control_move_to, .6, {8.1,23,175.5}, Drive_Constant, Turn_Constant, 6.5); 
        
		//Primer tiro 
		pros::delay(3000); //2300 
		Robot::Shoot_normal(15);
		pros::delay(2500); //2000  
		Robot::Shoot_normal(15);
		pros::delay(2500);  //2000
		Robot::Shoot_normal(15);
        

		
		pros::delay(800);

		Robot::eat(200); 
        
		//Tumba torre de tres de nuestro lado 
		Robot::RPM= 425; 
		Robot::Move_to_point(Control_move_to, 1, {24.83,18.86,42.45}, Drive_Constant, Turn_Constant, 3); 
		Robot::Move_to_point(Control_move_to, .085, {39.42,36,37.8}, Drive_Constant, Turn_Constant, 8); 
		
		//Posicion para realizar la segunda tanda 
		Robot::Move_to_point(Control_move_to, .8, {36.09,45.44,154.25}, Drive_Constant, Turn_Constant, 6); 
		
			//Segundo tiro 
		pros::delay(3000); 
		Robot::Shoot_normal(20);
		pros::delay(2500);  
		Robot::Shoot_normal(20);
		pros::delay(2500);  
		Robot::Shoot_normal(20);

		pros::delay(750);
        
		//Escuadra
		Robot::Move_to_point(Control_move_to, .75, {54.22-2,4.87,37.22}, Drive_Constant, Turn_Constant, 6);
		
		Robot::Move_to_point(Control_move_to, .8, {58,4.87,37.22}, Drive_Constant, Turn_Constant, 5);  
		Robot::Move_to_point(Control_move_to, .075, {60,35.4+5,43.8}, Drive_Constant, Turn_Constant, 9); 

		//Posicion tercera tanda
		Robot::Move_to_point(Control_move_to, .8, {36.09,45.44,154.25}, Drive_Constant, Turn_Constant, 6); 
				
		//Tercer tiro 
		pros::delay(3000); 
		Robot::Shoot_normal(20);
		pros::delay(2500);  
		Robot::Shoot_normal(20);
		pros::delay(2500);  
		Robot::Shoot_normal(20);

        

		

		
		  
    } 

	

}



void opcontrol() {
    //Robot::start_task("PRINTING", screen::odom_stats);
	//Robot::start_task("GPS", Robot::rastreo);



	Robot::start_task("get_orientation", Robot::track_orientation);
	Robot::start_task("Drive", Robot::drive);
	Robot::start_task("PID_FLYWHEEL", Robot::Flywheel_PID_motor); 
	Robot::start_task("drifting", Robot::PID_drift_Alex_double); 	







}