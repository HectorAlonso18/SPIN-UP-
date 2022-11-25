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
    
		
	//Robot va por el primer disco y gira el roller. 
  
  
	Robot::eat(true);
	//El robot toma el primer disco
	//Robot::Odom_Movement(fuctPtr_move_to,1, {1.8,6.8,316}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	
	Robot::Odom_Movement(fuctPtr_move_to,1, {1.8,8,316}, Drive_Constant,Turn_Constant, 3, 0,0,0);
    pros::delay(500); 
    Robot::Odom_Movement(fuctPtr_move_to,1, {10,6,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	



	Robot::eat(false);
	//Se acerca al roller
	Robot::Odom_Movement(fuctPtr_move_to, 1,{-8.5,0.5,180}, Drive_Constant,Turn_Constant, 2, 0,0,0);
	//Gira Roller
	Robot::Move_Roller(338, 180);
    
	pros::delay(500); 
    
	Robot::Odom_Movement(fuctPtr_move_to, 1,{-8.5,6,180}, Drive_Constant,Turn_Constant, 1, 0,0,0);

	//5
	Robot::Odom_Movement(fuctPtr_move_to, 1,{8,8,157}, Drive_Constant,Turn_Constant, 3, 0,0,0);

	//Realiza tiro
	pros::delay(10); 
    Robot::Flywheel_pid_shoot(2890, {.01535,.00000525,4.05}, 3,125);
	pros::delay(100); 

  
    
	//IR A LA ESCUADRA 
////////////////////////////////////////////////////////////////////////////////////////////
	/*
	Robot::eat(true);
 

	Robot::Odom_Movement(fuctPtr_move_to, 1, {24,10,90}, Drive_Constant, Turn_Constant, 3, 0, 0, 0);

	//Agarra el primer disco 
	Robot::Odom_Movement(fuctPtr_move_to, 1, {40,16.5,90}, Drive_Constant, Turn_Constant, 3, 0, 0, 0);

    //Retrocede
	Robot::Odom_Movement(fuctPtr_move_to, 1, {28,17,90}, Drive_Constant, Turn_Constant, 3, 0, 0, 0);
 
	//Prepara para el segundo disco
    Robot::Odom_Movement(fuctPtr_move_to, 1, {28,30,90}, Drive_Constant, Turn_Constant, 3, 0, 0, 0);

	//Agarra el segundo disco
	Robot::Odom_Movement(fuctPtr_move_to, 1, {40,30,90}, Drive_Constant, Turn_Constant, 3, 0, 0, 0);

	//Va por el último disco
	Robot::Odom_Movement(fuctPtr_move_to, 1, {28,30,90}, Drive_Constant, Turn_Constant, 3, 0, 0, 0);

	Robot::Odom_Movement(fuctPtr_move_to, 1, {28,43,90}, Drive_Constant, Turn_Constant, 3, 0, 0, 0);

	Robot::Odom_Movement(fuctPtr_move_to, 1, {40,43,90}, Drive_Constant, Turn_Constant, 3, 0, 0, 0);

	Robot::Odom_Movement(fuctPtr_move_to, 1, {28,43,157}, Drive_Constant, Turn_Constant, 3, 0, 0, 0);

 	Robot::Flywheel_pid_shoot(2700, {.01535,.00000525,4.05}, 3,150);
*/
	

  


}

//184

void opcontrol() {

	Robot::start_task("DRIVE", Robot::drive);
    //Robot::start_task("TRACKING", Robot::raestro);
    
   
}
