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
	Robot::Odom_Movement(fuctPtr_move_to,1, {-1.22,8.5,318}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	pros::delay(1000);
	Robot::Odom_Movement(fuctPtr_move_to,1, {4.5,5.5,0}, Drive_Constant,Turn_Constant, 3, 0,0,0);	
	Robot::Odom_Movement(fuctPtr_move_to,1, {15,5.5,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);


    //El robot se acomoda para girar el roller
	Robot::Odom_Movement(fuctPtr_move_to,1, {-2.31,5.8,180}, Drive_Constant,Turn_Constant, 1,5, 0,0);
	pros::delay(500);
	Robot::eat(false);
	//Se acerca al roller
	Robot::Odom_Movement(fuctPtr_move_to, 1,{-4,3.5,180}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	//Gira Roller
	Robot::Move_Roller(-338, 180);
	

	//Robot realiza el primer disparo y mete los 3 discos

	pros::delay(500);
	Robot::Odom_Movement(fuctPtr_move_to,1, {6,17,178}, Drive_Constant,Turn_Constant, 3, 0,0,0);	

	pros::delay(10); 
    Robot::Flywheel_pid_shoot(2860, {.01535,.00000525,4.05}, 3,0);

 

	//Va hacia la escuadra
	Robot::eat(true);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{40,0,90}, Drive_Constant,Turn_Constant, 1, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{46,17,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{59,17,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{46,17,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{46,27.5,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{59,27.5,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{46,27.5,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{46,40,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{61,40,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{46,40,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	
	Robot::Odom_Movement(fuctPtr_move_to, 1,{43,41,152.5}, Drive_Constant,Turn_Constant, 3, 0,0,0);
    Robot::Odom_Movement(fuctPtr_move_to, 1,{43,42,152.5}, Drive_Constant,Turn_Constant, 3, 0,0,0);
    pros::delay(100);
	Robot::eat(false);
	pros::delay(100);
	Robot::Flywheel_pid_shoot(2100, {.01535,.00000525,4.05}, 3,1000);	





  


}

//184

void opcontrol() {

	Robot::start_task("DRIVE", Robot::drive);
    Robot::start_task("TRACKING", Robot::raestro);
    
   
}
