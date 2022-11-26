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
    
	//Enciede el intaker
	Robot::eat(12000);
	pros::delay(200);

	//El robot toma el primer disco
	//Robot::Odom_Movement(fuctPtr_move_to,1, {1.8,6.8,316}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to,1, {-2.5,12,360-45}, Drive_Constant,Turn_Constant, 3, 0,0,0);
    pros::delay(250); 
	
	//Hace recorrido para asegurar el disco
	Robot::Odom_Movement(fuctPtr_move_to,0.25, {0,9,0}, Drive_Constant,Turn_Constant, 3, 0,0,0);
    Robot::Odom_Movement(fuctPtr_move_to,0.75, {12,9,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::eat(0);
	Robot::Move_Roller(2000, 200);
	//Se acerca al roller

	Robot::Odom_Movement(fuctPtr_move_to,1, {-6,8,180}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	pros::delay(500);
	

	Robot::Odom_Movement(fuctPtr_move_to,1, {-6,6,180}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	//Gira el roller
	Robot::Move_Roller(385, 180);
	pros::delay(250);

	//Se dirige a posición de tirar
	Robot::Odom_Movement(fuctPtr_move_to,1, {3,12,180-2}, Drive_Constant,Turn_Constant, 3, 0,0,0);
    //Dispara
	Robot::Flywheel_pid_shoot(2900,{.01535,.00000525,4.05},1, 3,175);

	//Va hacia la esquina
	Robot::Odom_Movement(fuctPtr_move_to,1, {35,12,90}, Drive_Constant,Turn_Constant, 2, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to,1, {50,22.5,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);

	//Empieza a comer 
	//Primer disco
	Robot::eat(12000);
	Robot::Odom_Movement(fuctPtr_move_to,1, {65,22.5,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to,1, {50,22.5,90}, Drive_Constant,Turn_Constant, 3, 0,0,0);


/*


	Robot::eat(0);

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
    Robot::Flywheel_pid_shoot(2890,{.01535,.00000525,4.05},1, 3,125);
	pros::delay(100); 

  */
	

}



void opcontrol() {
	Robot::start_task("DRIVE", Robot::drive);

}
