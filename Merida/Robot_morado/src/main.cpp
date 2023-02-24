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
	std::cout<<"OKI DOKI"<<std::endl; 
}


void disabled() {}


void competition_initialize() {}


void autonomous() {
    
	/* Pendientes: 
	-> Verificar si el brian puede abrir la terminal.
    -> Tunear de nuevo el PID del flywheel. 
	-> Probar las funciones de disparo. 
	-> Poner ligas en los encoders para tener mejor precision. 
	-> Verificar las task de PID_DRIFT y verificar que jalen. 
	*/


	//El robot va por el primer disco y lo come
	Robot::eat(200); 

	//Iniciamos task necesarias para hacer la rutina
    Robot::start_task("gps", Robot::rastreo);    
	
	//Robot::RPM=3350; 
	Robot::start_task("flywheel_pid", Robot::Flywheel_PID); 
    
    

    Robot::Odom_Movement(Control_move_to, .5, {-16,10,360-90}, Drive_Constant, Turn_Constant, 2, 0, 0);
	
	pros::delay(1500); 
    
	//Apunta a la canasta para poder empezar con la serie de 3 disparos
	//.33
	Robot::Turning(Control_move_to, 120,1, Turn_Constant, 1.25,0,0,0);
    

	pros::delay(1000); 
	
	/*
	//Primer tres tiros. 
	Robot::Shoot_PID(1, "LONG", 1000); 
    
	Robot::RPM=3240; 
	pros::delay(500); 
	Robot::Shoot_PID(1, "LONG", 1000); 
    Robot::RPM=3250;
	pros::delay(750); 
	Robot::Shoot_PID(1, "LONG", 1000); 
    */

	//Comentaré esto debido a que quiero probar las funciones básicas para poder tirar los primeros tres discos

    
	//El robot va por el disco que está en la linea 
	Robot::eat(200); 
    Robot::Odom_Movement(Control_move_to, 1, {-24.5,15.5,360-45}, Drive_Constant, Turn_Constant, 2, 0, 0);
    
	pros::delay(1000);
	//Se perfila para la diagonal para poder tomar los otros dos discos
	Robot::Odom_Movement(Control_move_to, 1, {-23,8,360-135}, Drive_Constant, Turn_Constant, 2, 0, 0); 
	
	//Come los dos discos
	Robot::Odom_Movement(Control_move_to, .3, {-23 - 19 -5 -7 -1 ,-17,360-135}, Drive_Constant, Turn_Constant, 2, 0, 0);
	Robot::Odom_Movement(Control_move_to, .3, {-23 - 19 -5 -7 -1 -3,-17,360-135}, Drive_Constant, Turn_Constant, 2, 0, 0);
	
	pros::delay(2000); 

	//Apunta a la canasta para poder empezar con la serie de 3 disparos
	Robot::Turning(Control_move_facing_to, 0,.33, Turn_Constant, 1.25, Robot::High_GoalX, Robot::High_GoalY,-180);

  	
	//Segunda tanda de tiros
    /*
	Robot::RPM=3345;
	pros::delay(500); 
	Robot::Shoot_PID(1, "LONG", 1000); 
	Robot::RPM=3243; 
	pros::delay(500); 
	Robot::Shoot_PID(1, "LONG", 1000); 
    Robot::RPM=3253;
	pros::delay(750); 
	Robot::Shoot_PID(1, "LONG", 1000); 
    */
    
	//Va hacía la escuadra y se prepara para poder tomar otros 3 discos
	Robot::Odom_Movement(Control_move_to, .4, {-28  ,-22,135}, Drive_Constant, Turn_Constant, 4, 0, 0);
    pros::delay(500); 
	Robot::Odom_Movement(Control_move_to, .2, {-5  ,-29,135}, Drive_Constant, Turn_Constant, 4, 0, 0);
    
	pros::delay(10);
   
	Robot::Odom_Movement(Control_move_to, .75, {-5  ,34,92}, Drive_Constant, Turn_Constant, 5, 0, 0);
    Robot::eat(0); 
	pros::delay(250);  
	Robot::Odom_Movement(Control_move_to, .75, {14  ,34,92}, Drive_Constant, Turn_Constant, 5, 0, 0);
    pros::delay(10); 
	Robot::Move_Roller_pos(500, 100); 

}



void opcontrol() {
	
	
	Robot::start_task("GET_ORIENTATION",Robot::track_orientation); 
	Robot::start_task("PID_DRIFT", Robot::PID_drift_Alex_double); 
	Robot::start_task("PID_FLYWHEEEL", Robot::Flywheel_PID); 
	Robot::start_task("drive", Robot::drive);
    
}