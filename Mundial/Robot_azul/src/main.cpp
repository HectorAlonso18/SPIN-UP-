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


void skills_routine(){
	Robot::start_task("gps",Robot::rastreo);
	Robot::start_task("flywheeel", Robot::Flywheel_PID_a);
    
	Robot::eat_voltaje(12000);
	//toma el primer disco
	Robot::Move_to_point(Control_move_to, 1, {-24,-3,270},Drive_Constant, Turn_Constant, 2.5,0,0,false);
	Robot::Move_to_point(Control_move_to, 1, {-14,-3,270},Drive_Constant, Turn_Constant, 2.5,0,0,false);
 

	//Se acerca al roller y lo gira
	Robot::Move_to_point(Control_move_to, 1, {-42.5,-11,-90},Drive_Constant, Turn_Constant, 2,0,0,false);
    Robot::eat_voltaje(0);
	Robot::Move_to_point(Control_move_to, 1, {-46.5,-11,-90},Drive_Constant, Turn_Constant,1.53,0,0,false);
	Robot::Move_Roller_pos(300,200);
	Robot::Move_to_point(Control_move_to, 2, {-42.5,-11,-90},Drive_Constant, Turn_Constant, 0.75,0,0,false);
    
	//Se perfila para el segundo roller y lo gira
	Robot::Move_to_point(Control_move_to, 1, {-30,0,0},Drive_Constant, Turn_Constant, 1,0,0,false);
	Robot::Move_to_point(Control_move_to, 1, {-30,3.5,0},Drive_Constant, Turn_Constant, 1,0,0,false);

	Robot::Move_Roller_pos(300,200);
	Robot::RPM=1650; 
    Robot::eat_voltaje(12000); 

	//Se prepara para poder tirar
	Robot::Move_to_point(Control_move_to, 1, {30,-1,270-5},Drive_Constant, Turn_Constant, 2.5,0,0,false);

	Robot::Move_to_point(Control_move_to, 1, {30,-1,270-5},Drive_Constant, Turn_Constant, 1,0,0,false);
	Robot::eat_voltaje(0);
	pros::delay(1500); 
    
	//Tanda de tiros
	Robot::Shoot_normal(80); 
	pros::delay(1000);
	Robot::Shoot_normal(80); 
	pros::delay(1000);
	Robot::Shoot_normal(80); 


	Robot::RPM=0; 
    pros::delay(2000); 
    
	//Sección del segundo disparo
	//Va la escuadra y come los 3 discos 
	Robot::eat_voltaje(12000); 
	Robot::Turning(Control_move_to, 90+45, 1, Turn_Constant, 1.5, 0,0,0);

	Robot::Move_to_point(Control_move_to, 1, {31.5,-4,90+45},Drive_Constant, Turn_Constant, 1.5,0,0,false);
    Robot::Move_to_point(Control_move_to, 1, {31.5,-28-14,90+45},Drive_Constant, Turn_Constant, 4,0,0,false);
	pros::delay(1000); 
	Robot::eat_voltaje(0); 

	Robot::RPM=1750; //Enciende el flywheel para poder hacer la tanda

	//Se despega de la escuadra para evitar que se llegue atorar
	Robot::Move_to_point(Control_move_to, 1, {31.5-5,-28-14-5,90+45},Drive_Constant, Turn_Constant, 1.5,0,0,false);
	Robot::Move_to_point(Control_move_to, 1, {31.5,-28-14,90+45},Drive_Constant, Turn_Constant, 1.5,0,0,false);
	
	pros::delay(250);
	//Apunta la canasta
    Robot::Turning(Control_move_to, 180+23, 1, Turn_Constant, 5, 0,0,0);
	pros::delay(2000); 

  
	//Tanda de tiros
	Robot::Shoot_normal(80); 
	pros::delay(1000);
	Robot::Shoot_normal(80); 
	pros::delay(1000);
	Robot::Shoot_normal(80); 
	pros::delay(1000);

    
	Robot::RPM =0 ; 
	Robot::eat_voltaje(12000); 
	//va hacia el roller 
	Robot::Move_to_point(Control_move_to, 1, {72,-102,90},Drive_Constant, Turn_Constant, 8,0,0,false);
	Robot::eat_voltaje(0);

	//Se pega al roller y lo gira  el segundo roller 
	Robot::Move_to_point(Control_move_to, 1, {77.5,-102,90},Drive_Constant, Turn_Constant, 3,0,0,false);
    pros::delay(250);
	Robot::Move_Roller_pos(300,200);
	pros::delay(250); 

	//Se despega del roller 
	Robot::Move_to_point(Control_move_to, 1, {57,-102,90},Drive_Constant, Turn_Constant, 2.5,0,0,false);
	
	//Se acomoda para poder girar el otro roller cuarto roller
	Robot::Move_to_point(Control_move_to, 1, {57,-123,180},Drive_Constant, Turn_Constant, 2.5,0,0,false);
	Robot::Move_Roller_pos(300,200);
	pros::delay(250);

	//Se posiciona para expansión 
	Robot::Move_to_point(Control_move_to, 1, {65,-112,360-45},Drive_Constant, Turn_Constant, 2.5,0,0,false);

    Robot::Turning(Control_move_to, 315-180, 1, Turn_Constant, 2.5, 0,0,0);

	pros::delay(500);  
	Robot::Release_expansion(true);
}

void auton_routine(){
 Robot::start_task("gps",Robot::rastreo);

	Robot::start_task("flywheeel", Robot::Flywheel_PID_a);
    
	///2350 pego en el techo le bajaré 50 rpm
	//2280
    Robot::RPM=2260; //Revoluciones para el primer tiro  

   
	//Apunta a la canasta pra realizar el primer tiro 

	//anterior -14.5 y le siguio  faltantando, muy hacia la izquierda
	//13 jalo muy bien
	Robot::Turning(Control_move_to, 180-13, .5, Turn_Constant, 2, 0,0,0);
    
	//Tanda de primer tiro 
	pros::delay(2000); 
	Robot::Shoot_PID(1, "LONG", 1000); 
	pros::delay(2000);
	Robot::Turning(Control_move_to, 180-13, .5, Turn_Constant, 2, 0,0,0);
     
	
	pros::delay(1500);
	Robot::Shoot_PID(1, "LONG", 1000); 
	pros::delay(1000);
    
	//Se perfila para poder girar el roller
	Robot::Turning(Control_move_to, 180, 1, Turn_Constant, 2, 0,0,0);
    

	//Se acerca al roller para poder girarlo
	/// y anterior -> -1
	Robot::Move_to_point(Control_move_to, .75, {0,-8.5,180},Drive_Constant, Turn_Constant, 1,0,0,false);
    pros::delay(100);

	Robot::RPM=0; 
	//Gira el roller
	Robot::Move_Roller_pos(-170, 200);
    pros::delay(10);
    
	
	//Se despega del roller 
	Robot::Move_to_point(Control_move_to, 1, {0,8,180},Drive_Constant, Turn_Constant, 3,0,0,false);
    

	//Se acerca a la torre de 3 discos
    Robot::Move_to_point(Control_move_to, 1, {30,1,0},Drive_Constant, Turn_Constant, 3.5,0,0,false);
    pros::delay(10); 
    

	//Golpe a la    
	//Potencia anterior .6
	Robot::Move_to_point(Control_move_to, .5, {30,25,0},Drive_Constant, Turn_Constant, 3.5,0,0,false);
	//Come los discos 
	Robot::eat_voltaje(12000); 
	///anteriormente la y era 45 para poder comer los tres discos, ahora solo es para el primero
	Robot::Move_to_point(Control_move_to, .05, {30,26,0},Drive_Constant, Turn_Constant, 8,0,0,false);
	
	Robot::eat(0); 
	pros::delay(1000);

    //Velocidad del segundo tiro 
	Robot::RPM=2150; 
    
	//Posicion para realizar el segundo tiro
	Robot::Move_to_point(Control_move_to, 1, {20,10,0},Drive_Constant, Turn_Constant, 3,0,0,false);
	//Perfilación para el segundo tiro
	//anterior -27
	Robot::Turning(Control_move_to, 180-17-15, .5, Turn_Constant, 1.5, 0,0,0);
   

	pros::delay(500);
	Robot::eat_voltaje(0);


	//Serie de tiros a canasta
	Robot::Shoot_PID(1, "SHORT", 1000);  
	pros::delay(1000); 
	Robot::Shoot_PID(1, "SHORT", 1000);
	pros::delay(1000); 
	Robot::Shoot_PID(1, "SHORT", 1000);   
}


void autonomous() {	
	  
	
}



void opcontrol() {
	
	Robot::start_task("get_orientation", Robot::track_orientation);
	Robot::start_task("Drive", Robot::drive);
	Robot::start_task("PID_DRIFTING", Robot::Flywheel_PID_a); 
	
	//Robot::start_task("drifting", Robot::PID_drift_Cesar); 	
}