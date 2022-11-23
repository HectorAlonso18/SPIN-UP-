#include "main.h"
#include "parametros.h"
#include "pros/rtos.hpp"
#include <iostream>
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
    
	//Recuerda que puedes dejar el cursor en cada funcion y te da una descripción de que hace y que parametros recibe 

	/*
	Robot::Odom_Movement(fuctPtr_move_to, {0,-6,0}, Drive_Constant, Turn_Constant, 3, 0, 0);
	pros::delay(5000);
    Robot::Odom_Movement(fuctPtr_move_to, {-14,-6,180}, Drive_Constant, Turn_Constant, 3, 0, 0);
    pros::delay(5000);
	Robot::Odom_Movement(fuctPtr_move_to, {-15,-23,180}, Drive_Constant, Turn_Constant, 3, 0, 0);
    pros::delay(5000);
	Robot::Odom_Movement(fuctPtr_move_to, {-16.7,-23,180}, Drive_Constant, Turn_Constant, 3, 0, 0);
	Robot::eat(true);
	pros::delay(5000);
	Robot::Odom_Movement(fuctPtr_move_to, {-16.7,-27.2,182}, Drive_Constant, Turn_Constant, 3, 0, 0);
	*/
    
	
    //Función para poder mover el flywheel con PID
	//Estas son las constantes dle pid, SE PUEDEN VER SUEJETAS A CAMBIO
	//Tienen numeros de disparos, se pueden disminuir o aumentar
	//El tiempo es cero
	
	
	//Robot::Flywheel_pid(2700, {.01535,.00000525,4.05}, 3);

	//Está función es para utilizar el intake y comer 
	//Robot::eat(true); /*y para pararlo se utiliza*/Robot::eat(false); 

	//Para mover el roller se utiliza la siguiente función
	//Robot::Move_Roller( grados del encoder, la velocidad en RPM)
	//Para cambiar el sentido, solo ponle un valor negativo a la distancia. 
    //Robot::Move_Roller(-350,100); 

	//Función para moverte a un punto y otro con PID Y con una orientación en especifico 
	//Drive_Constant y Turn_constant son las constantes ya definidas para este robot, pero puedes ingresar las tuyas
	//En forma de vector, {kp,ki,kd} ó modificarlas en la parte de parametros.cpp 

	//ES IMPORTANTE QUE EL OFFSET ESTE EN CERO, AL IGUAL QUE EL TARGETX Y EL TARGET Y

	//Robot::Odom_Movement(fuctPtr_move_to, {0,23,180}, Drive_Constant, Turn_Constant, 3, 0, 0,0);
	
	//Función para moverte a un punto a otro, pero apuntando a un objetivo mientras se mueve. 
    //En el apartado de la orientacion da igual cual valor se coloque
	//es importante colocar el valor del target x o target y, para que funciones debes de poner las coordenasdas respecto al origen del robot
	//El robot lo mirará de frente o sea estarán centrados, por lo tanto puedes utilizar el parametro offset, para que el robot quede desfasado y que el flywheel sea el que quede en fase
	//con el objeto. 

	//En el archivo Robot.cpp, existe unas variables que son las coordenadas de la high goal, se pueden modificar y en vez de colocar las coordenadas como parametro
	//solo escribes el nombre de las variables, dentro de está función 

	//Los valores del target y offset los puse al azar, no son los correctos : ) 
	//Robot::Odom_Movement(fuctPtr_facing_to, {0,23,0}, Drive_Constant, Turn_Constant, 3, 10, 30,10);
	

	//Esto mismo sucede con las funciones para poder girar

	//Girar normal
    
	//ES IMPORTANTE QUE EL OFFSET ESTE EN CERO, AL IGUAL QUE EL TARGETX Y EL TARGET Y
	//ES la misma logica que para las funciones de moverse
	//Robot::Turning(fuctPtr_move_to, 180, Turn_Constant, 3, 0, 0, 0);
	
	//Girar respecto a un objeto (sus coordenadas)
    
	//Los valores del target y offset los puse al azar, no son los correctos : ) 
	//Robot::Turning(fuctPtr_facing_to, 180, Turn_Constant, 3, 10, 30, 30);

	Robot::eat(true);
	Robot::Odom_Movement(fuctPtr_move_to,1, {-1.9,11,360-43}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	pros::delay(1000);
	//Robot::Odom_Movement(fuctPtr_move_to, .1,{-2.2,10.4,350-50}, Drive_Constant,Turn_Constant, 3, 0,0,0);

	Robot::Odom_Movement(fuctPtr_move_to,1, {-2.31,5.88,180}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	pros::delay(500);
	Robot::eat(false);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{-2.6,4.81,180}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Move_Roller(600, 150);
	pros::delay(500);
	Robot::Odom_Movement(fuctPtr_move_to,1, {6,17,180-1.8}, Drive_Constant,Turn_Constant, 3, 0,0,0);	
	Robot::Flywheel_pid(2850, {.01535,.00000525,4.05}, 3);
	//Aqui elrobot mete los primeros tres discos
	//Robot::Odom_Movement(fuctPtr_facing_to, {27,22,0}, Drive_Constant, Turn_Constant, 3, 28, 24,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{31,8,0}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to,1, {31,24,0}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::eat(true);
	Robot::Odom_Movement(fuctPtr_move_to, .1,{31,38,0}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{31,25,0}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{31,40,0}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, .2,{31,41,0}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to, 1,{31,35,0}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	//pros::delay(500);
	
	//segundo tiro
	Robot::Odom_Movement(fuctPtr_move_to, 1,{31,5,0}, Drive_Constant,Turn_Constant, 3, 0,0,0);
	Robot::Odom_Movement(fuctPtr_move_to,1, {6,17,180-2.1}, Drive_Constant,Turn_Constant, 3, 0,0,0);	
	Robot::eat(false);
	Robot::Flywheel_pid(2760, {.01535,.00000525,4.05}, 3);
	//Robot::eat(false);
	//Robot::Odom_Movement(fuctPtr_move_to, {30,28,0}, Drive_Constant, Turn_Constant, 3, 0,0,0);
}

//184

void opcontrol() {

	Robot::start_task("DRIVE", Robot::drive);
    Robot::start_task("TRACKING", Robot::raestro);
    
   
}
