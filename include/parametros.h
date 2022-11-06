#ifndef PARAMETROS_H
#define PARAMETROS_H
#include <vector>

//Coordena X extraida de python
extern std::vector<double> X_python;
//Coordena Y extraida de python 
extern std::vector<double> Y_python;

//Constantes de PID para DRIVE
extern std::vector<double>Drive_Constant;
//Constantes de PID para TURN 
extern std::vector<double>Turn_Constant; 

/*Punteros para la pasarlas como funcion y tener diferentes implementaciones en un mismo codigo*/

//Puntero para la funcion Odom_move
extern double(*fuctPtr_move_to)(double,double,double);
//Puntero para la funcion odom_move
extern double(*fuctPtr_facing_to)(double,double,double);

//Puntero para la funcion PID movement
extern void(*fuctPtr_chasis_lineal)(double,double);
//Puntero para la funcion PID movement
extern void(*fuctPtr_chasis_turn)(double,double); 

#endif