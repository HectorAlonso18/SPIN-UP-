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


extern double(*fuctPtr_move_to)(double,double,double);
extern double(*fuctPtr_facing_to)(double,double,double);

extern void(*fuctPtr_chasis_lineal)(double,double);
extern void(*fuctPtr_chasis_turn)(double,double); 

#endif