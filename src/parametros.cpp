#include "parametros.h"
#include "odometria.h"
#include <vector>


std::vector<double> X_python{0}; 
std::vector<double> Y_python{0};

///////Constantes del PID///////////////////////

//Constantes del PID para posicion
//8,.037,60
std::vector<double>Drive_Constant={8, .037, 60};
//Constantes del PID para la orientacion 
//1.56,.015,15
std::vector<double>Turn_Constant={0, 0, 0}; 


double(*fuctPtr_move_to)(double,double,double) = Control_move_to;
double(*fuctPtr_facing_to)(double,double,double)= Control_move_facing_to;

void(*fuctPtr_chasis_lineal)(double,double)= Control_PID_lineal;
void(*fuctPtr_chasis_turn)(double,double) = Control_PID_turn; 



