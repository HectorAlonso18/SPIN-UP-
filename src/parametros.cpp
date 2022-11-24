#include "parametros.h"
#include "odometria.h"
#include <vector>


std::vector<double> X_python{0}; 
std::vector<double> Y_python{0};

//Azul
///////Constantes del PID///////////////////////

//Constantes del PID para posicion

std::vector<double>Drive_Constant={10,.0001,22.5};
//Constantes del PID para la orientacion 

std::vector<double>Turn_Constant={1.5,0.00150,1.7}; 


//Constantes del PID para Flywheel
std::vector<double> Flywheel_Constant ={.01535,.00000525,4.05};

double(*fuctPtr_move_to)(double,double,double) = Control_move_to;
double(*fuctPtr_facing_to)(double,double,double)= Control_move_facing_to;





