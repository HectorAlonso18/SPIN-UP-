#include "parametros.h"
#include "odometria.h"
#include <vector>





std::vector<double> X_path{-100}; 
std::vector<double> Y_path{47.5};

//Azul
///////Constantes del PID///////////////////////


std::vector<double>Drive_Constant={10,.0001,22.5};


//1.5,0.00150,1.7
std::vector<double>Turn_Constant={1.5,0.00150,1.7}; 

//.05,.0000001,5.5
std::vector<double> Flywheel_Constant ={.04,0,2.5};

double(*fuctPtr_move_to)(double,double,double) = Control_move_to;
double(*fuctPtr_facing_to)(double,double,double)= Control_move_facing_to;






