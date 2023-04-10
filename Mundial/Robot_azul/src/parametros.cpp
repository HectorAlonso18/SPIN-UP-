#include "parametros.h"
#include "odometria.h"
#include <vector>





std::vector<double> X_path{-100}; 
std::vector<double> Y_path{47.5};

//Azul
///////Constantes del PID///////////////////////



std::vector<double>Drive_Constant={9,.00012,19.25};


std::vector<double>Turn_Constant={2,0.00280,12}; 

//10,0,.05
std::vector<double> Flywheel_Constant = {9,0.0025,1.5}; 


double(*fuctPtr_move_to)(double,double,double) = Control_move_to;
double(*fuctPtr_facing_to)(double,double,double)= Control_move_facing_to;






