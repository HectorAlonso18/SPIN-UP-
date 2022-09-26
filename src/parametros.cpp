#include "parametros.h"
#include "odometria.h"
#include <vector>

std::vector<double> X_python{22.226053, 22.226053, 0.9391290000000001, -0.313043, -22.852139, -46.643407, 
-44.452106, 0.9391290000000001, 0.626086, 22.852139, 45.078192, 46.017321, 45.391235, 0.0}; 
std::vector<double> Y_python{2.504344, 24.417354, 45.391235, 68.55641700000001, 69.495546, 46.330364, 92.034642, 92.660728, 46.643407, 44.765149, 46.956450000000004, 23.791268000000002, 1.8782580000000002, 0.0};

std::vector<double>Drive_Constant={8, 0.037, 60}; 
std::vector<double>Turn_Constant={1.56, .015, 15}; 

double(*fuctPtr_move_to)(double,double,double) = Control_move_to;
double(*fuctPtr_facing_to)(double,double,double)= Control_move_facing_to;

void(*fuctPtr_chasis_lineal)(double,double)= Control_PID_lineal;
void(*fuctPtr_chasis_turn)(double,double) = Control_PID_turn; 



