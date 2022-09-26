
#include <vector>

/*Reducir un ángulo de 0 a 360*/
double reducir_angulo_0_360(double anguloGrados);

/*Reducir un ángulo de -180 a 180*/
double reducir_angulo_180_180(double anguloGrados);

/*Calcula el angulo entre dos vectores */
double get_angle_pro(std::vector<double> Current, std::vector<double> Target);

double Control_move_to(double Orientacion,double TargetX,double TargetY);
double Control_move_facing_to(double Orientacion,double TargetX,double TargetY);

void Control_PID_lineal(double finalpower, double finalpower_turn);
void Control_PID_turn(double finalpower,double finalpower_turn);

