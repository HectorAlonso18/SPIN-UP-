
#include <vector>

/*Reducir un ángulo de 0 a 360*/
double reducir_angulo_0_360(double anguloGrados);

/*Reducir un ángulo de -180 a 180*/
double reducir_angulo_180_180(double anguloGrados);


/*Calcula el angulo entre dos vectores (Tomando en cuenta el plano del Robot) */
double get_angle_pro(std::vector<double> Current, std::vector<double> Target);


//Diferentes configuraciones para los movimientos con odometria

//move_to, te puedes mover de un punto a otro, escogiendo la orientación
double Control_move_to(double Orientacion,double TargetX,double TargetY);
//move_facint_to, te puedes mover de un punto a otro, mientras apuntas a otro
double Control_move_facing_to(double Orientacion,double TargetX,double TargetY);

//Diferentes configuraciones para compensador de posición o de orientación 

