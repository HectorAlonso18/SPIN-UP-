#ifndef ODOMETRIA_H
#define ODOMETRIA_H





//Distancia de las tracking wheels al centro
extern float Distancia_D;
extern float Distancia_B; 

//Radios de las tracking wheels dividido entre 360
extern float radioD;
extern float radioB;

//Variables para calcular cambio en las tracking wheels
 extern float prev_D;
 extern float prev_B;

//Variables para Posicion
 extern float prevOrientacionRad;
  extern float prevGlobalX;
 extern float prevGlobalY;
 

//Coordenadas locales
 extern float localX;
 extern float localY;

//Cambio en las tracking wheels
 extern float delta_D;
 extern float delta_B;

//Posiciones absolutas
 extern float absGlobalX;
 extern float absGlobalY;

//Valores del encoder
 extern float D_actual;
 extern float B_actual;

extern float absOrientacionRad;
extern float   absOrientacionDeg;


extern float prev_A;


//Variables para pasar voltaje
extern float theta_coordenadas;
extern float strafe_lf_rb_odometria;
extern float strafe_rf_lb_odometria;
extern float max_strafe;




float grados_a_radianes(float anguloGrados);
float reducir_angulo_0_360(float anguloGrados);
float reducir_angulo_180_180(float anguloGrados);

void reiniciar_variables(void);
void updateEncoders(void);
void updatePosicion(void);
void Resetear_Posicion(void);

void Raestreo_fn(void* param);

//pros::Task Raestro;


void Odom_mover(float X, float Y, float Orientacion, float kp_drive, float ki_drive, float kd_drive, float kp_turn,float ki_turn,float kd_turn);


#endif