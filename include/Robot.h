#include "main.h"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>



class Robot
{
public: 
 static pros::Motor LeftFront_1; 
 static pros::Motor LeftFront_2;

 static pros::Motor LeftBack_1;
 static pros::Motor LeftBack_2;

 static pros::Motor RightFront_1;
 static pros::Motor RightFront_2;

 static pros::Motor RightBack_1;
 static pros::Motor RightBack_2;

 static pros::Motor FlyWheel_1;
 static pros::Motor Flywheel_2; 

 static pros::Motor intaker_1;
 static pros::Motor intaker_2;

 static pros::Motor Indexer;

 static pros::Imu gyro;
 static pros::Rotation Rotacion; 

 static pros::ADIEncoder Encoder_Derecho;
 static pros::ADIEncoder Encoder_back;

 static pros::Controller master;

 /* Mapping of tasks instantiated during the program */
 static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

 /* Note: tasks are the pros version of threads, or a method for having independent subroutines run at the same time. Using
 threading allows us to have different functions run simultaneously, which helps us save time and increase versatility in
 our code */

 /*Starts a task and pairs it with a unique task ID to allow us to keep track of its status*/                                                      
 static void start_task(std::string name, void (*func)(void *)); 

 /*Checks if task exists */
 static bool task_exists(std::string name);

 /*Kills a specific task by terminating it and removing it from Robot::tasks*/
 static void kill_task(std::string name);
 
 /*Distancia del Encoder Y al centro del robot */
 static double Distancia_Encoder_Y;
 /*Distancia del Encoder X al centro del robot*/
 static double Distancia_Encoder_X;
 
 /*tracking wheel Y radio */
 static double radioY;
 /*tracking wheel X radio*/
 static double radioX; 

 /*tracking wheel Y diameter */
 static double Diametro_Y;
 /*tracking wheel X diameter */
 static double Diametro_X;
 
 /*Frecuencia del Encoder_Derecho*/
 static double f_Derecho;
 /*Frecuencia del Encoder_Trasero*/
 static double f_back;

 /*Valores previos del Encoder*/  
 static double prev_Y;
 /*Valores previos del Encoder*/ 
 static double prev_X;
 
 ////Variables necesarias para el calculo de coordenadas y posicion///

 static double prevOrientacionRad;
 static double prevGlobalX;
 static double prevGlobalY;
///////////////////////////////////////////////////////////////////

 //Orientacion actual en radianes
 static std::atomic<double>absOrientacionRad;
 //Oreintacion actual en grados
 static std::atomic<double>absOrientacionDeg;
 
 //Variable snecesarias para el calculo de coordenadas////
 static double localX;
 static double localY;

 static double delta_Y;
 static double delta_X;
//////////////////////////////////////////////////////////
 
 //Coordenadas actuales del robot

 static std::atomic<double>absGlobalX; //Coordenada en X
 static std::atomic<double>absGlobalY; //Coordenada en Y

 static double lookAheadDis; //Parametro para purepursuit

 //Valor de los dos encoders actuales
 //Encoder Y
 static double EncoderY_Actual; 
 //Encoder X
 static double EncoderX_Actual;
 //
 
 //Orientacion_anterior
 static double prev_A;
 
 //Sistema de coordenadas para la canasta
 //Coordenada en X
 static double High_GoalX;
 //Coordenada en Y
 static double High_GoalY;
 
 //Variable necesaria para el PID en el modo driver, es el final Power
 static std::atomic<double> turn_joytick;

 /*Raestrea la posicion y orientacion del robot usando odometria*/
 static void raestro(void *ptr);
 /*Imprime las coordenadas X, Y , Orientacion */
 static void get_data(void);
 
 /*Actualiza los valores de los Encoders */
 static void updateEncoders(void);
 /*Actualiza los valores de las Coordenadas */
 static void updatePosicion(void);

 /*Mueve el robot a una determinada coordenada y posición, usando dos controladores PID
   Modo_facing -> Mueve el robot a una determinada coordenada y una coordenada a apuntar*/
 static void Odom_Movement(double(*fuctPtr_Mode)(double,double,double),std::vector<double> posicion, std::vector<double>DrivePID, std::vector<double>TurnPID, double tiempo, double TargetX, double TargetY);
 
 /*Mueve el robot con PID,  Modo 1:lineal , Modo 2: giros*/
 static void PID_Movement(void(*fuctPtr_Mode)(double,double),std::vector<double>posicion,std::vector<double>DrivePID,std::vector<double> TurnPID,double tiempo);

 static void move_to_pure_pursuit(std::vector< std::vector<double> > points, std::vector<double> final_point, int lookAheadDistance);

 static void python_movement( double(*fuctPtr_mode)(double,double,double),std::vector<double> X, std::vector<double> Y, float tiempo);

 //Controlador del flywheel
 static void move_flywheel(int RPM, float tiempo);

 /*Modo driver del robot */
 //Modo driver
 static void drive(void *ptr);
 //Calculos necesarios para mover un X-Drive
 static void x_drive(double power, double strafe_lf_rb, double strafe_rf_lb, double turn);
 //Mueve el Flywheel con una entrada de voltaje (mV)
 static void move_Flywheel(int power);
 //Mueve el Intake con una entrada de voltaje (mV)
 static void move_Intake(int power);
 //Acciona el disparador dependiendo de la entrada (bool)
 static void move_Indexer(void); 
 //PID de modo driver, setea un angulo durante el periodo Driver, el robot se quedara anclado
 //En la posicion que se ha indicado, puedes desaclarlo moviendo el joytisck derecho
 static void PID_drift(void *ptr);

 /*setear el tipo de frenado */
 static void brake(std::string mode);

 /*Reinicia todos los sensores */
 static void reset_sensors();

};
