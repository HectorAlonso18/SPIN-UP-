#include "main.h"
#include "pros/adi.hpp"

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

 static pros::Imu gyro;

 static pros::ADIEncoder Encoder_Derecho;
 static pros::ADIEncoder Encoder_back;

 static pros::ADIDigitalOut piston; 

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

 static double prevOrientacionRad;
 static double prevGlobalX;
 static double prevGlobalY;

 static std::atomic<double>absOrientacionRad;
 static std::atomic<double>absOrientacionDeg;

 static double localX;
 static double localY;

 static double delta_Y;
 static double delta_X;

 static std::atomic<double>absGlobalX;
 static std::atomic<double>absGlobalY;

 static double lookAheadDis;

 static double EncoderY_Actual;
 static double EncoderX_Actual;

 static double prev_A;

 static double High_GoalX;
 static double High_GoalY;

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
 static void drive(void *ptr);
 static void x_drive(double power, double strafe_lf_rb, double strafe_rf_lb, double turn);
 static void Fly_wheel_action(int power);

 static void Piston_movement(bool state); 

 static void PID_drift(void *ptr);

 /*setear el tipo de frenado */
 static void brake(std::string mode);

 /*Reinicia todos los sensores */
 static void reset_sensors();

};
