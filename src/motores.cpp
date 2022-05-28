#include "main.h"

//Los motores se enumeran del frente pa delante, es decir tu viendo el robot de frente 
 pros::Motor LeftFront_1 (10,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
 pros::Motor LeftFront_2 (9,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);

 pros::Motor LeftBack_1 (1,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
 pros::Motor LeftBack_2 (2,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);


 pros::Motor RightFront_1   (20,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
 pros::Motor RightFront_2   (19,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);


 pros::Motor RightBack_1   (11,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
 pros::Motor RightBack_2   (12,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);






 pros::Imu gyro(5);


pros::ADIEncoder Encoder_Derecho ('A','B',true);
pros::ADIEncoder Encoder_back ('G','H',false);

