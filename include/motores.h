#ifndef MOTORES_H
#define MOTORES_H


//////////////
///////////////Motores del chassis//////////////////////////

extern pros::Motor LeftFront_1; 
extern pros::Motor LeftFront_2;

extern pros::Motor LeftBack_1;
extern pros::Motor LeftBack_2;



extern pros::Motor RightFront_1;
extern pros::Motor RightFront_2;

extern pros::Motor RightBack_1;
extern pros::Motor RightBack_2;



//////////////////////////////////////////////////////////////////////////

///////////////////////SENSORES DE PUERTO INTELIGENTE//////////////////
extern pros::Imu gyro;

extern pros::ADIEncoder Encoder_Derecho;
extern pros::ADIEncoder Encoder_back;

#endif