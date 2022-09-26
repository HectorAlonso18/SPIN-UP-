#ifndef  PID_HPP
#define PID_HPP

typedef struct{

   	float kp;           
  	float ki;            								
	float kd;

	float integral_raw;
	float last_error;

	float zonaintegralactiva;
	float integralpowerlimit;

	float error;

	float proporcion;
	float integral;
	float derivada;
	
	float finalpower;            							
 
}PID_Control;

extern PID_Control DRIVE;
extern PID_Control TURN;



#endif