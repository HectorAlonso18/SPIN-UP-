#include "PID.hpp"
#include "auxiliar_fuctions.h"
#include "main.h"


PID_Control DRIVE={
    .kp=0, .ki=0, .kd=0, .integral_raw=0, .last_error=0, .zonaintegralactiva=0, .integralpowerlimit=0, .error=0, .proporcion=0, .integral=0, .derivada=0, .finalpower=0    
};

PID_Control TURN={
     .kp=0, .ki=0, .kd=0, .integral_raw=0, .last_error=0, .zonaintegralactiva=0, .integralpowerlimit=0, .error=0, .proporcion=0, .integral=0, .derivada=0, .finalpower=0
};

PID_Control FLYWHEEL = {
    .kp=0, .ki=0, .kd=0, .integral_raw=0, .last_error=0, .zonaintegralactiva=0, .integralpowerlimit=0, .error=0, .proporcion=0, .integral=0, .derivada=0, .maximo=0 ,.finalpower=0
};


float Get_Proporcion (PID_Control Compensator){
   return  Compensator.proporcion = Compensator.error * Compensator.kp; 
}

float Get_Integral_raw (PID_Control Compensator){
    if(fabs(Compensator.error)>Compensator.zonaintegralactiva && Compensator.error!=0){Compensator.integral_raw=0;}

    else{Compensator.integral_raw+= Compensator.error;}

    Compensator.integral_raw = In_range(Compensator.integral_raw, -Compensator.integralpowerlimit, Compensator.integralpowerlimit); 

    return Compensator.integral_raw; 
}

float Get_Integral(PID_Control Compensator){
   return Compensator.integral = Compensator.ki * Compensator.integral_raw; 
}

float Get_Derivada(PID_Control Compensator){
   return Compensator.derivada= Compensator.kd* (Compensator.error- Compensator.last_error); 
}

float Get_Pid_Output(PID_Control Compensator, float potencia){
    Compensator.finalpower= ceil(Compensator.proporcion + Compensator.integral + Compensator.derivada);
    Compensator.finalpower = In_range(Compensator.finalpower, -Compensator.maximo, Compensator.maximo); 
    Compensator.finalpower *= potencia; 
    return Compensator.finalpower; 
}