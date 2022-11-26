#include "PID.hpp"
#include "main.h"


PID_Control DRIVE={
    .kp=0, .ki=0, .kd=0, .integral_raw=0, .last_error=0, .zonaintegralactiva=0, .integralpowerlimit=0, .error=0, .proporcion=0, .integral=0, .derivada=0, .finalpower=0    
};

PID_Control TURN={
     .kp=0, .ki=0, .kd=0, .integral_raw=0, .last_error=0, .zonaintegralactiva=0, .integralpowerlimit=0, .error=0, .proporcion=0, .integral=0, .derivada=0, .finalpower=0
};

PID_Control FLYWHEEL = {
    .kp=0, .ki=0, .kd=0, .integral_raw=0, .last_error=0, .zonaintegralactiva=0, .integralpowerlimit=0, .error=0, .proporcion=0, .integral=0, .derivada=0, .finalpower=0
};




