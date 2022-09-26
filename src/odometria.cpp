#include "main.h"
#define TO_DEGREES(n) (n*180)/M_PI;

double reducir_angulo_0_360(double anguloGrados){
    while(!(anguloGrados >=0 && anguloGrados<=360)){
        anguloGrados = anguloGrados<0 ? anguloGrados+=360 : anguloGrados>360 ? anguloGrados-=360 : anguloGrados;
        //if(anguloGrados<0){anguloGrados+=360;}
        //if(anguloGrados>360){anguloGrados-=360;}
    }
    return (anguloGrados);
}

double reducir_angulo_180_180(double anguloGrados){
    while(!(anguloGrados>=-180 && anguloGrados<=180)){
        anguloGrados = anguloGrados<-180 ? anguloGrados+=360 : anguloGrados>180 ? anguloGrados -=360 : anguloGrados;

        //if(anguloGrados<-180){anguloGrados+=360;}
        //if(anguloGrados>180){anguloGrados-=360;}
    }
    return (anguloGrados);
}


double get_angle_pro(std::vector<double> Current, std::vector<double> Target){
    
    //Calculamos un diferencial de Y y X
    //dy = Y2 - Y1
    double dy = Target[1] - Current[1]; 
    //dx= X2 -X1
	double dx = Target[0] - Current[0]; 
    
    //Calculamos el angulo, utilizando arco tangente, pero con el plano desfasado para que concuerda con el del robot
	double Angulo =  TO_DEGREES (atan2(dx, dy));

    //El resultado lo ponemos en un rango de 0-360
	Angulo = reducir_angulo_0_360(Angulo);
	
	return  Angulo ;
}
 
double  Control_move_to(double Orientacion,double TargetX,double TargetY){
    TargetX=0;
    TargetY=0;
    Orientacion=Orientacion;
    return Orientacion;
}
 
double Control_move_facing_to(double Orientacion,double TargetX,double TargetY){
    TargetX=TargetX;
    TargetY=TargetY;
    Orientacion= get_angle_pro({Robot::absGlobalX,Robot::absGlobalY}, {TargetX,TargetY});
    return Orientacion;
}


void Control_PID_lineal(double finalpower, double finalpower_turn){
    finalpower=finalpower;
    finalpower_turn=0;
}

void Control_PID_turn(double finalpower,double finalpower_turn){
    finalpower=0;
    finalpower_turn=finalpower_turn;
}









