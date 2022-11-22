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
    
    int sign_dy = dy>0 ? 1 : dy<0 ? -1 : sign_dy; 
    int sign_dx = dx>0 ? 1 : dx<0 ? -1 : sign_dx; 

    //Calculamos el angulo, utilizando arco tangente, pero con el plano desfasado para que concuerda con el del robot

	double Angulo =  TO_DEGREES (atan2(dx, dy));
    
    //Diferentes escenerios, dependiendo de los valores y signos de dy y dx, podemos setear
    //El angulo correspondiente que con el calculo normal del Angulo no es posible obtener el valor 
    //Correcto del angulo entre los dos puntos. 
    Angulo = dx==0 && sign_dy ==-1 ? 180 : dy==0 && sign_dx==1 ? 90 : dy==0 && sign_dx==-1 ? 270 : Angulo; 
    
    //El resultado lo ponemos en un rango de 0-360
	Angulo = reducir_angulo_0_360(Angulo);
	
	return  Angulo ;
}


double  Control_move_to(double Orientacion,double TargetX,double TargetY){
    //Se desprecian los valores de los target y solos se acepta el de Orientacion
    TargetX=0;
    TargetY=0;
    //No existe una alteracion en el calculo de orientacion
    Orientacion=Orientacion;
    return Orientacion;
}
 
double Control_move_facing_to(double Orientacion,double TargetX,double TargetY){
    //No se desperecian los valores del los target ni de la orientacion
    TargetX=TargetX;
    TargetY=TargetY;
    //Se altera el valor de la orientacion, la orientacion seria el resultado de la la diferencia que existe
    //Entre las coordendas actuales y las del target, para que el robot pueda apuntar a ese punto mientras se
    //Ejecuta el PID
    Orientacion= get_angle_pro({Robot::absGlobalX,Robot::absGlobalY}, {TargetX,TargetY});
    return Orientacion;
}








