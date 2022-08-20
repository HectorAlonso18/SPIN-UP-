#include "main.h"



double reducir_angulo_0_360(double anguloGrados){
     while(!(anguloGrados >=0 && anguloGrados<=360)){
        if(anguloGrados<0){anguloGrados+=360;}

        if(anguloGrados>360){anguloGrados-=360;}
    }

    return (anguloGrados);
}


double reducir_angulo_180_180(double anguloGrados){
      while(!(anguloGrados>=-180 && anguloGrados<=180)){
        if(anguloGrados<-180){anguloGrados+=360;}

        if(anguloGrados>180){anguloGrados-=360;}
    }
    
    return (anguloGrados);
}









