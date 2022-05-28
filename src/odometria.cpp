#include "main.h"



//void Control_Odometria::reiniciar_variables(){
    //Distancia de las tracking wheels al centro
    float Distancia_D=0.75;

    float Distancia_B=0.30; 


    //Radios de las tracking wheels
    float radioD=1.5627;
    float radioB=1.5627;

    //Diametro de las tracking wheels 
    float Diametro_D=radioD*2;
    float Diametro_B=radioB*2;

    //360 por que existen 360 "ticks" en una revolucion en el encoder
    float f_Derecho=(3.1415*Diametro_D)/360;
    float f_back= (3.1415*Diametro_B)/360;

    //Zona muerta del encoder al momento de girar el robot 
    //De esta manera solo vamos a obtener las coordenadas cuando el robot se mueva en el eje x y Y 
   

    //Variables para calcular cambio en las tracking wheels
    float prev_D=0;
    float prev_B=0;

    //Variables para Posicion 
    float prevOrientacionRad=0;
    float prevGlobalX=0;
    float prevGlobalY=0;

    float absOrientacionRad=0;
    float absOrientacionDeg=0;

    //Coordenadas locales
    float localX=0;
    float localY=0;

    //Cambio en las tracking wheels
    float delta_D=0;
    float delta_B=0;

    //Posiciones absolutas
    float absGlobalX=0;
    float absGlobalY=0;

    //Valores del encoder
    float D_actual=0;
    float B_actual=0;

    float prev_A=0;

    //Variables para pasar voltaje
    float  theta_coordenadas=0;
    float strafe_lf_rb_odometria=0;
    float strafe_rf_lb_odometria=0;
    float max_strafe=0;

    
//}

float grados_a_radianes(float anguloGrados){
    anguloGrados/=57.2957795;
    return (anguloGrados);
}


float reducir_angulo_0_360(float anguloGrados){
     while(!(anguloGrados >=0 && anguloGrados<=360)){
        if(anguloGrados<0){
            anguloGrados+=360;
        }

        if(anguloGrados>360){
            anguloGrados-=360;
        }
    }

    return (anguloGrados);
}

float reducir_angulo_180_180(float anguloGrados){
      while(!(anguloGrados>=-180 && anguloGrados<=180)){
        if(anguloGrados<-180){
            anguloGrados+=180;
        }

        if(anguloGrados>180){
            anguloGrados-=180;
        }
    }

    
    return (anguloGrados);
}

void updateEncoders(void){
    //Realizar la resta entre el valor de los encoder menos la zona muerta hace
    //que en los giros no haya un cambio en la posicion

    float dead_zone_encoder_D = (2*gyro.get_heading()*Distancia_D)/(Diametro_D);
    float dead_zone_encoder_B = (2*gyro.get_heading()*Distancia_B)/(Diametro_B);

    D_actual= (Encoder_Derecho.get_value()- dead_zone_encoder_D) * (f_Derecho);
    B_actual = (Encoder_back.get_value() - dead_zone_encoder_B) * (f_back);


    delta_D = D_actual - prev_D;
    delta_B = B_actual - prev_B;

    prev_D= D_actual;
    prev_B = B_actual;


    //std::cout<<"\nEncoder Y: "<<D_actual;
    //std::cout<<"\tEncoder X: "<<B_actual;


}

void updatePosicion(void){

    absOrientacionDeg= gyro.get_heading();
    absOrientacionRad= grados_a_radianes(absOrientacionDeg);
    
    float angle_gyro_Rad= grados_a_radianes(gyro.get_heading());
    float delta_A= angle_gyro_Rad-prev_A;
    prev_A=angle_gyro_Rad;


    if(delta_A==0){
        localX = delta_B;
        localY = delta_D;
    }

    else{

        localX =(2*sin(delta_A/2)) * ((delta_B/delta_A)+Distancia_B);
        localY= (2*sin(delta_A/2)) * ((delta_D/delta_A)+Distancia_D);
    }

    float anguloPolarLocal=0;
    float distanciaPolarLocal=0;

    //Calculamos las coordenadas polares
    if (localX == 0 && localY == 0){  //Evitamos error de NaN, al momento de pasarlas a polares
        anguloPolarLocal = 0;
        distanciaPolarLocal = 0;
    } 
    
    else {
        anguloPolarLocal = atan2(localY, localX); 
       
        distanciaPolarLocal = hypot(localX,localY);
    }

    //Convertimos las coordenadas polares a globales
    float distanciaPolarGlobal = distanciaPolarLocal;
    float anguloPolarGlobal = anguloPolarLocal - prevOrientacionRad - (delta_A/2);

    float globalX = distanciaPolarGlobal * cos(anguloPolarGlobal);
    float globalY = distanciaPolarGlobal * sin(anguloPolarGlobal);

    //Calculamos las coordenadas y orientacion absolutas
    absGlobalX = Fil::kalman.filter(prevGlobalX + globalX); 
    absGlobalY = Fil::kalman.filter(prevGlobalY + globalY); 

    prevGlobalX = absGlobalX;
    prevGlobalY = absGlobalY;

    prevOrientacionRad = absOrientacionRad;

    std::cout<<"\nCoordenada en x "<< absGlobalX;
    std::cout<<"\t\tCoordenada en y "<< absGlobalY;
    std::cout<<"\t\tAngulo : "<< absOrientacionDeg;
}

void Raestreo_fn(void* param){
    while(1){
        updateEncoders();
        updatePosicion();
        pros::delay(10);
    }
}

//pros::Task Raestro(Raestreo_fn);



void Resetear_Posicion(void){
    Encoder_Derecho.reset();
    Encoder_back.reset();

    prev_D=0;
    prev_B=0;

    updateEncoders();
    updatePosicion();

    prevGlobalX=0;
    prevGlobalY=0;
}

void Odom_mover(float X, float Y, float Orientacion, float kp_drive, float ki_drive, float kd_drive, float kp_turn,float ki_turn,float kd_turn){
    
    
    Orientacion= reducir_angulo_0_360(Orientacion);

    float integral_raw_drive=0;
    float last_error_drive=0;

    float zonaintegralactiva_drive= (sqrt ( (pow((X-absGlobalX),2)) + pow((Y-absGlobalY),2) ) ) * .45;
    float integralpowerlimit_drive= 50/ki_drive;

    float integral_raw_turn=0;
    float last_error_turn=0;

    float zonaintegralactiva_turn= Orientacion * .3;
    float integralpowerlimit_turn= 50/ki_turn;

    float condicion_odometria=false;

    while(condicion_odometria==false){

       

        
        updateEncoders();
        updatePosicion();

        float drive_error=  (sqrt ( (pow((X-absGlobalX),2)) + pow((Y-absGlobalY),2) ) );
        float turn_error= reducir_angulo_180_180(Orientacion-absOrientacionDeg);

        float drive_proporcion= drive_error * kp_drive;
        float turn_proporcion= turn_error * kp_turn;

        if(fabs(drive_error)<zonaintegralactiva_drive && drive_error!=0){integral_raw_drive=0;}

        else{
            integral_raw_drive+= drive_error;
        }

        if(fabs(turn_error)<zonaintegralactiva_turn && turn_error!=0){integral_raw_turn=0;}

        else{
            integral_raw_turn+= turn_error;
        }

        integral_raw_drive = integral_raw_drive > integralpowerlimit_drive ? integralpowerlimit_drive : integral_raw_drive < -integralpowerlimit_drive ? -integralpowerlimit_drive: integral_raw_drive;

        float integral_drive= ki_drive*integral_raw_drive;

        integral_raw_turn= integral_raw_turn > integralpowerlimit_turn ? integral_raw_turn : integral_raw_turn <-integralpowerlimit_turn ? -integral_raw_drive : integral_raw_turn;

        float integral_turn= ki_turn*integral_raw_turn;

        float derivada_drive = kd_drive * (drive_error - last_error_drive);
        last_error_drive=drive_error;

        float derivada_turn= kd_turn * (turn_error - last_error_turn);
        last_error_turn= turn_error;

         float finalpower_drive= (ceil(drive_proporcion+integral_drive+derivada_drive));
         float finalpower_turn= (ceil(turn_proporcion+integral_turn+derivada_turn));

        finalpower_drive= finalpower_drive > 180 ? 180 : finalpower_drive < -180 ? -180:finalpower_drive;
        finalpower_turn= finalpower_turn > 150 ? 150 : finalpower_drive < -150 ? -150:finalpower_turn;

        if(fabs(absOrientacionDeg)<3){
           finalpower_turn=0; 
        }

        if(fabs(drive_error)<1){
            finalpower_drive=0;
        }
      

       LeftFront_1.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn) ;
       LeftFront_2.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);

       LeftBack_1.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);
       LeftBack_2.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);

       RightFront_1.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
       RightFront_2.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
       
       RightBack_1.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
       RightBack_2.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
    
      // std::cout<<"\nFinal power "<<finalpower_drive;
       // std::cout<<"\nCoordenada en x "<< absGlobalX;
        //std::cout<<"\tCoordenada en y "<< absGlobalY;
        //std::cout<<"\tError: "<< drive_error;
        std::cout<<"\n Giro: "<<absOrientacionDeg;

        std::cout<<"\tError giro: "<<turn_error;
        std::cout<<"\tError drive "<<drive_error;

       if(fabs(drive_error)<1 && fabs(turn_error) < 3 ){
           condicion_odometria=true;
       }

        
        pros::delay(10);
    }


        std::cout<<"\nCiclo se cerro";

        LeftFront_1.move_velocity(0);   RightFront_1.move_velocity(0);
        LeftFront_2.move_velocity(0);   RightFront_2.move_velocity(0);

        LeftBack_1.move_velocity(0);   RightBack_1.move_velocity(0);
        LeftBack_2.move_velocity(0);   RightBack_2.move_velocity(0);

        pros::delay(10);



}

