#include "Purepursuit.h"
#include "main.h"


#include "odometria.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

#include <cmath>
#include <atomic>
#include <iostream>
#include <ostream>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
#include <bits/stdc++.h> 


//Lambda fuction, para convertir de grados a radianes
#define TO_RAD(n) n * M_PI / 180;
#define TO_DEGREES(n) (n*180)/M_PI;


//Puerto 1, 4  no jala

pros::Motor Robot::LeftFront_1(3,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::LeftFront_2(2,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::LeftBack_1(12,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::LeftBack_2(11,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::RightFront_1(9,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::RightFront_2(10,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::RightBack_1(19,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::RightBack_2(20,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::FlyWheel_1(17,pros::E_MOTOR_GEARSET_06,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::Flywheel_2(18,pros::E_MOTOR_GEARSET_06,false,pros::E_MOTOR_ENCODER_DEGREES); 
 
pros::Imu Robot::gyro(8);

pros::ADIEncoder Robot::Encoder_Derecho('E','F',false);
pros::ADIEncoder Robot::Encoder_back('G','H',true);

pros::ADIDigitalOut Robot::piston('A'); 


pros::Controller Robot::master(pros::E_CONTROLLER_MASTER);
 
std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

void Robot::start_task(std::string name, void(*func)(void*)){
     if(!task_exists(name)){
         tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, (void*)"PROS",TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT,""))));
     }
 }


bool Robot::task_exists(std::string name) {
	return tasks.find(name) != tasks.end();
}


void Robot::kill_task(std::string name) {
	if (task_exists(name)) {
		tasks.erase(name);
	}
}


 //6.53
 //2.3
 
double Robot::Distancia_Encoder_Y=1;
double Robot::Distancia_Encoder_X=1;
  
double Robot::radioY=1.5627;
double Robot::radioX=1.5627;

double Robot::Diametro_Y=Robot::radioY*2;
double Robot::Diametro_X=Robot::radioX*2;

double Robot::f_Derecho=(M_PI*Robot::Diametro_Y)/360;
double Robot::f_back=(M_PI*Robot::Diametro_X)/360;
   
double Robot::prev_Y=0;
double Robot::prev_X=0;

double Robot::prevOrientacionRad=0;
double Robot::prevGlobalX=0;
double Robot::prevGlobalY=0;

std::atomic<double>Robot::absOrientacionRad(0);
std::atomic<double>Robot::absOrientacionDeg(0);

double Robot::localX=0;
double Robot::localY=0;

double Robot::delta_Y=0;
double Robot::delta_X=0;

std::atomic<double>Robot::absGlobalX(0);
std::atomic<double>Robot::absGlobalY(0);

double lookAheadDis=0;

double Robot::EncoderY_Actual=0;
double Robot::EncoderX_Actual=0;

double Robot::prev_A=0;

double Robot::High_GoalX=5.06;
double Robot::High_GoalY=96.73;

void Robot::raestro(void *ptr){
    while(true){
        updateEncoders();
        updatePosicion();
        pros::delay(10);
    }
}

void Robot::get_data(void){
    for(auto i=0; i<=1000; i++){
    std::cout<<"\nHeading: "<<"\t"<<Robot::absOrientacionDeg;
	std::cout<<"\tX: "<<"\t"<<Robot::absGlobalX;
	std::cout<<"\tY: "<<"\t"<<Robot::absGlobalY; 
	pros::delay(10);
	i+=10;
    }
}

double Robot::get_angle_pro(std::vector<double> Current, std::vector<double> Target){

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

void Robot::updateEncoders(void){
    /* 
    Utilizando trigonometria, podemos calcular una zona muerta que nos ayudara a que 
    Los encoder no cambien de valor cuando el robot rote, dejando los cambios del encoder 
    unicamente relaciónados con un cambio de Posición y no uno de Rotación
    */

    //Calculo de Zona muerta de los Encoder
    float dead_zone_encoder_D = (2*gyro.get_heading()*Distancia_Encoder_Y)/(Diametro_Y);
    float dead_zone_encoder_B = (2*gyro.get_heading()*Distancia_Encoder_X)/(Diametro_X);

    //Valor del Encoder con bounds (Zona muerta)
    EncoderY_Actual= (Encoder_Derecho.get_value()- dead_zone_encoder_D) * (f_Derecho);
    EncoderX_Actual = (Encoder_back.get_value() - dead_zone_encoder_B) * (f_back);
    
    //Deltas en los valores de los Encoder
    delta_Y = EncoderY_Actual - prev_Y;
    delta_X = EncoderX_Actual - prev_X;
    
    //Almacenando los valores anteriores de los Encoder
    prev_Y= EncoderY_Actual;
    prev_X= EncoderX_Actual;   

}



void Robot::updatePosicion(void){
    /*
    Almacenamiento de datos sobre orientación y ángulo del robot, en radianes y Grados
    */
    absOrientacionDeg= gyro.get_heading();
    absOrientacionRad= TO_RAD(absOrientacionDeg);
    
    float angle_gyro_Rad= TO_RAD(gyro.get_heading());
    float delta_A= angle_gyro_Rad-prev_A;
    prev_A=angle_gyro_Rad;

    //Si no hay cambio en el ángulo el local offset será igual a cero
    if(delta_A==0){
        localX = delta_X;
        localY = delta_Y;
    }
    
    //De lo contrario se calculará el local offset en X y en Y [ ]
    else{
        localX =(2*sin(delta_A/2)) * ((delta_X/delta_A)+Distancia_Encoder_X);
        localY= (2*sin(delta_A/2)) * ((delta_Y/delta_A)+Distancia_Encoder_Y);
    }

    //Variables para realizar el cambio a valores polares 
    float anguloPolarLocal=0;
    float distanciaPolarLocal=0;

    //Calculamos las coordenadas polares
    if (localX == 0 && localY == 0){  //Evitamos error de NaN, al momento de pasarlas a polares
        anguloPolarLocal = 0;
        distanciaPolarLocal = 0;
    } 

    //Si es diferente de cero, calcular las coordenadas polares 
    else {
        anguloPolarLocal = atan2(localY, localX); 
        distanciaPolarLocal = hypot(localX,localY);
    }

    //Convertimos las coordenadas polares a globales
    float distanciaPolarGlobal = distanciaPolarLocal;
    //Para el Angulo, debemos de calcular la orientación promedio dado por = thetha_0 + DeltaThehta/2
    //Para después rotarlo por -theta Polar 
    float anguloPolarGlobal = anguloPolarLocal - prevOrientacionRad - (delta_A/2);

    //Después debemos de regresar nuestras coordenadas polares a globales

    float globalX = distanciaPolarGlobal * cos(anguloPolarGlobal);
    float globalY = distanciaPolarGlobal * sin(anguloPolarGlobal);

    //Calculamos las nuevas posiciones absolutas
    //Tomando en cuenta las coordenadas relativas que se suman conforme la anterior
    absGlobalX =(prevGlobalX + globalX); 
    absGlobalY =(prevGlobalY + globalY); 

    prevGlobalX = absGlobalX;
    prevGlobalY = absGlobalY;

    prevOrientacionRad = absOrientacionRad;

   // std::cout<<"\nCoordenada X "<<Robot::absGlobalX;
   // std::cout<<"\tCoordenada Y "<<Robot::absGlobalY;

   
}

 

 void Robot::move_to(std::vector<double> posicion, double kp_drive, double ki_drive, double kd_drive, double kp_turn,double ki_turn,double kd_turn, double tiempo){
     
     double X= posicion[0];
     double Y=posicion[1];
     double Orientacion= posicion[2];

     Orientacion= reducir_angulo_0_360(Orientacion);
    
     float integral_raw_drive=0;
     float last_error_drive=0;

     float zonaintegralactiva_drive= (hypot(X-absGlobalX,Y-absGlobalY))*.45;
     float integralpowerlimit_drive= 50/ki_drive;

     float integral_raw_turn=0;
     float last_error_turn=0;

     float zonaintegralactiva_turn= Orientacion * .3;
     float integralpowerlimit_turn= 50/ki_turn;

     bool condicion_odometria=false;
     
     tiempo*=1000;
     int contador=0;

     while(condicion_odometria==false){

         float drive_error= hypot(X-absGlobalX,Y-absGlobalY); 
         float turn_error= reducir_angulo_180_180(Orientacion- absOrientacionDeg);

         float drive_proporcion= drive_error * kp_drive;
         float turn_proporcion= turn_error * kp_turn;

         if(fabs(drive_error)>zonaintegralactiva_drive && drive_error!=0){integral_raw_drive=0;}

         else{integral_raw_drive+= drive_error;}

         if(fabs(turn_error)>zonaintegralactiva_turn && turn_error!=0){integral_raw_turn=0;}

         else{integral_raw_turn+= turn_error;}

         integral_raw_drive = integral_raw_drive > integralpowerlimit_drive ? integralpowerlimit_drive : integral_raw_drive < -integralpowerlimit_drive ? -integralpowerlimit_drive: integral_raw_drive;

         float integral_drive= ki_drive*integral_raw_drive;

         integral_raw_turn= integral_raw_turn > integralpowerlimit_turn ? integral_raw_turn : integral_raw_turn <-integralpowerlimit_turn ? -integral_raw_turn : integral_raw_turn;

         float integral_turn= ki_turn*integral_raw_turn;

         float derivada_drive = kd_drive * (drive_error - last_error_drive);
         last_error_drive=drive_error;

         float derivada_turn= kd_turn * (turn_error - last_error_turn);
         last_error_turn= turn_error;

         float finalpower_drive= (ceil(drive_proporcion+integral_drive+derivada_drive));
         float finalpower_turn= (ceil(turn_proporcion+integral_turn+derivada_turn));

         finalpower_drive= finalpower_drive > 180 ? 180 : finalpower_drive < -180 ? -180:finalpower_drive;
         finalpower_turn= finalpower_turn > 150 ? 150 : finalpower_turn < -150 ? -150:finalpower_turn;

         //if(fabs(turn_error)<turn_error_range){finalpower_turn=0;}
         //if(fabs(drive_error)<drive_error_range){finalpower_drive=0;}

         Robot::LeftFront_1.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn) ;
         Robot::LeftFront_2.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);

         Robot::LeftBack_1.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);
         Robot::LeftBack_2.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);

         Robot::RightFront_1.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
         Robot::RightFront_2.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
       
         Robot::RightBack_1.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
         Robot::RightBack_2.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
         
        
         if((fabs(drive_error)<.2 && fabs(turn_error) <2)|| contador>=tiempo){condicion_odometria=true;}

         std::cout<<"\nTiempo: "<<contador;
         std::cout<<"\tD Error: "<<drive_error;
         std::cout<<"\tT Error: "<<turn_error;

         pros::delay(10);
         contador+=10;
    }
        
        Robot::brake("stop");
             
        pros::delay(10);
 }

void Robot::move_facing_to(std::vector<double>posicion, double TargetX, double TargetY, double kp_drive, double ki_drive, double kd_drive, double kp_turn,double ki_turn,double kd_turn, double tiempo){
    double X=posicion[0];
    double Y=posicion[1]; 

    float integral_raw_drive=0;
    float last_error_drive=0;

    float zonaintegralactiva_drive= (hypot(X-absGlobalX,Y-absGlobalY))*.45;
    float integralpowerlimit_drive= 50/ki_drive;

    float integral_raw_turn=0;
    float last_error_turn=0;

    float integralpowerlimit_turn= 50/ki_turn;

    bool condicion_odometria=false;
     
    tiempo*=1000;
    int contador=0;

    while(condicion_odometria==false){
         
         double Orientacion= get_angle_pro({absGlobalX,absGlobalY}, {TargetX,TargetY});
         float zonaintegralactiva_turn= Orientacion * .3; 

         float drive_error= hypot(X-absGlobalX,Y-absGlobalY); 
         float turn_error= reducir_angulo_180_180(Orientacion- absOrientacionDeg);

         float drive_proporcion= drive_error * kp_drive;
         float turn_proporcion= turn_error * kp_turn;

         if(fabs(drive_error)>zonaintegralactiva_drive && drive_error!=0){integral_raw_drive=0;}

         else{integral_raw_drive+= drive_error;}

         if(fabs(turn_error)>zonaintegralactiva_turn && turn_error!=0){integral_raw_turn=0;}

         else{integral_raw_turn+= turn_error;}

         integral_raw_drive = integral_raw_drive > integralpowerlimit_drive ? integralpowerlimit_drive : integral_raw_drive < -integralpowerlimit_drive ? -integralpowerlimit_drive: integral_raw_drive;

         float integral_drive= ki_drive*integral_raw_drive;

         integral_raw_turn= integral_raw_turn > integralpowerlimit_turn ? integral_raw_turn : integral_raw_turn <-integralpowerlimit_turn ? -integral_raw_turn : integral_raw_turn;

         float integral_turn= ki_turn*integral_raw_turn;

         float derivada_drive = kd_drive * (drive_error - last_error_drive);
         last_error_drive=drive_error;

         float derivada_turn= kd_turn * (turn_error - last_error_turn);
         last_error_turn= turn_error;

         float finalpower_drive= (ceil(drive_proporcion+integral_drive+derivada_drive));
         float finalpower_turn= (ceil(turn_proporcion+integral_turn+derivada_turn));

         finalpower_drive= finalpower_drive > 180 ? 180 : finalpower_drive < -180 ? -180:finalpower_drive;
         finalpower_turn= finalpower_turn > 150 ? 150 : finalpower_turn < -150 ? -150:finalpower_turn;

         //if(fabs(turn_error)<turn_error_range){finalpower_turn=0;}
         //if(fabs(drive_error)<drive_error_range){finalpower_drive=0;}

         Robot::LeftFront_1.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn) ;
         Robot::LeftFront_2.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);

         Robot::LeftBack_1.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);
         Robot::LeftBack_2.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);

         Robot::RightFront_1.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
         Robot::RightFront_2.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
       
         Robot::RightBack_1.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
         Robot::RightBack_2.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
         
         if((fabs(drive_error)<.2 && fabs(turn_error) <2)|| contador>=tiempo){condicion_odometria=true;}

         std::cout<<"\nTiempo: "<<contador;
         std::cout<<"\tD Error: "<<drive_error;
         std::cout<<"\tT Error: "<<turn_error;

         pros::delay(10);
         contador+=10;
    }
        
        Robot::brake("stop");
             
        pros::delay(10);
    
}



 void Robot::controlador_chassis(std::vector<double> posicion, double kp_drive, double ki_drive, double kd_drive,double tiempo){
     double X= posicion[0];
     double Y=posicion[1];

     float integral_raw_drive=0;
     float last_error_drive=0;

     float zonaintegralactiva_drive= (hypot(X-absGlobalX,Y-absGlobalY))*.45;
     float integralpowerlimit_drive= 50/ki_drive;

     bool condicion_odometria=false;
     
     int contador=0;
     tiempo*=1000;

     std::cout<<"\nZona Integral Activa: "<<zonaintegralactiva_drive<<std::endl;
     std::cout<<"\tIntegralPowerLimit: "<<integralpowerlimit_drive<<std::endl;

     while(condicion_odometria==false){

         float drive_error= hypot(X-absGlobalX,Y-absGlobalY); 

         float drive_proporcion= drive_error * kp_drive;
         
         //Corregido
         if(fabs(drive_error)>zonaintegralactiva_drive && drive_error!=0){integral_raw_drive=0;}

         else{integral_raw_drive+= drive_error;}
     
         integral_raw_drive = integral_raw_drive > integralpowerlimit_drive ? integralpowerlimit_drive : integral_raw_drive < -integralpowerlimit_drive ? -integralpowerlimit_drive: integral_raw_drive;

         float integral_drive= ki_drive*integral_raw_drive;

         float derivada_drive = kd_drive * (drive_error - last_error_drive);
         last_error_drive=drive_error;
 
         float finalpower_drive= (ceil(drive_proporcion+integral_drive+derivada_drive));
    
         finalpower_drive= finalpower_drive > 180 ? 180 : finalpower_drive < -180 ? -180:finalpower_drive;

         Robot::LeftFront_1.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))) ;
         Robot::LeftFront_2.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4)));

         Robot::LeftBack_1.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4)));
         Robot::LeftBack_2.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4)));

         Robot::RightFront_1.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4)));
         Robot::RightFront_2.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4)));
       
         Robot::RightBack_1.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4)));
         Robot::RightBack_2.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4)));
         
        if(fabs(drive_error)<.2 || contador>=tiempo){condicion_odometria=true;}

        std::cout<<"\nTiempo: "<<"\t"<<contador<<"\t";
        std::cout<<"\tX: "<<"\t"<<absGlobalX<<"\t";
        std::cout<<"\t Y: "<<"\t"<<absGlobalY<<"\t";
        std::cout<<"\tError: "<<"\t"<<drive_error<<"\t";
        std::cout<<"\tP: "<<"\t"<<drive_proporcion<<"\t";
        std::cout<<"\tI: "<<"\t"<<integral_drive<<"\t";
        std::cout<<"\tD: "<<"\t"<<derivada_drive<<"\t";
        std::cout<<"\tPower: "<<"\t"<<finalpower_drive<<"\t";
    
        pros::delay(10);
        contador+=10;
    }

        Robot::brake("stop");
             
        pros::delay(10);
 }


 void Robot::controlador_giro(double Orientacion,double kp_turn,double ki_turn,double kd_turn,float tiempo){

     Orientacion= reducir_angulo_180_180(Orientacion);

     tiempo*=1000;

     float integral_raw_turn=0;
     float last_error_turn=0;

     float zonaintegralactiva_turn= Orientacion * .3;
     float integralpowerlimit_turn= 50/ki_turn;

     int contador=0;
     std::cout<<"\nZonaIntegralActiva"<<zonaintegralactiva_turn;
     std::cout<<"\tIntegralPowerLimit"<<integralpowerlimit_turn;
     
     bool condicion_odometria=false;

     while(condicion_odometria==false){

         float turn_error= reducir_angulo_180_180(Orientacion- absOrientacionDeg);

         float turn_proporcion= turn_error * kp_turn;


         if(fabs(turn_error)>zonaintegralactiva_turn && turn_error!=0){integral_raw_turn=0;}

         else{integral_raw_turn+= turn_error;}

        
         integral_raw_turn= integral_raw_turn > integralpowerlimit_turn ? integral_raw_turn : integral_raw_turn <-integralpowerlimit_turn ? -integral_raw_turn : integral_raw_turn;

         float integral_turn= ki_turn*integral_raw_turn;

         float derivada_turn= kd_turn * (turn_error - last_error_turn);
         last_error_turn= turn_error;

         float finalpower_turn= (ceil(turn_proporcion+integral_turn+derivada_turn));

         finalpower_turn= finalpower_turn > 150 ? 150 : finalpower_turn < -150 ? -150:finalpower_turn;

         if(fabs(turn_error)<2){finalpower_turn=0;}

         Robot::LeftFront_1.move_velocity(finalpower_turn) ;
         Robot::LeftFront_2.move_velocity(finalpower_turn);

         Robot::LeftBack_1.move_velocity(finalpower_turn);
         Robot::LeftBack_2.move_velocity(finalpower_turn);

         Robot::RightFront_1.move_velocity(-finalpower_turn);
         Robot::RightFront_2.move_velocity(-finalpower_turn);
       
         Robot::RightBack_1.move_velocity(-finalpower_turn);
         Robot::RightBack_2.move_velocity(-finalpower_turn);
         
        
         if(fabs(turn_error) < 2 || contador>=tiempo){condicion_odometria=true;}

        std::cout<<"\nTiempo: "<<"\t"<<contador<<"\t";
        std::cout<<"\tHeading: "<<"\t"<<absOrientacionDeg<<"\t"; 
        std::cout<<"\tError: "<<"\t"<<turn_error<<"\t";
        std::cout<<"\tP: "<<"\t"<<turn_proporcion<<"\t";
        std::cout<<"\tI: "<<"\t"<<integral_turn<<"\t";
        std::cout<<"\tD: "<<"\t"<<derivada_turn<<"\t";
        std::cout<<"\tPower: "<<"\t"<<finalpower_turn<<"\t";

        contador+=10;


         pros::delay(10);
    }

        Robot::brake("stop");
        pros::delay(10);
        
 }




/*
 void Robot::move_to_pure_pursuit(std::vector<std::vector<double>> points, std::vector<double> final_point, int lookAheadDistance){
    
    std::vector<double> end; 
    std::vector<double> start;
    std::vector<double> target;
    std::vector<double>  current {(float)absGlobalX, (float)absGlobalY};
    double Orientacion;

    for(int index=0; index < points.size()-1; index++){
        start= points[index];
        end= points[index+1];

        while (distance(current,end)>lookAheadDis) {
            target= get_intersection(start, end, current, lookAheadDis);
            Orientacion= get_degrees(target, current);

            std::vector<double> Posicion {target[0],target[1],Orientacion};
            move_to(Posicion,  8, 0.037, 60,1.56, .015, 15, 4);
            current={(float)absGlobalX, (float)absGlobalY};
            pros::delay(10);
        }
    }
            move_to(final_point,  8, 0.037, 60,1.56, .015, 15,4);
 }

*/
  

 //Funcion para probar el controlador lineal
 void Robot::test_lineal(void){
    Robot::controlador_chassis({0,24},  10, 5.12, .46,2.5);
    Robot::controlador_chassis({24,24},  10, 5.12, .46,2.5);
    Robot::controlador_chassis({24,0},  10, 5.12, .46,2.5);
    Robot::controlador_chassis({0,0},  10, 5.12, .46,2.5);
 }
 //Funcion para probar el controlador rotacional
 void Robot::test_giro(void){
    Robot::controlador_giro(90, 1.25, 0, .4, 1.5); 
	Robot::controlador_giro(180, 1.25, 0, .4, 1.5);
	Robot::controlador_giro(90, 1.25, 0, .4, 1.5);
	Robot::controlador_giro(0, 1.25, 0, .4, 1.5);
	Robot::controlador_giro(270, 1.25, 0, .4, 1.5);
	Robot::controlador_giro(180, 1.25, 0, .4, 1.5);
	Robot::controlador_giro(0, 1.25, 0, .4, 1.5);
 }
 //Funcion para probar el controlador completo
 void Robot::test_odom(void){
    Robot::move_to({0,24,180}, 10, 5.12, .46, 1.25, 0, .4,2.5);
	Robot::move_to({24,24,180}, 10, 5.12, .46, 1.25, 0, .4,2.5);
	Robot::move_to({24,0,0}, 10, 5.12, .46, 1.25, 0, .4,2.5);
	Robot::move_to({0,0,0}, 10, 5.12, .46, 1.25, 0, .4,2.5);
 } 

void Robot::drive(void*ptr){
    int power_flywheel=0;

    bool state_piston=false; 
    while(true){
        //Chassis 
        double y= master.get_analog(ANALOG_LEFT_Y);
        double turn=master.get_analog(ANALOG_RIGHT_X);
        double x=master.get_analog(ANALOG_LEFT_X);
        double theta=atan2(y,x);
        double power=hypot(x,y);
        double inercia_rad = TO_RAD(gyro.get_heading());
                                                 
                                            // + inercia_Rad
        double strafe_lf_rb=cos(theta - M_PI/4 + inercia_rad); 
        double strafe_rf_lb=sin(theta - M_PI/4 + inercia_rad);

        x_drive(power,strafe_lf_rb,strafe_rf_lb,turn);


        //Flywheel
        power_flywheel= master.get_digital(DIGITAL_R2)==1 ? 12000 : master.get_digital(DIGITAL_R1)==1 ? -12000 :0; 
        Fly_wheel_action(power_flywheel);

        state_piston= master.get_digital_new_press(DIGITAL_A) ? !state_piston : state_piston;
        Piston_movement(state_piston);  

        std::cout<<"\nEstado Piston: "<<state_piston; 
                                

        pros::delay(5);  
    }
}


 void Robot::x_drive(double power, double strafe_lf_rb,double strafe_rf_lb,double turn){
     double max=MAX(abs(strafe_lf_rb),abs(strafe_rf_lb));
     
     LeftFront_1.move(power*(strafe_lf_rb/max)+turn);    RightFront_1.move(power*(strafe_rf_lb/max)-turn);
	 LeftFront_2.move(power*(strafe_lf_rb/max)+turn);    RightFront_2.move(power*(strafe_rf_lb/max)-turn);

	 LeftBack_1.move(power*(strafe_rf_lb/max)+turn);	 RightBack_1.move(power*(strafe_lf_rb/max)-turn);
	 LeftBack_2.move(power*(strafe_rf_lb/max)+turn);     RightBack_2.move(power*(strafe_lf_rb/max)-turn);
 }

 void Robot::Fly_wheel_action(int power){
    FlyWheel_1.move_voltage(power); 
    Flywheel_2.move_voltage(power);
 }

 void Robot::Piston_movement(bool state){
    piston.set_value(state); 
 } 


 
 void Robot::brake(std::string mode){
    if (mode.compare("coast") == 0)
	{
		LeftFront_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        LeftFront_2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        LeftBack_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        LeftBack_2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        RightFront_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        RightFront_2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        RightBack_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        RightBack_2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	}

    else if (mode.compare("hold")==0) {
        LeftFront_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        LeftFront_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        LeftBack_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        LeftBack_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        RightFront_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        RightFront_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        RightBack_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        RightBack_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    else{
        LeftFront_1 = LeftFront_2 = LeftBack_1 = LeftBack_2 = RightFront_1 = RightFront_2 = RightBack_1 = RightBack_2 = 0;
    }
   

 }


 void Robot::reset_sensors(){
     gyro.reset();
     Encoder_Derecho.reset();
     Encoder_back.reset();
 }

