#include "PID.hpp"
#include "Purepursuit.h"
#include "auxiliar_fuctions.h"
#include "display/lv_hal/lv_hal_indev.h"
#include "filtros.h"
#include "main.h"

#include "odometria.h"
#include "parametros.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
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
#include "okapi/api.hpp"
//Lambda fuction, para convertir de grados a radianes
#define TO_RAD(n) n * M_PI / 180;
#define END_GAME 1 


#define PORT_LEFT_FRONT_1 3 //r
#define PORT_LEFT_FRONT_2 2 //r 



#define PORT_LEFT_BACK_1 11 //r
#define PORT_LEFT_BACK_2 12 //t

#define PORT_RIGHT_FRONT_1 7 //r
#define PORT_RIGHT_FRONT_2 8 //r

#define PORT_RIGHT_BACK_1 10 //r
#define PORT_RIGHT_BACK_2 9  //r



#define PORT_FLYWHEEL_1 16 //r
#define PORT_FLYWHEEL_2 1  //r

#define PORT_INTAKER_1 19 //r
#define PORT_INTAKER_2 18  //r

#define PORT_INDEXER 13 //r

#define PORT_EXPANSION 20 //r


#define PORT_GYRO 14 //r 

#define PORT_ROTATION 17 //r


pros::Motor Robot::LeftFront_1(PORT_LEFT_FRONT_1,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::LeftFront_2(PORT_LEFT_FRONT_2,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::LeftBack_1(PORT_LEFT_BACK_1,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::LeftBack_2(PORT_LEFT_BACK_2,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::RightFront_1(PORT_RIGHT_FRONT_1,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::RightFront_2(PORT_RIGHT_FRONT_2,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);


pros::Motor Robot::RightBack_1(PORT_RIGHT_BACK_1,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::RightBack_2(PORT_RIGHT_BACK_2,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::FlyWheel_1(PORT_FLYWHEEL_1,pros::E_MOTOR_GEARSET_06,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::Flywheel_2(PORT_FLYWHEEL_2,pros::E_MOTOR_GEARSET_06,true,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::intaker_1(PORT_INTAKER_1,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::intaker_2(PORT_INTAKER_2,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::Indexer(PORT_INDEXER, pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::Expansion_ (PORT_EXPANSION,pros::E_MOTOR_GEARSET_36,true,pros::E_MOTOR_ENCODER_DEGREES);  



pros::Imu Robot::gyro(PORT_GYRO);

pros::Rotation Robot::Rotacion(PORT_ROTATION);

pros::ADIEncoder Robot::Encoder_Derecho('C','D',true);

pros::ADIEncoder Robot::Encoder_back('A','B',true);

pros::ADIMotor Robot::Green_led('E');
pros::ADIMotor Robot::Blue_led('F');  


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
   
    if (task_exists(name)==false) {
		tasks.erase(name);
	}
}
 
double Robot::Distancia_Encoder_Y=.01; 
double Robot::Distancia_Encoder_X=.01;
  
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

double Robot::High_GoalX=-100;
double Robot::High_GoalY=47.5;

int Robot::RPM=0; 

int Last_RPM=0; 

std::atomic<bool> Robot::Ready_to_shoot=false;

std::atomic<double> Robot::turn_drift=0;

void Robot::rastreo(void *ptr){
    while(true){
        updateEncoders();
        updatePosicion();
        pros::delay(10);
    }
}

void Robot::track_orientation(void *ptr){
    while (true) {
        absOrientacionDeg= gyro.get_heading();
        absOrientacionRad = TO_RAD(absOrientacionDeg);
        pros::delay(10);   
    }
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
    /*Almacenamiento de datos sobre orientación y ángulo del robot, en radianes y Grados*/
    absOrientacionDeg= gyro.get_heading();


    absOrientacionRad= TO_RAD(absOrientacionDeg);
    
    //float angle_gyro_Rad= TO_RAD(gyro.get_heading());
    float delta_A= absOrientacionRad-prev_A;
    prev_A=absOrientacionRad;

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
   // std::cout<<"\tCoordenada Y \t"<<Robot::absGlobalY;

  //  std::cout<<"\tAngulo \t"<<Robot::absOrientacionDeg;
}

float Robot::clean_readings(okapi::EKFFilter KALMAN){
    float velocidad=0; 
    KALMAN.filter(0);
    for(int i= 0 ; i<50 ; i++){
        velocidad= KALMAN.filter(0); 
        pros::delay(10); 
    }
    
    return velocidad;
}


void Robot::Move_to_point(double(*fuctPtr_Mode)(double,double,double),double potencia,std::vector<double> posicion, std::vector<double>DrivePID, std::vector<double>TurnPID, double tiempo_sec, float TargetX, float TargetY,bool pure_pursuit){
     
     /*
     Calculos necesarios para despues continuar con el ciclo de PID
     Debido a que son dos compensadores, se realizan los calculos necesarios 
     Para cada uno de ellos
     */

     double X= posicion[0];
     double Y=posicion[1];
     double Orientacion= posicion[2];

     double strafe_lf_rb_odom=0;  
     double strafe_rf_lb_odom=0; 

     DRIVE.kp=DrivePID[0];
     DRIVE.ki=DrivePID[1];
     DRIVE.kd=DrivePID[2];

     TURN.kp=TurnPID[0];
     TURN.ki=TurnPID[1];
     TURN.kd=TurnPID[2];

     Orientacion= reducir_angulo_0_360(Orientacion);
     
     DRIVE.integral_raw=0;
     DRIVE.last_error=0;

     DRIVE.zonaintegralactiva= (hypot(X-absGlobalX,Y-absGlobalY))*.45;
     DRIVE.integralpowerlimit= 50/DRIVE.ki;

     TURN.integral_raw=0;
     TURN.last_error=0;
    
     TURN.zonaintegralactiva= Orientacion * .3;
     TURN.integralpowerlimit= 50/TURN.ki;
     
     DRIVE.maximo=180;  
     TURN.maximo=150; 

     bool condicion_odometria=false;
     
     tiempo_sec*=1000;
     int contador=0;

     while(condicion_odometria==false){
        //Aqui entra el puntero como parametro y cambia el valor de orientacion, dependiendo de cual puntero se haya elegido
        // si se escogio facing to -> el robot apuntará a un punto
        // si se escogio move to -> el robot no apuntará a ningun punto, en cambio tendra una orientacion predefinida
        Orientacion=fuctPtr_Mode(Orientacion,TargetX,TargetY);
       
        TURN.zonaintegralactiva= Orientacion*.3;


        DRIVE.error= hypot(X-absGlobalX,Y-absGlobalY); 
        TURN.error= reducir_angulo_180_180(Orientacion- absOrientacionDeg);

        DRIVE.proporcion= DRIVE.error * DRIVE.kp;
        TURN.proporcion= TURN.error * TURN.kp;

        if(fabs(DRIVE.error)>DRIVE.zonaintegralactiva && DRIVE.error!=0){DRIVE.integral_raw=0;}

        else{DRIVE.integral_raw+= DRIVE.error;}

        if(fabs(TURN.error)>TURN.zonaintegralactiva && TURN.error!=0){TURN.integral_raw=0;}

        else{TURN.integral_raw+= TURN.error;}

        DRIVE.integral_raw = DRIVE.integral_raw > DRIVE.integralpowerlimit ? DRIVE.integralpowerlimit : DRIVE.integral_raw < -DRIVE.integralpowerlimit ? -DRIVE.integralpowerlimit: DRIVE.integral_raw;

        DRIVE.integral= DRIVE.ki*DRIVE.integral_raw;

        TURN.integral_raw= TURN.integral_raw > TURN.integralpowerlimit ? TURN.integral_raw : TURN.integral_raw <-TURN.integralpowerlimit ? -TURN.integral_raw : TURN.integral_raw;

        TURN.integral= TURN.ki*TURN.integral_raw;

        DRIVE.derivada = DRIVE.kd* (DRIVE.error - DRIVE.last_error);
        DRIVE.last_error=DRIVE.error;

        TURN.derivada= TURN.kd * (TURN.error - TURN.last_error);
        TURN.last_error= TURN.error;

        DRIVE.finalpower= (ceil(DRIVE.proporcion+DRIVE.integral+DRIVE.derivada));
        TURN.finalpower= (ceil(TURN.proporcion+TURN.integral+TURN.derivada));

        DRIVE.finalpower= DRIVE.finalpower > 180 ? 180 : DRIVE.finalpower < -180 ? -180:DRIVE.finalpower;
        TURN.finalpower= TURN.finalpower > 150 ? 150 : TURN.finalpower < -150 ? -150:TURN.finalpower;

        DRIVE.finalpower *= potencia;
        TURN.finalpower *= potencia;
        
        strafe_lf_rb_odom=cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - M_PI/4);  
        strafe_rf_lb_odom=sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - M_PI/4); 
        
        std::cout<<"\nTarget: \t24"; 
        std::cout<<"\tActual:  \t"<<absGlobalY; 

        x_drive(DRIVE.finalpower, strafe_lf_rb_odom, strafe_rf_lb_odom, TURN.finalpower, "AUTO");
         
        if((fabs(DRIVE.error)<.2  && fabs(TURN.error) <2 )|| contador>=tiempo_sec){condicion_odometria=true;}

        pros::delay(10);
        contador+=10;
    }
    
    if(pure_pursuit==false){Robot::brake("stop");} 

  
   
    pros::delay(10);

}

void Robot::Turning(double(*fuctPtr_Mode)(double,double,double),float Orientacion,float potencia ,std::vector<double> TurnPID,double tiempo_sec,float TargetX, float TargetY, float Turn_offset){

    TURN.kp=TurnPID[0];
    TURN.ki=TurnPID[1];
    TURN.kd=TurnPID[2];

    Orientacion= reducir_angulo_0_360(Orientacion)-Turn_offset;

    TURN.integral_raw=0;
    TURN.last_error=0;

    TURN.zonaintegralactiva= Orientacion *.3;
    TURN.integralpowerlimit= 50/TURN.ki;

    TURN.maximo=150; 

    bool condicion=false;
     
    tiempo_sec*=1000;
    int contador=0;

    std::cout<<"\nZona integral "<<TURN.zonaintegralactiva;

    while(condicion==false){ 

        Orientacion = fuctPtr_Mode(Orientacion , TargetX , TargetY) -Turn_offset ; 
              
        TURN.error= reducir_angulo_180_180(Orientacion- absOrientacionDeg);
       
        TURN.proporcion= TURN.error * TURN.kp;

        if(fabs(TURN.error)>TURN.zonaintegralactiva && TURN.error!=0){TURN.integral_raw=0;}

        else{TURN.integral_raw+= TURN.error;}

        TURN.integral_raw= TURN.integral_raw > TURN.integralpowerlimit ? TURN.integral_raw : TURN.integral_raw <-TURN.integralpowerlimit ? -TURN.integral_raw : TURN.integral_raw;

        TURN.integral= TURN.ki*TURN.integral_raw;

        TURN.derivada= TURN.kd * (TURN.error - TURN.last_error);
        TURN.last_error= TURN.error;
       
        TURN.finalpower= (ceil(TURN.proporcion+TURN.integral+TURN.derivada));

        TURN.finalpower= TURN.finalpower > 150 ? 150 : TURN.finalpower < -150 ? -150:TURN.finalpower;

        std::cout<<"\nTarget: \t"<<Orientacion; 
        std::cout<<"\tCurrent: \t"<<absOrientacionDeg;
        std::cout<<"\tP: \t"<<TURN.kp;
        std::cout<<"\tI: \t"<<TURN.ki;
        std::cout<<"\tD: \t"<<TURN.kd;
   

    
        Robot::LeftFront_1.move_velocity(TURN.finalpower) ;
        Robot::LeftFront_2.move_velocity(TURN.finalpower);

        Robot::LeftBack_1.move_velocity(TURN.finalpower);
        Robot::LeftBack_2.move_velocity(TURN.finalpower);

        Robot::RightFront_1.move_velocity(-TURN.finalpower);
        Robot::RightFront_2.move_velocity(-TURN.finalpower);
       
        Robot::RightBack_1.move_velocity(-TURN.finalpower);
        Robot::RightBack_2.move_velocity(-TURN.finalpower);
     

        if( fabs(TURN.error) <1 || contador>=tiempo_sec){condicion=true;}


        pros::delay(10);
        contador+=10;

    }

    Robot::brake("stop");   
    
    pros::delay(10);

}


void Robot::tune_pid(float tiempo_sec, float step_percent, pros::motor_gearset_e Gearset,std::vector<pros::Motor> Motores, std::string Entrada){

    step_percent /=100; 

    int RPM_motor = Gearset==pros::E_MOTOR_GEARSET_36 ? 100 : Gearset ==pros::E_MOTOR_GEARSET_18 ? 200 : Gearset ==pros::E_MOTOR_GEARSET_06 ?600 : 0;
    
    step_percent = Entrada.compare("VELOCIDAD") == 0 ? step_percent*RPM_motor : Entrada.compare("VOLTAJE") ==0 ? step_percent*12000 : 0; 
 
    int contador= 0; 
    tiempo_sec *=1000; 

    int Salida=0;
    int Salida_promedio=0;
    
    std::cout<<"\nStep: "<<step_percent; 

    bool inicio=true; 
    while(contador <tiempo_sec){
           
       if(inicio){
        for(int i=0; i<=Motores.size()-1; i++){
            if(Entrada.compare("VELOCIDAD")==0){Motores[i].move_velocity(step_percent);}

            if (Entrada.compare("VOLTAJE")==0){Motores[i].move_voltage(step_percent);}
    
            pros::delay(10);
        
        inicio=false;
       }
       }

       for(int i=0; i<=Motores.size()-1; i++){
        Salida = Entrada.compare("VELOCIDAD")==0 ? Salida= Motores[i].get_actual_velocity() : Entrada.compare("VOLTAJE")==0 ? Salida= Motores[i].get_voltage() : 0;
       }

       Salida_promedio = Salida;
    
       std::cout<<"\nTiempo: \t"<<contador; 
       std::cout<<"\tSalida: \t"<< Salida_promedio; 
        
       pros::delay(10); 
       contador+=10;
    
    }
} 


void Robot::follow_path(double(*fuctPtr_mode)(double,double,double), std::vector<std::vector<double>> Path, double Orientacion,double lookAheadDistance, int lastFoundIndex){
     
    for(auto i=0; i<Path.size(); i++){
        std::vector<double> target_point(3); 

        target_point = goal_search( absGlobalX,absGlobalY , Path, lookAheadDistance, lastFoundIndex, i); 
        
        target_point[2] = Orientacion; 

        //Romper ciclo solo si la solucion encontrada está más cerca del siguiente punto que la posicion actual
        if ( solution_backwards(absGlobalX, absGlobalY, Path, target_point, i)) {lastFoundIndex = i; break; }

        //De lo contrario aumentar index y seguir avanzando
        else {lastFoundIndex = i + 1;}
        
        Move_to_point(fuctPtr_mode, 1, {0}, Drive_Constant, Turn_Constant, 1.5, target_point[0], target_point[1], true); 
    }
    
    Robot::brake("stop"); 
}


void Robot::Flywheel_PID_a (void*ptr){
  
    FLYWHEEL.kp = Flywheel_Constant[0];
    FLYWHEEL.ki = Flywheel_Constant[1];
    FLYWHEEL.kd=  Flywheel_Constant[2]; 

    FLYWHEEL.integral_raw=0;
    FLYWHEEL.last_error=0;

    FLYWHEEL.zonaintegralactiva = Robot::RPM* .45;
    FLYWHEEL.integralpowerlimit = 50/FLYWHEEL.ki; 

    int contador=0; 
    
    float prev_velocidad=0; 

    float velocidad=clean_readings(Fil::kalman);
    float aceleracion = velocidad-prev_velocidad; 
    prev_velocidad = velocidad; 

    FLYWHEEL.maximo=12000; 
    
    float feed_forward =0;
    
    //2.23
    float kv= 2.12;
    int Ks= 1500;
    //3.25 
    float ka = 2.25; 

    float V=  Ks*sign(velocidad)+ kv* velocidad + ka*aceleracion;
    
    pros::delay(10);
    
    while(1){
    
        FLYWHEEL.zonaintegralactiva = RPM* .45;

        velocidad = Fil::kalman.filter(Rotacion.get_velocity()/6)  ;
        aceleracion= (velocidad - prev_velocidad); 
        prev_velocidad=velocidad; 

        //calculo del error
        FLYWHEEL.error = RPM - velocidad;
 
        FLYWHEEL.proporcion = FLYWHEEL.kp * FLYWHEEL.error;

        if (fabs (FLYWHEEL.error) > FLYWHEEL.zonaintegralactiva && FLYWHEEL.error !=0){FLYWHEEL.integral_raw=0;}

        else{FLYWHEEL.integral_raw += FLYWHEEL.error;}

        FLYWHEEL.integral_raw = FLYWHEEL.integral_raw > FLYWHEEL.integralpowerlimit ? FLYWHEEL.integralpowerlimit : FLYWHEEL.integral_raw < -FLYWHEEL.integralpowerlimit ? -FLYWHEEL.integralpowerlimit : FLYWHEEL.integral_raw;
        FLYWHEEL.integral = FLYWHEEL.ki * FLYWHEEL.integral_raw; 

        FLYWHEEL.derivada = FLYWHEEL.kd*(FLYWHEEL.error - FLYWHEEL.last_error);
        FLYWHEEL.last_error = FLYWHEEL.error; 
           
        V=  Ks*sign(velocidad)+ kv* velocidad +ka*aceleracion ;

        FLYWHEEL.finalpower = ceil (FLYWHEEL.proporcion + FLYWHEEL.integral + FLYWHEEL.derivada+V);
        FLYWHEEL.finalpower = FLYWHEEL.finalpower > FLYWHEEL.maximo ? FLYWHEEL.maximo: FLYWHEEL.finalpower;
    
        Robot::move_Flywheel(FLYWHEEL.finalpower);

        if (!(abs(FLYWHEEL.error)   <50)) {
            Ready_to_shoot=false;
        }

        else {
            Ready_to_shoot=true; 
        }
        
        if(RPM==0){
            Robot::move_Flywheel(0);
        }
        
        std::cout<<"\n Tiempo: \t"<<contador; 
        std::cout<<"\t Target: \t"<<RPM; 
        std::cout<<"\t Current: \t"<<velocidad; 
        std::cout<<"\t Error: "<<FLYWHEEL.error;
        std::cout<<"\t FeedForward: "<<V;    
        std::cout<<"\tP: "<<FLYWHEEL.proporcion; 
        std::cout<<"\tI: "<<FLYWHEEL.integral;  
        std::cout<<"\tD: "<<FLYWHEEL.derivada;
        std::cout<<"\tPower: "<<FLYWHEEL.finalpower;
           
        pros::delay(10);
        contador+=10;   
    } 
    
    FlyWheel_1.move_voltage(0);
    Flywheel_2.move_voltage(0);
}


void Robot::Flywheel_PID_motor (void*ptr){
    
    FLYWHEEL.kp = Flywheel_Constant[0];
    FLYWHEEL.ki = Flywheel_Constant[1];
    FLYWHEEL.kd=  Flywheel_Constant[2]; 

    FLYWHEEL.integral_raw=0;
    FLYWHEEL.last_error=0;

    FLYWHEEL.zonaintegralactiva = Robot::RPM* .45;
    FLYWHEEL.integralpowerlimit = 50/FLYWHEEL.ki; 

    int contador=0; 
    
    float prev_velocidad=0; 

    float velocidad=clean_readings(Fil::kalman);
    float aceleracion = velocidad-prev_velocidad; 
    prev_velocidad = velocidad; 

    FLYWHEEL.maximo=12000; 
    
    float feed_forward =0;
    
    
    float kv= 15.55;
    int Ks= 2500; 
    float ka =5; 
    float V=  Ks*sign(velocidad)+ kv* velocidad + ka*aceleracion;
    
    pros::delay(10);
    
    while(1){
    
        FLYWHEEL.zonaintegralactiva = RPM* .45;
        velocidad = Fil::kalman.filter(FlyWheel_1.get_actual_velocity()) ; 
        aceleracion= (velocidad - prev_velocidad); 
        prev_velocidad=velocidad; 

        //calculo del error
        FLYWHEEL.error = RPM - velocidad;
 
        FLYWHEEL.proporcion = FLYWHEEL.kp * FLYWHEEL.error;

        if (fabs (FLYWHEEL.error) > FLYWHEEL.zonaintegralactiva && FLYWHEEL.error !=0){FLYWHEEL.integral_raw=0;}

        else{FLYWHEEL.integral_raw += FLYWHEEL.error;}

        FLYWHEEL.integral_raw = FLYWHEEL.integral_raw > FLYWHEEL.integralpowerlimit ? FLYWHEEL.integralpowerlimit : FLYWHEEL.integral_raw < -FLYWHEEL.integralpowerlimit ? -FLYWHEEL.integralpowerlimit : FLYWHEEL.integral_raw;
        FLYWHEEL.integral = FLYWHEEL.ki * FLYWHEEL.integral_raw; 

        FLYWHEEL.derivada = FLYWHEEL.kd*(FLYWHEEL.error - FLYWHEEL.last_error);
        FLYWHEEL.last_error = FLYWHEEL.error; 
           
        V=  Ks*sign(velocidad)+ kv* velocidad +ka*aceleracion ;

        FLYWHEEL.finalpower = ceil (FLYWHEEL.proporcion + FLYWHEEL.integral + FLYWHEEL.derivada+V);
        FLYWHEEL.finalpower = FLYWHEEL.finalpower > FLYWHEEL.maximo ? FLYWHEEL.maximo: FLYWHEEL.finalpower;
    
        Robot::move_Flywheel(FLYWHEEL.finalpower);

        if (!(abs(FLYWHEEL.error)   <50)) {
            Ready_to_shoot=false;
        }

        else {
            Ready_to_shoot=true; 
        }
        
        if(RPM==0){
            Robot::move_Flywheel(0);
        }
        
        std::cout<<"\n Tiempo: \t"<<contador; 
        std::cout<<"\t Target: \t"<<RPM; 
        std::cout<<"\t Current: \t"<<velocidad; 
        //std::cout<<"\t Error: "<<FLYWHEEL.error;
        std::cout<<"\t FeedForward: \t"<<V;    
        std::cout<<"\tP: \t"<<FLYWHEEL.proporcion; 
        std::cout<<"\tI: \t"<<FLYWHEEL.integral;  
        std::cout<<"\tD: \t"<<FLYWHEEL.derivada;
        std::cout<<"\tPower: \t"<<FLYWHEEL.finalpower;
           
        pros::delay(10);
        contador+=10;   
    } 
}

void Robot::Shoot_PID(int disparos,std::string mode,int delay_msec){
  int RPM_SHOOT = mode.compare("LONG")==0 ? LOW_VELOCITY_SHOOT : mode.compare("MEDIUM")==0 ? MEDIUM_VELOCITY_SHOOT : mode.compare("SHORT") ==0 ? HIGH_VELOCITY_SHOOT : 0;  
  int n_disparo=0;
  bool paro=false;

  while (paro==false) {

    if(Ready_to_shoot==true){
        Indexer.tare_position(); 
        Move_Indexer_pos(INDEXER_SHOOT_POSITION, RPM_SHOOT);
        n_disparo++; 
        pros::delay(delay_msec);  
    }

    paro = n_disparo>= disparos ? true : false; 

  } 



}

void Robot::Shoot_normal(int velocity_RPM){
    Indexer.tare_position(); 
    Move_Indexer_pos(INDEXER_SHOOT_POSITION,velocity_RPM); 
} 

void Robot::eat (int RPM){
    intaker_1.move_velocity(RPM);  
    intaker_2.move_velocity(RPM); 
} 

void  Robot::eat_voltaje (int mv){
        intaker_1.move_voltage(mv);  
    intaker_2.move_voltage(mv); 
}

void Robot::Move_Roller_pos(float Position, int RPM){
    intaker_1.tare_position();
    intaker_2.tare_position();

    intaker_1.move_absolute(Position, RPM);
    intaker_2.move_absolute(Position, RPM);
} 

void Robot::Move_Indexer_pos(float Position, int RPM){
    Indexer.move_absolute(Position, RPM);
}


void Robot::Turn_lights(int mode){
    mode = mode>MAX_LIGHT_CONFIGURATION ? MAX_LIGHT_CONFIGURATION :mode; 
    
    switch (mode) {
    case LOW_DISTANCE_LIGHT: Lights(255, 255 ); break;
    case MEDIUM_DISTANCE_LIGHT: Lights(255, 0); break;
    case HIGH_DISTANCE_LIGHT:  Lights(0, 255); break;
    default: Lights(0, 0); break; 
    }

}


void Robot::Lights(int _green, int _blue){
    Green_led.set_value(_green/2); 
    Blue_led.set_value(_blue/2); 
}

bool End_game_time(int time){
    return time >= 60000; 
}

void Robot::Release_expansion(bool state){
    
    state = state!=KEEP_EXPANSION ? THROW_EXPANSION : KEEP_EXPANSION ;

    int Position = state==THROW_EXPANSION ? EXPANSION_POSITION : -5; 
    
    //Detener el flywheel en caso de que se active la expansion para evitar
    //Que el hilo pueda atorarse. 
    RPM = state!= KEEP_EXPANSION ? 0 : RPM;
    
    Expansion_.move_absolute(Position, 100);
    
} 


int joystick_sensitivity (int joystick_value, int n){
    
    return pow(joystick_value,n) / pow(127, n-1);  
}

void Robot::drive(void*ptr){
    int contador=0; 
    int power_flywheel=0;
    int power_indexer=0;
    int power_intake=0; 

    int indexer_velocity=0;

    int shoot_configuration=0; 
    int light_configuration = shoot_configuration; 

    bool state_flywheel=false; 
    bool state_indexer=false;
    
    float potencia_giro=1; 

    bool expansiooon=KEEP_EXPANSION; 
    bool avisar=true; 

    while(true){
        
        int y= master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); 
        int turn=master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * potencia_giro; 
        //int turn = RPM !=STOP_FLYWHEEL_VELOCITY ? joystick_sensitivity(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 3) :joystick_sensitivity(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 1); 
        int x=master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X); 
        
        potencia_giro = RPM !=STOP_FLYWHEEL_VELOCITY ? .50: 1; 
        double theta=atan2(y,x);
        double power=hypot(x,y);
     
        //double inercia_rad = TO_RAD(gyro.get_heading()); 
                                         
        double strafe_lf_rb=cos(theta - M_PI/4 + absOrientacionRad);
        double strafe_rf_lb=sin(theta - M_PI/4 + absOrientacionRad);
        
        x_drive(power,strafe_lf_rb,strafe_rf_lb,turn+turn_drift,"JOYSTICK");

        shoot_configuration = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)==1 ? shoot_configuration+=1: shoot_configuration>1 ? 0 : shoot_configuration;
        
        light_configuration=shoot_configuration; 
        Turn_lights(light_configuration);

        RPM = shoot_configuration==1 ? SHORT_FLYWHEEL_VELOCITY: shoot_configuration==0 ||expansiooon==THROW_EXPANSION ? STOP_FLYWHEEL_VELOCITY :  RPM;  
        
    

        indexer_velocity = shoot_configuration==1 ? HIGH_VELOCITY_SHOOT: 0  ;

        state_indexer = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)==1 ? true :false;
        set_Indexer(state_indexer, indexer_velocity,"DRIVE");    
        

        expansiooon= master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)==1 ? THROW_EXPANSION  && End_game_time(contador) == END_GAME: expansiooon; 
        Release_expansion(expansiooon); 
             
        power_intake=
        master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)==1 ? DRIVER_INTAKER_VELOCITY : master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)==1 ? -DRIVER_INTAKER_VELOCITY:0;
        
        move_Intake(power_intake);

     

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)==1){
            reset_sensors();
            pros::delay(3000); 
        }

        if(End_game_time(contador)==END_GAME && avisar==true){
            master.rumble(".-.-.-.-.-.-.-.-.-"); 
            avisar=false; 
        }
        
        std::cout<<"\nContador: "<<contador; 
        pros::delay(10);  
        contador+=10; 
    }
}

void Robot::x_drive(double power, double strafe_lf_rb,double strafe_rf_lb,double turn, std::string name){
    //Limitador de velocidad
    double max=std::max(abs(strafe_lf_rb),abs(strafe_rf_lb));

    ////////////Calculo de poder para cada una de las llantas///////////
    
    double power_left_front= power*(strafe_lf_rb/max)+(turn);
    double power_left_back= power*(strafe_rf_lb/max)+(turn);

    double power_righ_front= power*(strafe_rf_lb/max)-(turn);
    double power_righ_back= power*(strafe_lf_rb/max)-(turn);

    //Turn_joystick es la salida de PID_Drift, como este compensador
    //Actua en el modo driver, su valor tiene que estar incluido aqui
    ///////////////////////////////////////////////////////////////////
    
    if(name.compare("JOYSTICK")==0){
    //Finalmente movemos los motores
    LeftFront_1.move(power_left_front);    RightFront_1.move(power_righ_front);
	LeftFront_2.move(power_left_front);    RightFront_2.move(power_righ_front);

	LeftBack_1.move(power_left_back);	     RightBack_1.move(power_righ_back);
	LeftBack_2.move(power_left_back);      RightBack_2.move(power_righ_back);
    }

    else if(name.compare("AUTO")==0){
    LeftFront_1.move_velocity(power_left_front);    RightFront_1.move_velocity(power_righ_front);
	LeftFront_2.move_velocity(power_left_front);    RightFront_2.move_velocity(power_righ_front);

	LeftBack_1.move_velocity(power_left_back);	  RightBack_1.move_velocity(power_righ_back);
	LeftBack_2.move_velocity(power_left_back);      RightBack_2.move_velocity(power_righ_back);
    }

    else {LeftFront_1 = LeftFront_2 = LeftBack_1 = LeftBack_2 = RightFront_1 = RightFront_2 = RightBack_1 = RightBack_2 = 0;}
}

void Robot::move_Flywheel(int power){
    FlyWheel_1.move_voltage(power); 
    Flywheel_2.move_voltage(power);
}

void Robot::move_Intake(int power){
    intaker_1.move_voltage(power);
    intaker_2.move_voltage(power);
}

void Robot::move_Indexer_velocity(int RPM){
    Indexer.move_velocity(RPM); 
} 

void Robot::set_Indexer(bool state, int RPM,std::string mode){

    if(state == false){Indexer.move_absolute(0, 50);}
    
    else{
      
        
        if(mode.compare("AUTO")==0){ Move_Indexer_pos(INDEXER_SHOOT_POSITION, RPM);}

        if(mode.compare("DRIVE")==0){move_Indexer_velocity(RPM);}
    
    }
    
}

void Robot::PID_drift_Cesar(void *ptr){
  
    int target=0;

    TURN.integral_raw=0;
    TURN.last_error=0; 

    TURN.kp = Turn_Constant[0];
    TURN.ki = Turn_Constant[1];
    TURN.kd = Turn_Constant[2];
    
    bool start_pid =false;
    
    //Vector donde vienen las diferentes posiciones a girar
    std::vector<float> Positions {0,90,180,270,360};
    //Vecotr donde se almacenar las distancias entre la orientacion actual y la target
    std::vector<float> Distance {0,0,0,0,0};
    
    //Variables necesarias para encontrar el elemento minimo dentro del vector de distancia
    float min=Distance[0];
    int index_min=0;
    turn_drift=0;

    while(1){
        turn_drift=0; 
        
        if (start_pid==false) {
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) ){
                Distance = Get_Distances(Positions, absOrientacionDeg); 
                index_min = Get_Index_min(Distance);
                target = index_min == 0 ? 0 : index_min == 1 ? 90 : index_min == 2 ? 180 : index_min == 3 ? 270 : index_min ==4 ? 360: target;
                
                pros::delay(10); 
                start_pid=true;
            }
        }
   
        if(start_pid==true){

            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) ){
            Distance = Get_Distances(Positions, absOrientacionDeg); 
            index_min = Get_Index_min(Distance);
         
            target = index_min == 0 ? 0 : index_min == 1 ? 90 : index_min == 2 ? 180 : index_min == 3 ? 270 : target;
            
            }

            //std::cout<<"\n start ?: "<<start_pid;
            //std::cout<<"\t target: "<<target; 
            target = reducir_angulo_180_180(target);
            
            TURN.zonaintegralactiva= target * .3;
            TURN.integralpowerlimit= 50/TURN.ki;
                
            TURN.error= reducir_angulo_180_180(target- absOrientacionDeg);

            TURN.proporcion= TURN.error * TURN.kp;

            if(fabs(TURN.error)>TURN.zonaintegralactiva && TURN.error!=0){TURN.integral_raw=0;}

            else{TURN.integral_raw+= TURN.error;}

            TURN.integral_raw= TURN.integral_raw > TURN.integralpowerlimit ? TURN.integral_raw : TURN.integral_raw <-TURN.integralpowerlimit ? -TURN.integral_raw : TURN.integral_raw;

            TURN.integral= TURN.ki*TURN.integral_raw;

            TURN.derivada= TURN.kd * (TURN.error - TURN.last_error);
            TURN.last_error= TURN.error;
            
            TURN.finalpower= (ceil(TURN.proporcion+TURN.integral+TURN.derivada));

            TURN.finalpower= TURN.finalpower > 100 ? 100 : TURN.finalpower < -100 ? -100:TURN.finalpower;  
    
            if(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 50){
                start_pid=false;
                turn_drift=0;
                TURN.finalpower=0;
            }

            if(abs(TURN.error)<=2){TURN.finalpower=0;}

            turn_drift=TURN.finalpower;

            pros::delay(10);


        }
    }

}

void Robot::PID_drift_Alex_double(void*ptr){
     int target=0;

    TURN.integral_raw=0;
    TURN.last_error=0; 

    TURN.kp = Turn_Constant[0];
    TURN.ki = Turn_Constant[1];
    TURN.kd = Turn_Constant[2];
    
    bool start_pid =false;
    
    //Vector donde vienen las diferentes posiciones a girar
    std::vector<float> Positions {0,90,180,270,360};
    //Vecotr donde se almacenar las distancias entre la orientacion actual y la target
    std::vector<float> Distance {0,0,0,0,0};
    
    //Variables necesarias para encontrar el elemento minimo dentro del vector de distancia
    float min=Distance[0];
    int index_min=0;
    turn_drift=0;

    while(1){
        turn_drift=0; 
        
    
        if (start_pid==false) {
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
                Distance = Get_Distances(Positions, absOrientacionDeg); 
                index_min = Get_Index_min(Distance);
                target = index_min == 0 ? 0 : index_min == 1 ? 90 : index_min == 2 ? 180 : index_min == 3 ? 270 : index_min==4 ? 360: target;
                
                pros::delay(10); 
                start_pid=true;
            }

            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
                
                target= 315+180;
                pros::delay(10); 
                start_pid=true;
            }
        }
   
        if(start_pid==true){

            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) ){
            Distance = Get_Distances(Positions, absOrientacionDeg); 
            index_min = Get_Index_min(Distance);
         
            target = index_min == 0 ? 0 : index_min == 1 ? 90 : index_min == 2 ? 180 : index_min == 3 ? 270 : target;
            }

            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
                target= 315+180;
            }

            
            target = reducir_angulo_180_180(target);
            
            TURN.zonaintegralactiva= target * .3;
            TURN.integralpowerlimit= 50/TURN.ki;
                
            TURN.error= reducir_angulo_180_180(target- absOrientacionDeg);

            TURN.proporcion= TURN.error * TURN.kp;

            if(fabs(TURN.error)>TURN.zonaintegralactiva && TURN.error!=0){TURN.integral_raw=0;}

            else{TURN.integral_raw+= TURN.error;}

            TURN.integral_raw= TURN.integral_raw > TURN.integralpowerlimit ? TURN.integral_raw : TURN.integral_raw <-TURN.integralpowerlimit ? -TURN.integral_raw : TURN.integral_raw;

            TURN.integral= TURN.ki*TURN.integral_raw;

            TURN.derivada= TURN.kd * (TURN.error - TURN.last_error);
            TURN.last_error= TURN.error;
            
            TURN.finalpower= (ceil(TURN.proporcion+TURN.integral+TURN.derivada));

            TURN.finalpower= TURN.finalpower > 100 ? 100 : TURN.finalpower < -100 ? -100:TURN.finalpower;  
    
            if(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 50){
                start_pid=false;
                turn_drift=0;
                TURN.finalpower=0;
            }

            if(abs(TURN.error)<=2){TURN.finalpower=0;}

            turn_drift=TURN.finalpower;

            pros::delay(10);


        }
    }
}

void Robot::PID_drift_Alex_four(void *ptr){
    int target=0;

    TURN.integral_raw=0;
    TURN.last_error=0; 

    TURN.kp = Turn_Constant[0];
    TURN.ki = Turn_Constant[1];
    TURN.kd = Turn_Constant[2];
    
    bool start_pid =false;
   

    while(1){
        turn_drift=0; 
    
        if(start_pid==false){

            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) ==1 || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) ==1 ||
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) ==1 || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) ==1  ){
            pros::delay(10); 
            start_pid=true; 
            }
        }


        if(start_pid==true){
            //std::cout<<"\n start ?: "<<start_pid;
            //std::cout<<"\t target: "<<target; 
            target = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)==1 && master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)==1 ? 315+180 :master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) ==1 ? 0 : master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) ==1 ? 90:
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) ==1 ? 180 : master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) ==1 ? 270 : target; 

            target = reducir_angulo_180_180(target);
            
            TURN.zonaintegralactiva= target * .3;
            TURN.integralpowerlimit= 50/TURN.ki;
                
            TURN.error= reducir_angulo_180_180(target- absOrientacionDeg);

            TURN.proporcion= TURN.error * TURN.kp;

            if(fabs(TURN.error)>TURN.zonaintegralactiva && TURN.error!=0){TURN.integral_raw=0;}

            else{TURN.integral_raw+= TURN.error;}

            TURN.integral_raw= TURN.integral_raw > TURN.integralpowerlimit ? TURN.integral_raw : TURN.integral_raw <-TURN.integralpowerlimit ? -TURN.integral_raw : TURN.integral_raw;

            TURN.integral= TURN.ki*TURN.integral_raw;

            TURN.derivada= TURN.kd * (TURN.error - TURN.last_error);
            TURN.last_error= TURN.error;
            
            TURN.finalpower= (ceil(TURN.proporcion+TURN.integral+TURN.derivada));

            TURN.finalpower= TURN.finalpower > 100 ? 100 : TURN.finalpower < -100 ? -100:TURN.finalpower;  
    
            if(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 50){
                start_pid=false;
                turn_drift=0;
                TURN.finalpower=0;
 
            }

            if(abs(TURN.error)<=2){TURN.finalpower=0;}

            turn_drift=TURN.finalpower;

            pros::delay(10);

        }
    }

}


void Robot::brake(std::string mode){
    if (mode.compare("COAST") == 0){
        LeftFront_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        LeftFront_2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        LeftBack_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        LeftBack_2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        RightFront_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        RightFront_2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        RightBack_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        RightBack_2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	}

    else if (mode.compare("HOLD")==0) {
        LeftFront_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        LeftFront_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        LeftBack_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        LeftBack_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        RightFront_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        RightFront_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        RightBack_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        RightBack_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    else{LeftFront_1 = LeftFront_2 = LeftBack_1 = LeftBack_2 = RightFront_1 = RightFront_2 = RightBack_1 = RightBack_2 = 0;}

}

void Robot::reset_sensors(){
    gyro.reset();
    Encoder_Derecho.reset();
    Expansion_.tare_position();
   
    Indexer.tare_position(); 
    Encoder_back.reset();
    
    //Rotacion.reset(); 
    //Rotacion.set_reversed(true);  //Es necesario para reversear el sensor de rotación
    
}



