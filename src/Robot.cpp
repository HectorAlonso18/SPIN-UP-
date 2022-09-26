#include "PID.hpp"
#include "Purepursuit.h"
#include "main.h"

#include "odometria.h"
#include "parametros.h"
#include "pros/misc.h"
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

pros::Motor Robot::intaker_1(6,pros::E_MOTOR_GEARSET_06,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::intaker_2(7,pros::E_MOTOR_GEARSET_06,true,pros::E_MOTOR_ENCODER_DEGREES);
 
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

std::atomic<double> Robot::turn_joytick=0;

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

 

void Robot::Odom_Movement(double(*fuctPtr_Mode)(double,double,double),std::vector<double> posicion, std::vector<double>DrivePID, std::vector<double>TurnPID, double tiempo, double TargetX, double TargetY){
     double X= posicion[0];
     double Y=posicion[1];
     double Orientacion= posicion[2];

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
     
     bool condicion_odometria=false;
     
     tiempo*=1000;
     int contador=0;

     while(condicion_odometria==false){
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

        //if(fabs(turn_error)<turn_error_range){finalpower_turn=0;}
        //if(fabs(drive_error)<drive_error_range){finalpower_drive=0;}

        Robot::LeftFront_1.move_velocity(DRIVE.finalpower*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+TURN.finalpower) ;
        Robot::LeftFront_2.move_velocity(DRIVE.finalpower*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+TURN.finalpower);

        Robot::LeftBack_1.move_velocity(DRIVE.finalpower*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+TURN.finalpower);
        Robot::LeftBack_2.move_velocity(DRIVE.finalpower*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+TURN.finalpower);

        Robot::RightFront_1.move_velocity(DRIVE.finalpower*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-TURN.finalpower);
        Robot::RightFront_2.move_velocity(DRIVE.finalpower*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-TURN.finalpower);
       
        Robot::RightBack_1.move_velocity(DRIVE.finalpower*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-TURN.finalpower);
        Robot::RightBack_2.move_velocity(DRIVE.finalpower*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-TURN.finalpower);
         
        if((fabs(DRIVE.error)<.2 && fabs(TURN.error) <2)|| contador>=tiempo){condicion_odometria=true;}

        //std::cout<<"\nTiempo: "<<contador;
        //std::cout<<"\tD Error: "<<drive_error;
        //std::cout<<"\tT Error: "<<turn_error;

        std::cout<<"Orientacion: "<<Orientacion;

        pros::delay(10);
        contador+=10;
    }

    Robot::brake("stop");    
    pros::delay(10);
 }

    
void Robot::PID_Movement(void(*fuctPtr_Mode)(double,double),std::vector<double>posicion,std::vector<double>DrivePID,std::vector<double> TurnPID,double tiempo){
    double X= posicion[0];
    double Y=posicion[1];
    double Orientacion= posicion[2];

    DRIVE.kp=DrivePID[0];
    DRIVE.ki=DrivePID[1];
    DRIVE.kd=DrivePID[2];

    TURN.kp=TurnPID[0];
    TURN.ki=TurnPID[1];
    TURN.kd=TurnPID[2];

    Orientacion= reducir_angulo_0_360(Orientacion);
    
    float integral_raw_drive=0;
    float last_error_drive=0;

    float zonaintegralactiva_drive= (hypot(X-absGlobalX,Y-absGlobalY))*.45;
    float integralpowerlimit_drive= 50/DRIVE.ki;

    float integral_raw_turn=0;
    float last_error_turn=0;

    float zonaintegralactiva_turn= Orientacion * .3;
    float integralpowerlimit_turn= 50/TURN.ki;

    bool condicion_odometria=false;
     
    tiempo*=1000;
    int contador=0;

    while(condicion_odometria==false){
        
        float drive_error= hypot(X-absGlobalX,Y-absGlobalY); 
        float turn_error= reducir_angulo_180_180(Orientacion- absOrientacionDeg);

        float drive_proporcion= drive_error * DRIVE.kp;
        float turn_proporcion= turn_error * TURN.kp;

        if(fabs(drive_error)>zonaintegralactiva_drive && drive_error!=0){integral_raw_drive=0;}

        else{integral_raw_drive+= drive_error;}

        if(fabs(turn_error)>zonaintegralactiva_turn && turn_error!=0){integral_raw_turn=0;}

        else{integral_raw_turn+= turn_error;}

        integral_raw_drive = integral_raw_drive > integralpowerlimit_drive ? integralpowerlimit_drive : integral_raw_drive < -integralpowerlimit_drive ? -integralpowerlimit_drive: integral_raw_drive;

        float integral_drive= DRIVE.ki*integral_raw_drive;

        integral_raw_turn= integral_raw_turn > integralpowerlimit_turn ? integral_raw_turn : integral_raw_turn <-integralpowerlimit_turn ? -integral_raw_turn : integral_raw_turn;

        float integral_turn= TURN.ki*integral_raw_turn;

        float derivada_drive = DRIVE.kd* (drive_error - last_error_drive);
        last_error_drive=drive_error;

        float derivada_turn= TURN.kd * (turn_error - last_error_turn);
        last_error_turn= turn_error;

        float finalpower_drive= (ceil(drive_proporcion+integral_drive+derivada_drive));
        float finalpower_turn= (ceil(turn_proporcion+integral_turn+derivada_turn));

        fuctPtr_Mode(finalpower_drive,finalpower_turn);

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

/*
void Robot::move_to_pure_pursuit(std::vector< std::vector<double> > points, std::vector<double> final_point, int lookAheadDistance){

    std::vector<double> end;
    std::vector<double> start;
    std::vector<double> target;
    std::vector<double> current {absGlobalX,absGlobalY};
    double Orientacion; 

    for(int index=0; index<points.size()-1; index++){
        start= points[index];
        end= points[index+1];

        while(distance(current,end) > lookAheadDistance){
            target= get_intersection(start, end, current, lookAheadDistance);
            Orientacion= get_angle_pro(current, target);

            std::vector<double>posicion {target[0],target[1],Orientacion}; 
            Odom_Movement(Control_move_to, posicion,Drive_Constant,Turn_Constant, 5,0,0);
            current= {absGlobalX,absGlobalY};
            pros::delay(10);
        }
    }
    
    Odom_Movement(Control_move_to, {final_point}, Drive_Constant, Turn_Constant, 5, 0, 0);
    brake("stop");

}*/


 void Robot::python_movement(double(*fuctPtr_mode)(double,double,double),std::vector<double> X, std::vector<double> Y, float tiempo){
    for(auto i=0; i<X.size(); i++){
        Odom_Movement(fuctPtr_mode, {X[i],Y[i],0}, Drive_Constant, Turn_Constant, tiempo, 0,0);
    }
 }

/*Prototipo de flywheel / disparador
 
 Mientras el flywheel no haya alcanzado su target la función de disparo se activará

 Cuando lo alcance, la función disparará. Cuando el disco pasa por el flywheel va a desestablizarse
 por lo tanto no vuelve a disparar

 el flywheel con pid se vuelve a estabilizar -> disparo

 Por cada vez que te dispara se incrementa una variable que cuenta los tiros

 Una vez que llega a los 3 tiros, el ciclo se rompe y se para el flywheel

 la variable de disparo se reinicia para volver a empezar si llamamos la función
  
 flywheel{ 
 while(i<3){

    PID DE FLYWHEEL

    si se estabilizó-> state=true;

    if(state==true){
        funcion de disparo
        acciona y regresa con un delay 
        incrementamos variable i++; 
    } 
 }
 i=0;
 flywheel.stop
 }

*/



void Robot::drive(void*ptr){

    int power_flywheel=0;
    bool state_piston=false;
    int power_intake=0;

    while(true){
        //Chassis 
        int y= master.get_analog(ANALOG_LEFT_Y);
        int turn=master.get_analog(ANALOG_RIGHT_X);
        int x=master.get_analog(ANALOG_LEFT_X);

        double theta=atan2(y,x);
        double power=hypot(x,y);
        double inercia_rad = TO_RAD(gyro.get_heading());
                                                 
        double strafe_lf_rb=cos(theta - M_PI/4 + inercia_rad); 
        double strafe_rf_lb=sin(theta - M_PI/4 + inercia_rad);

        x_drive(power,strafe_lf_rb,strafe_rf_lb,turn);

        std::cout<<"\nturn_joytick: "<<turn_joytick;

        //Flywheel
        power_flywheel= master.get_digital(DIGITAL_R2)==1 ? 12000 : master.get_digital(DIGITAL_R1)==1 ? -12000 :0; 
        Fly_wheel_action(power_flywheel);
        
        power_intake=master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)==1 ? 12000 : master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)==1 ? -12000: 0;

        intaker_1.move_voltage(power_intake);
        intaker_2.move_voltage(power_intake);

        pros::delay(10);  
    }
}


void Robot::x_drive(double power, double strafe_lf_rb,double strafe_rf_lb,double turn){
    double max=MAX(abs(strafe_lf_rb),abs(strafe_rf_lb));

    LeftFront_1.move(power*(strafe_lf_rb/max)+(turn+turn_joytick));    RightFront_1.move(power*(strafe_rf_lb/max)-(turn+turn_joytick));
	LeftFront_2.move(power*(strafe_lf_rb/max)+(turn+turn_joytick));    RightFront_2.move(power*(strafe_rf_lb/max)-(turn+turn_joytick));

	LeftBack_1.move(power*(strafe_rf_lb/max)+(turn+turn_joytick));	 RightBack_1.move(power*(strafe_lf_rb/max)-(turn+turn_joytick));
	LeftBack_2.move(power*(strafe_rf_lb/max)+(turn+turn_joytick));     RightBack_2.move(power*(strafe_lf_rb/max)-(turn-turn_joytick));
}

void Robot::Fly_wheel_action(int power){
    FlyWheel_1.move_voltage(power); 
    Flywheel_2.move_voltage(power);
}

void Robot::Piston_movement(bool state){
    piston.set_value(state); 
}


void Robot::PID_drift(void *ptr){
    bool state_up, state_down, state_left, state_right;

    state_up=false;
    state_down=false;
    state_right=false;
    state_left=false;

    int target=0;

    TURN.integral_raw=0;
    TURN.last_error=0;

    TURN.kp = Turn_Constant[0];
    TURN.ki = Turn_Constant[1];
    TURN.kd = Turn_Constant[2];

    bool verificacion=false;

while(1){
  
 while(verificacion==false){

    turn_joytick=0;
  
    if(master.get_digital_new_press(DIGITAL_X)){
        state_up=true;
        state_down=false;
        state_right=false;
        state_left=false;
    }

     if(master.get_digital_new_press(DIGITAL_B)){
        state_up=false;
        state_down=true;
        state_right=false;
        state_left=false;
    }

     if(master.get_digital_new_press(DIGITAL_A)){
        state_up=false;
        state_down=false;
        state_right=true;
        state_left=false;
    }

    if(master.get_digital_new_press(DIGITAL_Y)){
        state_up=false;
        state_down=false;
        state_right=false;
        state_left=true;
    }


    target = state_up ==true ? 0 : state_down==true ? 180 : state_right==true ? 90 : state_left==true ?270 :target;

    //std::cout<<"\nverificacion: "<<verificacion;

    //std::cout<<"\tTarget: "<<target;

    if(state_up==true || state_down==true || state_right==true || state_left==true){
        verificacion=true;
    }
  
    pros::delay(10);
 }

    while(verificacion==true){

        if(master.get_digital_new_press(DIGITAL_X)){
            state_up=true;
            state_down=false;
            state_right=false;
            state_left=false;
        }

        if(master.get_digital_new_press(DIGITAL_B)){
            state_up=false;
            state_down=true;
            state_right=false;
            state_left=false;
        }

        if(master.get_digital_new_press(DIGITAL_A)){
            state_up=false;
            state_down=false;
            state_right=true;
            state_left=false;
        }

        if(master.get_digital_new_press(DIGITAL_Y)){
            state_up=false;
            state_down=false;
            state_right=false;
            state_left=true;
        }

        target = state_up ==true ? 0 : state_down==true ? 180 : state_right==true ? 90 : state_left==true ?270 :target;

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

        TURN.finalpower= TURN.finalpower > 150 ? 150 : TURN.finalpower < -150 ? 150:TURN.finalpower;    
    
        if(abs(master.get_analog(ANALOG_RIGHT_X)) > 50){
            state_up=false;
            state_down=false;
            state_right=false;
            state_left=false;
            verificacion=false;
        }

        if(abs(TURN.error)<=2){TURN.finalpower=0;}

        //std::cout<<"\nverificacion: "<<verificacion;

        //std::cout<<"\tTarget: "<<target;
        
        //std::cout<<"\tPower: "<<TURN.finalpower;

        turn_joytick= TURN.finalpower*.635;
/*
        Robot::LeftFront_1.move_velocity(TURN.finalpower) ;
        Robot::LeftFront_2.move_velocity(TURN.finalpower);

        Robot::LeftBack_1.move_velocity(TURN.finalpower);
        Robot::LeftBack_2.move_velocity(TURN.finalpower);

        Robot::RightFront_1.move_velocity(-TURN.finalpower);
        Robot::RightFront_2.move_velocity(-TURN.finalpower);
       
        Robot::RightBack_1.move_velocity(-TURN.finalpower);
        Robot::RightBack_2.move_velocity(-TURN.finalpower);
*/

        //std::cout<<"\tPower joytick: "<<turn_joytick;

        pros::delay(10);
    }

    Robot::brake("stop");
}
         
}

   
    
 
void Robot::brake(std::string mode){
    if (mode.compare("coast") == 0){
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

    else{LeftFront_1 = LeftFront_2 = LeftBack_1 = LeftBack_2 = RightFront_1 = RightFront_2 = RightBack_1 = RightBack_2 = 0;}

}


void Robot::reset_sensors(){
    gyro.reset();
    Encoder_Derecho.reset();
    Encoder_back.reset();
}

