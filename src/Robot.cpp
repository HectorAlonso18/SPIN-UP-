#include "main.h"
#include "pros/motors.h"
#include <cmath>
#include <atomic>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
#include <bits/stdc++.h> 

//Lambda fuction, para convertir de grados a radianes
#define TO_RAD(n) n * M_PI / 180;

pros::Motor Robot::LeftFront_1(10,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::LeftFront_2(9,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::LeftBack_1(1,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::LeftBack_2(2,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::RightFront_1(20,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::RightFront_2(19,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::RightBack_1(11,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::RightBack_2(12,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
 
pros::Imu Robot::gyro(5);

pros::ADIEncoder Robot::Encoder_Derecho('A','B',true);
pros::ADIEncoder Robot::Encoder_back('G','H',false);

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

double Robot::Distancia_Encoder_Y=.75;
double Robot::Distancia_Encoder_X=.30;
  
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

double Robot::EncoderY_Actual=0;
double Robot::EncoderX_Actual=0;

double Robot::prev_A=0;

void Robot::raestro(void *ptr){
    while(true){
        updateEncoders();
        updatePosicion();
        pros::delay(10);
    }
}


void Robot::updateEncoders(void){
    float dead_zone_encoder_D = (2*gyro.get_heading()*Distancia_Encoder_Y)/(Diametro_Y);
    float dead_zone_encoder_B = (2*gyro.get_heading()*Distancia_Encoder_X)/(Diametro_X);

    EncoderY_Actual= (Encoder_Derecho.get_value()- dead_zone_encoder_D) * (f_Derecho);
    EncoderX_Actual = (Encoder_back.get_value() - dead_zone_encoder_B) * (f_back);

    delta_Y = EncoderY_Actual - prev_Y;
    delta_X = EncoderX_Actual - prev_X;

    prev_Y= EncoderY_Actual;
    prev_X= EncoderX_Actual;   

}


void Robot::updatePosicion(void){
    absOrientacionDeg= reducir_angulo_0_360(gyro.get_heading());
    absOrientacionRad= TO_RAD(absOrientacionDeg);
    
    float angle_gyro_Rad= TO_RAD(reducir_angulo_0_360(gyro.get_heading()));
    float delta_A= angle_gyro_Rad-prev_A;
    prev_A=angle_gyro_Rad;

    if(delta_A==0){
        localX = delta_X;
        localY = delta_Y;
    }

    else{
        localX =(2*sin(delta_A/2)) * ((delta_X/delta_A)+Distancia_Encoder_X);
        localY= (2*sin(delta_A/2)) * ((delta_Y/delta_A)+Distancia_Encoder_Y);
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
    absGlobalX =(prevGlobalX + globalX); 
    absGlobalY =(prevGlobalY + globalY); 

    prevGlobalX = absGlobalX;
    prevGlobalY = absGlobalY;

    prevOrientacionRad = absOrientacionRad;

    //std::cout<<"\nCoordenada X "<<absGlobalX;
    //std::cout<<"\tCoordenada Y "<<absGlobalY;
}


 void Robot::move_to(double X, double Y, double Orientacion, double kp_drive, double ki_drive, double kd_drive, double kp_turn,double ki_turn,double kd_turn, double drive_error_range, double turn_error_range){
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

     while(condicion_odometria==false){
         float drive_error= hypot(X-absGlobalX,Y-absGlobalY); 
         float turn_error= reducir_angulo_180_180(Orientacion- absOrientacionDeg);

         float drive_proporcion= drive_error * kp_drive;
         float turn_proporcion= turn_error * kp_turn;

         if(fabs(drive_error)<zonaintegralactiva_drive && drive_error!=0){integral_raw_drive=0;}

         else{integral_raw_drive+= drive_error;}

         if(fabs(turn_error)<zonaintegralactiva_turn && turn_error!=0){integral_raw_turn=0;}

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

         if(fabs(turn_error)<turn_error_range){finalpower_turn=0;}

         if(fabs(drive_error)<drive_error_range){finalpower_drive=0;}

         Robot:: LeftFront_1.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn) ;
         Robot::LeftFront_2.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);

         Robot::LeftBack_1.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);
         Robot::LeftBack_2.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))+finalpower_turn);

         Robot::RightFront_1.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
         Robot::RightFront_2.move_velocity(finalpower_drive*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
       
         Robot::RightBack_1.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
         Robot::RightBack_2.move_velocity(finalpower_drive*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - PI/4))-finalpower_turn);
    
         if(fabs(drive_error)<drive_error_range && fabs(turn_error) < turn_error_range ){condicion_odometria=true;}

        //std::cout<<"\ndrive error"<<drive_error;
        //std::cout<<"\tturn error"<<turn_error;

         pros::delay(10);
    }

        Robot::LeftFront_1.move_velocity(0);   Robot::RightFront_1.move_velocity(0);
        Robot::LeftFront_2.move_velocity(0);   Robot::RightFront_2.move_velocity(0);
        
        Robot::LeftBack_1.move_velocity(0);   Robot::RightBack_1.move_velocity(0);
        Robot::LeftBack_2.move_velocity(0);    Robot::RightBack_2.move_velocity(0);
             
        pros::delay(10);
 }


 void Robot::rotate_to(double Orientacion, double kp_turn, double ki_turn, double kd_turn, double tiempo){
     Orientacion= reducir_angulo_0_360(Orientacion);

     float integral_raw_turn=0;
     float last_error_turn=0;

     float zonaintegralactiva_turn= Orientacion * .3;
     float integralpowerlimit_turn= 50/ki_turn;

     bool condicion_odometria=false;
     
     while(condicion_odometria==false){

         float turn_error= reducir_angulo_180_180(Orientacion-gyro.get_heading());
         float turn_proporcion= turn_error * kp_turn;

         if(fabs(turn_error)<zonaintegralactiva_turn && turn_error!=0){integral_raw_turn=0;}

         else{integral_raw_turn+= turn_error;}

         integral_raw_turn= integral_raw_turn > integralpowerlimit_turn ? integral_raw_turn : integral_raw_turn <-integralpowerlimit_turn ? -integral_raw_turn : integral_raw_turn;

         float integral_turn= ki_turn*integral_raw_turn;

         float derivada_turn= kd_turn * (turn_error - last_error_turn);
         last_error_turn= turn_error;
      
         float finalpower_turn= (ceil(turn_proporcion+integral_turn+derivada_turn));

         finalpower_turn= finalpower_turn > 150 ? 150 : finalpower_turn < -150 ? -150:finalpower_turn;

         if(fabs(turn_error)<2){finalpower_turn=0;}

         LeftFront_1.move_velocity(finalpower_turn) ;
         LeftFront_2.move_velocity(finalpower_turn);

         LeftBack_1.move_velocity(finalpower_turn);
         LeftBack_2.move_velocity(finalpower_turn);

         RightFront_1.move_velocity(-finalpower_turn);
         RightFront_2.move_velocity(-finalpower_turn);
       
         RightBack_1.move_velocity(-finalpower_turn);
         RightBack_2.move_velocity(-finalpower_turn);
    
         if(fabs(turn_error) < 2 ){condicion_odometria=true;}

         pros::delay(10);
    }

        Robot::LeftFront_1.move_velocity(0);   Robot::RightFront_1.move_velocity(0);
        Robot::LeftFront_2.move_velocity(0);   Robot::RightFront_2.move_velocity(0);
        
        Robot:: LeftBack_1.move_velocity(0);   Robot::RightBack_1.move_velocity(0);
        Robot::LeftBack_2.move_velocity(0);    Robot::RightBack_2.move_velocity(0);

        pros::delay(10);
 }



void Robot::drive(void*ptr){
    while(true){
        double y= master.get_analog(ANALOG_LEFT_Y);
        double turn=master.get_analog(ANALOG_RIGHT_X);
        double x=master.get_analog(ANALOG_LEFT_X);
        double theta=atan2(y,x);
        double power=hypot(x,y);
        double inercia_rad = TO_RAD(gyro.get_heading());

        double strafe_lf_rb=cos(theta - M_PI/4 + inercia_rad); 
        double strafe_rf_lb=sin(theta - M_PI/4 + inercia_rad);

        x_drive(power,strafe_lf_rb,strafe_rf_lb,turn);   

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

