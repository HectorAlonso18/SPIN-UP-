#include "PID.hpp"
#include "Purepursuit.h"
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

//Lambda fuction, para convertir de grados a radianes
#define TO_RAD(n) n * M_PI / 180;


//Puerto 1, 4  no jala

//Robot morado
pros::Motor Robot::LeftFront_1(2,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::LeftFront_2(3,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::LeftBack_1(11,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::LeftBack_2(12,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::RightFront_1(10,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::RightFront_2(9,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);


pros::Motor Robot::RightBack_1(20,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::RightBack_2(19,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::FlyWheel_1(5,pros::E_MOTOR_GEARSET_06,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::Flywheel_2(17,pros::E_MOTOR_GEARSET_06,false,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::intaker_1(6,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Robot::intaker_2(7,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Robot::Indexer(13, pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
 
pros::Imu Robot::gyro(8);

pros::Rotation Robot::Rotacion(18);


pros::ADIEncoder Robot::Encoder_Derecho('E','F',false);
pros::ADIEncoder Robot::Encoder_back('G','H',true);

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

        //Tiempo de actualizacion 
        pros::delay(10);
    }
}

void Robot::get_data(void){
    //Impresion durante 10 segundos
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

    //std::cout<<"\nCoordenada X "<<Robot::absGlobalX;
    //std::cout<<"\tCoordenada Y \t"<<Robot::absGlobalY;
    //std::cout<<"\tAngulo \t"<<Robot::absOrientacionDeg;
}

void Robot::Odom_Movement(double(*fuctPtr_Mode)(double,double,double),std::vector<double> posicion, std::vector<double>DrivePID, std::vector<double>TurnPID, double tiempo, double TargetX, double TargetY,float offset){
     
     /*
     Calculos necesarios para despues continuar con el ciclo de PID
     Debido a que son dos compensadores, se realizan los calculos necesarios 
     Para cada uno de ellos
     */

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
        //Aqui entra el puntero como parametro y cambia el valor de orientacion, dependiendo de cual puntero se haya elegido
        // si se escogio facing to -> el robot apuntará a un punto
        // si se escogio move to -> el robot no apuntará a ningun punto, en cambio tendra una orientacion predefinida
        Orientacion=fuctPtr_Mode(Orientacion,TargetX,TargetY);
        Orientacion+=offset;
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

        Robot::LeftFront_1.move_velocity(DRIVE.finalpower*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - M_PI/4))+TURN.finalpower) ;
        Robot::LeftFront_2.move_velocity(DRIVE.finalpower*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - M_PI/4))+TURN.finalpower);

        Robot::LeftBack_1.move_velocity(DRIVE.finalpower*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) -  M_PI/4))+TURN.finalpower);
        Robot::LeftBack_2.move_velocity(DRIVE.finalpower*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) -  M_PI/4))+TURN.finalpower);

        Robot::RightFront_1.move_velocity(DRIVE.finalpower*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - M_PI/4))-TURN.finalpower);
        Robot::RightFront_2.move_velocity(DRIVE.finalpower*(sin(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - M_PI/4))-TURN.finalpower);
       
        Robot::RightBack_1.move_velocity(DRIVE.finalpower*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - M_PI/4))-TURN.finalpower);
        Robot::RightBack_2.move_velocity(DRIVE.finalpower*(cos(absOrientacionRad + atan2(Y - absGlobalY, X - absGlobalX) - M_PI/4))-TURN.finalpower);

         
        if((fabs(DRIVE.error)<.2  && fabs(TURN.error) <2 )|| contador>=tiempo){condicion_odometria=true;}

        //std::cout<<"\n"<<contador; 
        //std::cout<<"\t"<<absGlobalY;
        //std::cout<<"\t24";
      

        pros::delay(10);
        contador+=10;
    }

    Robot::brake("stop");    
    pros::delay(10);

    /*
    for(int i=0; i<10000; i++){
        std::cout<<"\n"<<contador;
        std::cout<<"\t"<<absGlobalY;
        std::cout<<"\t24"; 
        pros::delay(10);
    }*/
}
  
void Robot::Turning(double(*fuctPtr_Mode)(double,double,double),float Orientacion,std::vector<double> TurnPID,double tiempo,float TargetX, float TargetY, float offset){

    TURN.kp=TurnPID[0];
    TURN.ki=TurnPID[1];
    TURN.kd=TurnPID[2];

    Orientacion= reducir_angulo_0_360(Orientacion);

    TURN.integral_raw=0;
    TURN.last_error=0;

    TURN.zonaintegralactiva= Orientacion *.3;
    TURN.integralpowerlimit= 50/TURN.ki;

    bool condicion=false;
     
    tiempo*=1000;
    int contador=0;

    std::cout<<"Zona integral "<<TURN.zonaintegralactiva;

    while(condicion==false){ 

        Orientacion = fuctPtr_Mode(Orientacion , TargetX , TargetY); 
        Orientacion += offset; 
              
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

    
        Robot::LeftFront_1.move_velocity(TURN.finalpower) ;
        Robot::LeftFront_2.move_velocity(TURN.finalpower);

        Robot::LeftBack_1.move_velocity(TURN.finalpower);
        Robot::LeftBack_2.move_velocity(TURN.finalpower);

        Robot::RightFront_1.move_velocity(-TURN.finalpower);
        Robot::RightFront_2.move_velocity(-TURN.finalpower);
       
        Robot::RightBack_1.move_velocity(-TURN.finalpower);
        Robot::RightBack_2.move_velocity(-TURN.finalpower);
     

        if( fabs(TURN.error) <2 || contador>=tiempo){condicion=true;}


        pros::delay(10);
        contador+=10;

        std::cout<<"\n"<<contador<<"\t"<<absOrientacionDeg<<"\t"<<"180";
    }
    Robot::brake("stop");   
  
    pros::delay(10);
}

void Robot::tune_pid(float tiempo, float step_percent){
    step_percent /=100; 
    step_percent= step_percent * 200; 
    int contador= 0; 
    tiempo *=1000; 
    float vel_chassis=0;
    std::cout<<"\nStep: "<<step_percent; 
    while(contador <tiempo){
        LeftFront_1.move_velocity(step_percent); RightFront_1.move_velocity(step_percent);  
        LeftFront_2.move_velocity(step_percent); RightFront_2.move_velocity(step_percent);

        LeftBack_1.move_velocity(step_percent); RightBack_1.move_velocity(step_percent);
        LeftBack_2.move_velocity(step_percent); RightBack_2.move_velocity(step_percent); 

        vel_chassis = (LeftFront_1.get_actual_velocity() + LeftFront_2.get_actual_velocity() + 
        RightFront_1.get_actual_velocity() + RightFront_2.get_target_velocity() + LeftBack_1.get_actual_velocity() + 
        LeftBack_2.get_actual_velocity() + RightBack_1.get_actual_velocity() + RightBack_2.get_actual_velocity() )/8;

        std::cout<<"\nTiempo: \t"<<contador; 
        std::cout<<"\tVel: \t"<< vel_chassis; 
        
        pros::delay(10); 
        contador+=10; 
    }
} 

void Robot::python_movement(double(*fuctPtr_mode)(double,double,double),std::vector<double> X, std::vector<double> Y, float tiempo){
    //Ciclo for que navega a traves del vector X
    for(auto i=0; i<X.size(); i++){
        //Mientras lo navega correra la función de Odom_Movement
        Odom_Movement(fuctPtr_mode, {X[i],Y[i],0}, Drive_Constant, Turn_Constant, tiempo, 0,0,0);
    }
}

void Robot:: Flywheel_pid (double RPM,std::vector<double>FlywheelPID, int n_disparos){

    //Puedes quitar las impresiones o utilizralas para ver como está el pedo

    FLYWHEEL.kp = FlywheelPID[0];
    FLYWHEEL.ki = FlywheelPID[1];
    FLYWHEEL.kd=  FlywheelPID[2]; 

    FLYWHEEL.integral_raw=0;
    FLYWHEEL.last_error=0;

    FLYWHEEL.zonaintegralactiva = RPM* .45;
    FLYWHEEL.integralpowerlimit = 50/FLYWHEEL.ki; 
    
    bool paro = false; 

    int contador=0; 
    
    float velocidad=0;

    int disparo=0; 

    bool estabilizooo=false;
    
    //El numero de disparos máximos son 3. 
    n_disparos= n_disparos>3 ?3 : n_disparos; 

    std::cout<<"\nZona Integral: "<<FLYWHEEL.zonaintegralactiva;
    
    while(paro==false){
        
        //Filtrado de la velocidad en RPM  esta esta manera de hacerlo o está otra
        velocidad = Fil::kalman.filter(Rotacion.get_velocity()/6)  ;
        //velocidad= Fil::kalman.filter(((Rotacion.get_velocity()/100)/.01)/6) pero según yo da lo mismo 
        
        //calculo del error
        FLYWHEEL.error = RPM - velocidad;

        FLYWHEEL.proporcion = FLYWHEEL.kp * FLYWHEEL.error;

        if (fabs (FLYWHEEL.error) > FLYWHEEL.zonaintegralactiva && FLYWHEEL.error !=0){FLYWHEEL.integral_raw=0;}

        else{FLYWHEEL.integral_raw += FLYWHEEL.error;}

        FLYWHEEL.integral_raw = FLYWHEEL.integral_raw > FLYWHEEL.integralpowerlimit ? FLYWHEEL.integralpowerlimit : FLYWHEEL.integral_raw < -FLYWHEEL.integralpowerlimit ? -FLYWHEEL.integralpowerlimit : FLYWHEEL.integral_raw;
        FLYWHEEL.integral = FLYWHEEL.ki * FLYWHEEL.integral_raw; 

        FLYWHEEL.derivada = FLYWHEEL.kd*(FLYWHEEL.error - FLYWHEEL.last_error);
        FLYWHEEL.last_error = FLYWHEEL.error; 
        
        //Manera de clampear el finalpower. 
        FLYWHEEL.finalpower += ceil (FLYWHEEL.proporcion + FLYWHEEL.integral + FLYWHEEL.derivada);

        Robot::FlyWheel_1.move_voltage(FLYWHEEL.finalpower);
        Robot::Flywheel_2.move_voltage(FLYWHEEL.finalpower);
        
        //Este es el sistema de disparo,
        //Cuenta con una tolerancia de 200 RPM, se puede modificar
        //Tolerancia de de hasta 200 RPM. 
        if(fabs(FLYWHEEL.error)<200 ){
            estabilizooo=true;  //Está es una variable para indicarnos cuando se estabiliza, no tiene una función en el código
                                //Pero para imprimir en la terminar es muy util, ya que te indica con un 0 o un 1, cuando el flywheel
                                //Esté estable

            Indexer.tare_position(); //Siempre se debe de reiniciar el encoder del indexer 
            Indexer.move_absolute(180,50); //Esta es la posicion y velocidad necesaria para realizar un disparo
            
            //Se pone un delay para darle tiempo al brian de darse cuenta que se desestabilizo el flywheel
            //Puedes probar disminuyendo hasta cual es lo minimo necesario
            pros::delay(500);  
                                             
            disparo++; //Con este incrementamos una variable para saber cuantos disparamos llevamos
        }

        else{
            estabilizooo=false;
        } 
        
        //Condicion para parar el flywheel, si se realizarón el numero de disparos deseados.
        paro = disparo==n_disparos ? true : false; 

        std::cout<<"\n"<<contador;
        std::cout<<"\t"<<velocidad;
        std::cout<<"\t"<<RPM;
        std::cout<<"\t"<<estabilizooo;  
        std::cout<<"\t"<<disparo;
        
        pros::delay(10);
        contador+=10;   
    } 
    
    FlyWheel_1.move_voltage(0);
    Flywheel_2.move_voltage(0);

}

 
void Robot::eat (bool state){
    int power_eat = state ==true  ? 12000 : state ==false ? 0 : power_eat;
    intaker_1.move_voltage(power_eat);  
    intaker_2.move_voltage(power_eat); 
} 

void Robot::Move_Roller(float distancia, int vel){
    intaker_1.tare_position();
    intaker_2.tare_position();

    intaker_1.move_absolute(distancia, vel);
    intaker_2.move_absolute(distancia, vel);
} 

void Robot::drive(void*ptr){

    int power_flywheel=0; //Poder del flywheel inicia en cero
    int power_indexer=0; //Estado de indexer inicia en false
    int power_intake=0; //Poder del intake inicia en cero

    bool state_flywheel=false; 
    

    while(true){
        /////Calculos necesarios para el movimiento del chassis///
        
        //ANALOG_LEFT_Y

        //Analog_right_x

        //analog_left_x
        int y= master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); //Joystick izquierdo (Arriba,Abajo)
        int turn=master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); //Joystick Derercho (Derecha, Izquierda)
        int x=master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X); //Joystick izquierdo (Derecha, Izquierda)

        double theta=atan2(y,x);
        double power=hypot(x,y);

        //Variable para conocer la orientacion actual en radianes (La otra variable se deja utilizar por el multitask)//        
        double inercia_rad = TO_RAD(gyro.get_heading()); 

        /*Angulo correspondiente a las llantas
        LeftFront y RightBack
        */                                         
        double strafe_lf_rb=cos(theta - M_PI/4 + inercia_rad);

        /*Angulo correspondiente a las llantas
        LeftBack y RighFront
        */ 
        double strafe_rf_lb=sin(theta - M_PI/4 + inercia_rad);
        
        //Funcion para manerar X-Drive
        x_drive(power,strafe_lf_rb,strafe_rf_lb,turn);

        /////////////////////////////Flywheel/////////////////////////////////////////////////////////////////////////////////////////// 
        state_flywheel = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)  ? !state_flywheel : state_flywheel; 

        power_flywheel= state_flywheel==true ? 12000 :  state_flywheel==false ? 0 : power_flywheel ;
       
        move_Flywheel(power_flywheel);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        /////////////////////////////////////Intake/////////////////////////////////////////////////////////////////////////////////////////////
        power_intake=master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)==1 ? 12000 : master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)==1 ? -12000: 0;
        move_Intake(power_intake);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        /////////////////////////////////////////////////////////Indexer////////////////////////////////////////////////////////////////////////////////////////////////////
        power_indexer = master.get_digital(pros::E_CONTROLLER_DIGITAL_A)==1 ? 6000 : master.get_digital(pros::E_CONTROLLER_DIGITAL_B)==1 ? -6000 : 0; 
        move_Indexer(power_indexer); 
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
        pros::delay(10);  
    }
}

void Robot::x_drive(double power, double strafe_lf_rb,double strafe_rf_lb,double turn){
    //Limitador de velocidad
    double max=std::max(abs(strafe_lf_rb),abs(strafe_rf_lb));

    ////////////Calculo de poder para cada una de las llantas///////////
    
    double power_left_front= power*(strafe_lf_rb/max)+(turn+turn_joytick);
    double power_left_back= power*(strafe_rf_lb/max)+(turn+turn_joytick);

    double power_righ_front= power*(strafe_rf_lb/max)-(turn+turn_joytick);
    double power_righ_back= power*(strafe_lf_rb/max)-(turn+turn_joytick);

    //Turn_joystick es la salida de PID_Drift, como este compensador
    //Actua en el modo driver, su valor tiene que estar incluido aqui
    ///////////////////////////////////////////////////////////////////
    
    //Finalmente movemos los motores
    LeftFront_1.move(power_left_front);    RightFront_1.move(power_righ_front);
	LeftFront_2.move(power_left_front);    RightFront_2.move(power_righ_front);

	LeftBack_1.move(power_left_back);	     RightBack_1.move(power_righ_back);
	LeftBack_2.move(power_left_back);      RightBack_2.move(power_righ_back);
}

void Robot::move_Flywheel(int power){
    FlyWheel_1.move_voltage(power); 
    Flywheel_2.move_voltage(power);
}

void Robot::move_Intake(int power){
    intaker_1.move_voltage(power);
    intaker_2.move_voltage(power);
}

void Robot::move_Indexer(int power){
    Indexer.move_voltage(power); 
} 

void Robot::PID_drift(void *ptr){
  
    int target=0;

    TURN.integral_raw=0;
    TURN.last_error=0; 
    
    TURN.kp = 2;
    TURN.ki = Turn_Constant[1];
    TURN.kd = Turn_Constant[2];
    
    bool state_up =false;
    bool verificacion=false;

    //Vector donde vienen las diferentes posiciones a girar
    std::vector<int> Positions {0,90,180,270,360};
    //Vecotr donde se almacenar las distancias entre la orientacion actual y la target
    std::vector<int> distance {0,0,0,0};
    
    //Variables necesarias para encontrar el elemento minimo dentro del vector de distancia
    int min=distance[0];
    int index_min=0;
    
    turn_joytick=0;

    while(1){
    //Ciclo sin fin, Dentro de este ciclo existen otros dos ciclos, que nos sirven
    //Para poder seleccionar el target y otro nos ayuda a ejecutar el PID 
        while(verificacion==false){
            //Inicializamos el power de PID
            turn_joytick=0;
    
            //Inicializamos las variables para encontrar el minimo
            min=distance[0];
            index_min=0;

            //Condicional para seleccionar el target que nosotros buscamos 
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
                //Ciclo for, para recorrer cada elemento dentro del vector de posision
                for (auto i = 0; i<Positions.size(); i++) {
                    distance[i] = abs(Positions[i] - absOrientacionDeg); //Calcula la distancia entre la orientacion actual y las posiciones target
                }
                
                //Ciclo for, para recorrer cada elemento dentro del vector de distancias 
                for (auto i = 1; i < distance.size(); i++) {
                    //Para encontrar el valor mas pequeño dentro de un vector, necesitamos suponer que el primer elemento es el menor
                    //si la distancia actual es menor que la menor
                    if (distance[i] < min) {
                        min = distance[i]; //menor toma el valor de la distancia
                        index_min = i;    //Se almacena el index de la distancia mas pequeña
                    }
                }

                //Una vez calculado el index, se actualiza la informacion e indicamos que se ha presionado el boton 
                state_up=true;
            }
        
                //Escogemos el target a movernos dependiendo del valor del index.
                target = index_min == 0 ? 0 : index_min == 1 ? 90 : index_min == 2 ? 180 : index_min == 3 ? 270 : index_min ==4 ? 0 : target;

                std::cout<<"\nverificacion: "<<verificacion;

                std::cout<<"\tTarget: "<<target;
                
                //Si el boton fue presionado, verificacion cambia de estado y se cierra el ciclo, entrando al siguiente
                
                verificacion = state_up ==true ? true : verificacion;

                pros::delay(10);
        }

        while(verificacion==true){
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
                min=distance[0];
                index_min=0;

                for (auto i = 0; i<Positions.size(); i++) {
                    distance[i] = abs(Positions[i] - absOrientacionDeg);
                }

                for (auto i = 1; i < distance.size(); i++) {
                    if (distance[i] < min) {
                        min = distance[i];
                        index_min = i;
                    }
                }
            }

            target = index_min == 0 ? 0 : index_min == 1 ? 90 : index_min == 2 ? 180 : index_min == 3 ? 270 : index_min ==4 ? 0 : target;

            //Inician operaciones de PID

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
        
            //Si el joystick correspondiente a los giros tiene un valor mayor a 50, significa que el driver quiere
            //Cambiar de orientacion, por lo tanto la orientacion se desbloquea y deja de accionarse el control PID
            if(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 50){
                state_up=false;
                verificacion=false;
                turn_joytick=0;
                TURN.finalpower=0;
            }

            if(abs(TURN.error)<=2){TURN.finalpower=0;}

            std::cout<<"\nverificacion: "<<verificacion;

            std::cout<<"\tTarget: "<<target;
            
            std::cout<<"\tPower: "<<TURN.finalpower;

            turn_joytick=TURN.finalpower;

            std::cout<<"\tPower joytick: "<<turn_joytick;
            pros::delay(10);
        }
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
    Rotacion.reset(); 
    Rotacion.set_reversed(false);  //Es necesario para reversear el sensor de rotación
}

