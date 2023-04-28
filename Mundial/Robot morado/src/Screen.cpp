#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_label.h"
#include "main.h"
#include "pros/rtos.hpp"
#include <string>


namespace screen {
    //Creamos un puntero de string, ponemos nuestro macro donde vienen todos los autonomos, separados por ""
const char *b[] = {AUTONOMOS, ""};
int autonomo;  //Variable que nos ayuda para el selector
int autonCount;

//Constante que nos ayuda a almacenar las rutinas
const char *btnmMap[] = {"","",""}; // 10 rutinas de autonomo

//Creamos objetos de tabla
lv_obj_t *tabview; //Este es nuestro objeto principal

//objetos para botones
lv_obj_t *matchBtnm; //objeto para las rutinas rojas


/////////////////////////////////////////////////////ESTO ES UN CALIS/////////////////////////////////////
//boton 5
lv_obj_t * Pokayoke;
lv_obj_t * Pokayoke_texto;
lv_style_t Pokayoke_fuera;
lv_style_t Pokayoke_apachurrado;
lv_obj_t * texto_confirmacion; //Botón para confirmar, [esto es un calis]
int id2=0;            //tambien esto es calis


//Funcion para la confirmación xd esto es calis ok 
 lv_res_t Pokayoke_confirmacion(lv_obj_t * btn)
{
		 id2 = lv_obj_get_free_num(btn);
		 if (id2!=0) {lv_obj_clean( lv_scr_act() );}
     return LV_RES_OK;
}
/////////////////////////////////////////////////////////////////////////////////////////////

//Función para la parte roja
lv_res_t matchBtnmAction(lv_obj_t *btnm, const char *txt){


  //fUNCION strcmp compara dos string caracter por caracter, si son iguales retorna0
	for(int i = 0; i < autonCount; i++){
		if(strcmp(txt, btnmMap[i]) == 0){  //si los strings txt y el map son iguales
			autonomo = i+1;  //autonomo incrementa 1
		}
	}

	if(autonomo==1){
		lv_label_set_text(texto_confirmacion, "#ffffff Ofensiva, Jalas ALV? #"); 
	}

	if(autonomo==2){
		lv_label_set_text(texto_confirmacion, "#ffffff Defensiva, Jalas ALV? #"); 
	}

	return LV_RES_OK; // returna ok
}




//Funcion para la parte de skills
lv_res_t skillsBtnAction(lv_obj_t *btn){
  //el valor del autonomo solo será 0 debido a que solo necesitamos un boton para esto
	autonomo = 0;


	lv_label_set_text(texto_confirmacion, "#ffffff Skills, Jalas ALV? #"); 
		

	return LV_RES_OK;
}

//Funcion importante ya que aqui se mostrara nuestra tabla en un ciclo while
void tabWatcher(void*ptr) {
	//Obtenemos el index de nuestra tabview activa
	int Tab_activa = lv_tabview_get_tab_act(tabview);

	while(1){
		//Obtenemos tambine el index de nuestra tabview actual
		int Tab_actual = lv_tabview_get_tab_act(tabview);

		//si nuestra actual tab es diferente que la tabactiva ambas obtienen el mismo valor
		if(Tab_actual != Tab_activa){
			Tab_activa = Tab_actual;
			if(Tab_activa == 0){
				if(autonomo == 0) autonomo = 1;
				autonomo = abs(autonomo);
				//Activamos toggling del boton rojo, el tercer parametro es el index del actual boton
				lv_btnm_set_toggle(matchBtnm, true, abs(autonomo)-1);

			}

			else{
				autonomo = 0;
				
			}
		}

		
   

		pros::delay(20);
	}
}

//Funcion principal para escoger
//parametro hue es el valor en HSV color, el segundo es el autonomo que tomara por default, el tercer es la lista de rutinas
void init(int hue, int default_auton, const char **autons){

	//inicializamos iterador
	int i = 0;

	do{
		//memcpy copia los valores de numero de bytes desde la ubicacion
		//a la que apunta el origen directamente al bloque de memoria al que apunta
		//el destino


	 //Primer parametro es:puntero de destino donde el contenido sera copiado
	 //Segundo parametro, es puntero a la fuente de datos a copiar
	 //tercer parametro, es el numero de bytes
		memcpy(&btnmMap[i], &autons[i], sizeof(&autons));
		i++;
		//hacer esto mientras los string sea diferentes
	}while(strcmp(autons[i], "") != 0);

	autonCount = i;

	autonomo = default_auton;

	// tema del lvgl

	lv_theme_t *th = lv_theme_alien_init(hue,NULL); //Obtiene el valor del macro de header
	lv_theme_set_current(th);

	// creamos el objeto tabview
	tabview = lv_tabview_create(lv_scr_act(), NULL);

	// Añadimos nuestros 3 tabs, que pueden ser scrolleados
	lv_obj_t *Tab_match = lv_tabview_add_tab(tabview, "Match");
	lv_obj_t *Tab_skills = lv_tabview_add_tab(tabview, "Skills");



  


	//este es el contenido de nuestro tabs

	//tab rojo
	matchBtnm = lv_btnm_create(Tab_match, NULL);
	lv_btnm_set_map(matchBtnm, btnmMap);
	lv_btnm_set_action(matchBtnm, matchBtnmAction);
	lv_btnm_set_toggle(matchBtnm, true, abs(autonomo)-1);//3
	lv_obj_set_size(matchBtnm, 450, 50);
	lv_obj_set_pos(matchBtnm, 0, 100);
	lv_obj_align(matchBtnm, NULL, LV_ALIGN_CENTER, 0, 0);

	//tab skills
	lv_obj_t *skillsBtn = lv_btn_create(Tab_skills, NULL);
	lv_obj_t *label = lv_label_create(skillsBtn, NULL);
	lv_label_set_text(label, "Skills");
	lv_btn_set_action(skillsBtn, LV_BTN_ACTION_CLICK, *skillsBtnAction);
	// lv_btn_set_state(skillsBtn, LV_BTN_STATE_TGL_REL);
	lv_obj_set_size(skillsBtn, 450, 50);
	lv_obj_set_pos(skillsBtn, 0, 100);
	lv_obj_align(skillsBtn, NULL, LV_ALIGN_CENTER, 0, 0);

	/////////////////////////////////calis////////////////////////////////////////////
	lv_style_copy(&Pokayoke_fuera, &lv_style_plain);
	Pokayoke_fuera.body.main_color = LV_COLOR_MAKE(255, 77, 255);
	Pokayoke_fuera.body.grad_color = LV_COLOR_MAKE(255, 77, 255);
	Pokayoke_fuera.body.radius = 0;
	Pokayoke_fuera.text.color = LV_COLOR_MAKE(255, 255, 255);

	lv_style_copy(&Pokayoke_apachurrado, &lv_style_plain);
	Pokayoke_apachurrado.body.main_color = LV_COLOR_MAKE(179, 0, 179);
	Pokayoke_apachurrado.body.grad_color = LV_COLOR_MAKE(179, 0, 179);
	Pokayoke_apachurrado.body.radius = 0;
	Pokayoke_apachurrado.text.color = LV_COLOR_MAKE(255, 255, 255);

	Pokayoke = lv_btn_create(lv_scr_act(), NULL);
	lv_obj_set_free_num(Pokayoke, 10);
	lv_btn_set_action(Pokayoke, LV_BTN_ACTION_CLICK,Pokayoke_confirmacion);
	lv_btn_set_style(Pokayoke, LV_BTN_STYLE_REL, &Pokayoke_fuera);
	lv_btn_set_style(Pokayoke, LV_BTN_STYLE_PR, &Pokayoke_apachurrado);
	lv_obj_set_size(Pokayoke, 150, 50);
	lv_obj_align(Pokayoke, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 10, 10);

	Pokayoke_texto = lv_label_create(Pokayoke, NULL);
	lv_label_set_text(Pokayoke_texto, "Aceptar");

	texto_confirmacion = lv_label_create(lv_scr_act(), NULL);
	
	lv_obj_align(texto_confirmacion, lv_scr_act(),LV_ALIGN_IN_BOTTOM_MID,-120,-10);
	lv_label_set_style(texto_confirmacion, &lv_style_plain);
	lv_label_set_recolor(texto_confirmacion,true);
	
	
	
	
    /////////////////////////////////////////////////////////////////////////////

    Robot::start_task("SELECTOR", tabWatcher);    
	
} 

lv_obj_t * Titulo; 
lv_obj_t * X_coordinate;
lv_obj_t * Y_coordinate;
lv_obj_t * heading;

void odom_stats(void*ptr){
	lv_obj_clean( lv_scr_act() );
    
	Titulo = lv_label_create(lv_scr_act(),NULL); 
	lv_obj_align(Titulo, lv_scr_act(),LV_ALIGN_IN_TOP_LEFT,100,0);
	

	X_coordinate = lv_label_create(lv_scr_act(), NULL);
	lv_obj_align(X_coordinate, lv_scr_act(),LV_ALIGN_IN_TOP_LEFT,0,100);
	
	Y_coordinate = lv_label_create(lv_scr_act(), NULL);
	lv_obj_align(Y_coordinate, lv_scr_act(),LV_ALIGN_IN_TOP_LEFT,100,100);
	
	heading = lv_label_create(lv_scr_act(), NULL);
	lv_obj_align(heading , lv_scr_act(),LV_ALIGN_IN_TOP_LEFT,200,100);

	
	lv_label_set_text(Titulo, "ODOMETRIA"); 
	

	while (1) {
	
	
	static char buffer_x[32]; 
	static char buffer_y[32]; 
	static char buffer_gyro [32]; 
     
    float X_s = Robot::absGlobalX;
	float Y_s = Robot::absGlobalY;  
	float Heading_s = Robot::absOrientacionDeg; 
    
	snprintf(buffer_x, 32, "X: %.2f", X_s); 
	lv_label_set_text(X_coordinate, buffer_x);

	snprintf(buffer_y, 32, "Y: %.2f", Y_s); 
	lv_label_set_text(Y_coordinate, buffer_y);

	snprintf(buffer_gyro, 32, "Heading (deg): %.2f", Heading_s); 
	lv_label_set_text(heading, buffer_gyro);

	pros::delay(10); 
    
	}

}

}