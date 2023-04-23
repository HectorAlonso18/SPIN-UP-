#include "main.h"

//Namespace para acceder a nuestros filtros
namespace Fil{
    okapi::EKFFilter kalman(.000001, okapi::ipow(.2,2)); //Filtro de kalman, se utiliza en el pid
    okapi::EmaFilter ema_filtro(.25); //Filtro ema, se utiliza en las lecturas del ultrasonico

}

