#ifndef FILTROS_H
#define FILTROS_H

#include "okapi/api.hpp"

//Namespace para acceder a nuestros filtros
namespace Fil{
extern okapi::EKFFilter kalman; //Filtro de kalman, se utiliza en el pid
extern okapi::EmaFilter ema_filtro; //Filtro ema, se utiliza en las lecturas del ultrasonico

}



#endif