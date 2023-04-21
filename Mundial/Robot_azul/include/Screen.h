#pragma once

#include <string>

//configuracion
#define HUE 183
#define DEFAULT 1
#define AUTONOMOS "Ofensiva", "Defensiva"

namespace screen{

extern int autonomo;
extern const char *b[];
void init(int hue = HUE, int default_auton = DEFAULT, const char **autonomos = b);
void odom_stats(void*ptr); 
}


