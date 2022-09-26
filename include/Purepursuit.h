
#include <vector>

/**
 * @return: Regresa un valor entero para representar el signo
 */
int sign(double x);

double distance(std::vector<double> p1, std::vector<double> p2);
double get_degrees(std::vector<double> p1, std::vector<double> p2);

std::vector<double> get_intersection(std::vector<double> start, std::vector<double> end, std::vector<double> current_coordinates, double lookAheadDis);


