#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H
#include <vector>

int sign(double x);

double distance(std::vector<double> p1, std::vector<double> p2);
double get_degrees(std::vector<double> p1, std::vector<double> p2);

std::vector<double> get_intersection(std::vector<double> start, std::vector<double> end, std::vector<double> cur, double lookAheadDis);




#endif