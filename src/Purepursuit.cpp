#include "main.h"
#include <vector>
#include <math.h>
#include <numeric>
#include <cmath>

#define SQD(n) pow(n,2)


int sign(double x) {
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}


double get_degrees(std::vector<double> p1, std::vector<double> p2) {
  double x = p1[0] - p2[0];
  double y = p1[1] - p2[1];
  return atan2(-x, y) * 180 / M_PI;
}


double distance(std::vector<double> p1, std::vector<double> p2) {
  return hypot(pow((p1[0]-p2[0]),2) , pow((p1[1]-p2[1]),2) );  
}


std::vector<double> get_insertion(std::vector<double> start, std::vector<double> end, std::vector<double> current_coordinates, double lookAheadDis){
  std::vector<double> p1 {(start[0] - current_coordinates[0] ), (start[1] - current_coordinates[1])};
  std::vector<double> p2 {(end[0] - current_coordinates[0]) , (end[1] - current_coordinates[1])};

  double dx = p2[0] - p1[0];
  double dy = p2[1] - p1[1];
  float d = hypot(dx,dy); 
  float D = p1[0] * p2[1] - p2[0] * p1[1];
  float discriminant = abs(SQD(lookAheadDis) * SQD(d) - SQD(D));

  
  float sol_x1 = (D * dy + sign(dy) * dx * sqrt(discriminant)) / SQD(d);
  float sol_y1 = (-D * dx + abs(dy) * sqrt(discriminant)) / SQD(d);
  float sol_x2 = (D * dy - sign(dy) * dx * sqrt(discriminant)) / SQD(d);
  float sol_y2 = (-D * dx - abs(dy) * sqrt(discriminant)) / SQD(d);
  
  std::vector<double> intersection_1 {sol_x1,sol_y1};
  std::vector<double> intersection_2 {sol_x2,sol_y2};

  float distance_1 = distance(p2, intersection_1);
  float distance_2 = distance(p2,intersection_2);

  std::vector<double> calc1 {(sol_x1+current_coordinates[0]),(sol_y1+current_coordinates[1])};
  std::vector<double> calc2 {(sol_x2+current_coordinates[0]),(sol_y2+current_coordinates[1])};

  if(distance_1 <distance_2) return calc1;
  if(distance_1 > distance_2) return calc2;


  return {0};

}


