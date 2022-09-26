#include "main.h"
#include <vector>
#include <math.h>
#include <numeric>
#include <cmath>

#define SQD(n) pow(n,2)


int sign(double x) {
  if (x >= 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}


double distance(std::vector<double> p1, std::vector<double> p2) {
  return hypot(pow((p1[0]-p2[0]),2) , pow((p1[1]-p2[1]),2) );  
}


std::vector<double> get_intersection(std::vector<double> start, std::vector<double> end, std::vector<double> current_coordinates, double lookAheadDis){
  
  std::vector<double> p1 {start[0] - current_coordinates[0], start[1] - current_coordinates[1]};
  std::vector<double> p2 {end[0]- current_coordinates[0], end[1]- current_coordinates[1]};

  /*
   x1_offset= p1[0];
   y1_offset = p1[1];

   x2_offset = p2[0];
   y2_offset = p2[1];
  */

  double dx= p2[0] - p1[0];
  double dy= p2[1] - p2[1];
  float dr= hypot(dx,dy);
  float D = p1[0] * p2[1] - p2[0]*p1[1];
  float discriminant = abs(SQD(lookAheadDis) * SQD(dr) - SQD(D));
  
  float x1 = (D * dy + sign(dy) * dx * sqrt(discriminant)) / SQD(dr);
  float y1 = (-D * dx + abs(dy) * sqrt(discriminant)) / SQD(dr);

  float x2 = (D * dy - sign(dy) * dx * sqrt(discriminant)) / SQD(dr);
  float y2 = (-D * dx - abs(dy) * sqrt(discriminant)) / SQD(dr);
  
  /* https://mathworld.wolfram.com/Circle-LineIntersection.html */
  
  std::vector<double> intersection1{x1,y1};
  std::vector<double> intersection2{x2,y2};

  float distance_1= distance(p2, intersection1);
  float distance_2= distance(p2,intersection2);
  
  std::vector<double> calc1 {(x1+current_coordinates[0]),(y1+current_coordinates[1])};
  std::vector<double> calc2 {(x2+current_coordinates[0]),(y2+current_coordinates[1])};

  if(distance_1 <distance_2){
    return calc1;
  }

  else if(distance_1 > distance_2){
    return calc2;
  }

  else{
    return {0};
  }

}


