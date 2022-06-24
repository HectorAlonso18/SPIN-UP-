#include "main.h"
#include <vector>
#include <math.h>
#include <numeric>
#include <cmath>

#define SQD(n) pow(n,2)


/**
 * @return: An integer value (1, -1 or 0) that represents the sign of the input x
 */
int sign(double x) {
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

/**
 * @desc: Calculates the heading required for our Robot to continuously face forward while moving from @param p1 to @param 
  p2. This is important because it allows our intakes to face any balls we might run into instead of knocking them aside
 * @param p1: A vector of length two in the format {X, Y} that contains Robot::X and Robot::Y
 * @param p2: A vector of length two in the format {X, Y} that contains the X and Y coordinate of the target point
 * @return: Returns the slope between two points in degrees, which can be translated easily into Robot heading. 
 */
double get_degrees(std::vector<double> p1, std::vector<double> p2) {
  double x = p1[0] - p2[0];
  double y = p1[1] - p2[1];
  return atan2(-x, y) * 180 / M_PI;
}

/**
 * @param p1: A vector of length two in the format {X, Y} that contains Robot::X and Robot::Y
 * @param p2: A vector of length two in the format {X, Y} that contains the X and Y coordinate of the target point
 * @return: Provides the distance between the two points using the standard distance formula
 */
double distance(std::vector<double> p1, std::vector<double> p2) {
  return hypot(pow((p1[0]-p2[0]),2) , pow((p1[1]-p2[1]),2) );  
}

/**
  * @desc: Calculates the next step of our pure pursuit mvoement given a segment of the waypoint path and our current
    position
  * @param start: A vector of length two in the format {X, Y} containing the first point of a two-point segment of the 
    waypoint path
  * @param end: A vector of length two in the format {X, Y} containing the second point of a two-point segment of the 
    waypoint path
  * @param cur: A vector of length two in the format {X, Y} that contains Robot::x and Robot::y
  * @param radius: A scalar representing how far our lookahead distance should be- in other words, how early we should start 
    turning in anticipation of a change in direction
 * @return: The intersection between a circle centered on our Robot's current position with radius @param radius and the 
    line formed between @param start and @param end
 */
  std::vector<double> get_intersection(std::vector<double> start, std::vector<double> end, std::vector<double> cur, double radius) {
      std::vector<double> p1 {(start[0] - cur[0] ), (start[1] - cur[1])};
      std::vector<double> p2 {(end[0] - cur[0]) , (end[1] - cur[1])};

      double dx = p2[0] - p1[0];
      double dy = p2[1] - p1[1];
      float d = hypot(dx,dy); 
      float D = p1[0] * p2[1] - p2[0] * p1[1];
      float discriminant = abs(SQD(radius) * SQD(d) - SQD(D));

      float x1 = (D * dy + sign(dy) * dx * sqrt(discriminant)) / SQD(d);
      float y1 = (-D * dx + abs(dy) * sqrt(discriminant)) / SQD(d);
      float x2 = (D * dy - sign(dy) * dx * sqrt(discriminant)) / SQD(d);
      float y2 = (-D * dx - abs(dy) * sqrt(discriminant)) / SQD(d);
      /* Above calculations can be explained and proven here: https://mathworld.wolfram.com/Circle-LineIntersection.html */

      std::vector<double> intersection1 {x1, y1};
      std::vector<double> intersection2 {x2, y2};

      float distance1 = distance(p2, intersection1);
      float distance2 = distance(p2, intersection2);

      std::vector<double> calc1 {(x1 + cur[0]), (y1 + cur[1])};
      std::vector<double> calc2 {(x2 + cur[0]), (y2 + cur[1])};

      if (distance1 < distance2) return calc1;
      if (distance1 > distance2) return calc2;
  /* There are always two intersections between a circle and a line- if they do not appear visually, they are imaginary.
  We select the intersection that is closer to the end of the waypoint path so we are always moving forwards*/
}