#include "main.h"
#include "parametros.h"
#include <vector>
#include <math.h>
#include <numeric>
#include <cmath>

#define SQD(n) pow(n,2)


int sign(double num) {
  return num>=0 ?1 :-1; 
}


std::vector<std::vector<double>>Construct_Path(std::vector<double> X, std::vector<double>Y){
  
  std::vector < std::vector<double>> Path;
  for (int i = 0; i < X.size(); i++) {
    std::vector<double>vector_aux; 

    for (int j = 0; j < 1; j++) {
      vector_aux.push_back(X[i]);
      vector_aux.push_back(Y[i]);
    }

    Path.push_back(vector_aux);

  }

  return Path; 
} 


std::vector<std::vector<double>> Injecting_points(std::vector < std::vector<double>>Path){
  int spacing = 6; 
  std::vector<std::vector<double>> new_points;

  std::cout << "\n\n\n\n"; 
    
  for (int i = 0; i <Path.size()-1 ; i++) {
  
    double vectorcitoX = Path[i+1][0] - Path[i][0];
    double vectorcitoY = Path[i+1][1] - Path[i][1];
 
    double magnitud = sqrt( pow(vectorcitoX, 2) + pow(vectorcitoY, 2)); 
    
    int num_points_that_fit = ceil(magnitud / spacing); 
    

    double vectorcito_x_normalizado = (vectorcitoX) / (magnitud); 
    double vectorcito_y_normalizado = (vectorcitoY) / (magnitud);
  

    vectorcitoX = vectorcito_x_normalizado * spacing; 
    vectorcitoY = vectorcito_y_normalizado * spacing; 
 
    for (int j = 0; j < num_points_that_fit; j++) {
      std::vector<double>vector_aux;
      for (int k = 0; k < 1; k++){
        vector_aux.push_back((Path[i][0]) + vectorcitoX * j);
        vector_aux.push_back((Path[i][1]) + vectorcitoY * j);
      }

      new_points.push_back(vector_aux); 
    }

  }

  std::vector<double>vector_final; 

  vector_final.push_back(Path[Path.size() - 1][0]); 
  vector_final.push_back(Path[Path.size() - 1][1]);

  new_points.push_back(vector_final); 

  return new_points; 
} 

std::vector<std::vector<double>> Smoother(std::vector<std::vector<double>>Path, double a, double b, double tolerance){
  std::vector<std::vector<double>>newPath = Path; 
  double change = tolerance; 
  while (change >= tolerance) {
    change = 0; 
    for (int i = 1; i < Path.size() - 1; i++) {
      
      for (int j = 0; j < Path[i].size(); j++) {
        double aux = newPath[i][j]; 
        newPath[i][j] += a * (Path[i][j] - newPath[i][j]) + b *(newPath[i - 1][j] + newPath[i + 1][j] - (2 * newPath[i][j])); 
        change += std::abs(aux - newPath[i][j]); 
      }
    }
  }
      return newPath; 
}

std::vector<std::vector<double>>Construct_New_Path(std::vector<std::vector<double>> Path){
  Path= Injecting_points(Path);
  Path= Smoother(Path, 1-.75, .75, .001);
  return Path; 
} 


double distance_points(std::vector<double> p1, double p2_x, double p2_y){
  return hypot(p2_x - p1[0], p2_y - p1[1]);
}


//helper functions for goal_search 

bool exist_any_solution (double minX, double minY, double maxX,double maxY, std::vector<double> sol_pt1, std::vector<double> sol_pt2){
  return  ( (minX <= sol_pt1[0] && maxX >= sol_pt1[0]) && (minY <= sol_pt1[1] && maxY >= sol_pt1[1]) ) ||( (minX <= sol_pt2[0] && maxX >= sol_pt2[0]) && (minY <= sol_pt2[1] && maxY >= sol_pt2[1])); 
}

bool exist_both_solution(double minX, double minY, double maxX,double maxY, std::vector<double> sol_pt1, std::vector<double> sol_pt2){
  return ((minX <= sol_pt1[0] && maxX >= sol_pt1[0]) && (minY <= sol_pt1[1] && maxY >= sol_pt1[1])) &&((minX <= sol_pt2[0] && maxX >= sol_pt2[0]) && (minY <= sol_pt2[1] && maxY >= sol_pt2[1])); 
}

bool check_solution (double minX, double minY, double maxX,double maxY, std::vector<double> sol_pt1){
  return ((minX <= sol_pt1[0]) && (maxX >= sol_pt1[0])) && ((minY <= sol_pt1[1]) && (maxY >= sol_pt1[1]));
}

bool solution_backwards(double currentX, double  currentY,std::vector<std::vector<double>> Path, std::vector<double> goal_pt , int iterator){
  return distance_points(goal_pt, Path[iterator + 1][0], Path[iterator+ 1][1]) < distance_points({ currentX,currentY }, Path[iterator+ 1][0], Path[iterator + 1][1])  ; 
}


std::vector<double>  goal_search(double X, double Y, std::vector < std::vector<double>> Path,double lookAheadDistance, int lastFoundIndex, int iterator){
  
  int i = iterator; 

  double currentX = X; 
  double currentY = Y; 

  bool intersectFound = false; 
  int startingIndex = lastFoundIndex; 

  std::vector<double> goal_pt = { currentX,currentY }; 
    
  double x1 = Path[i][0] - currentX; 
  double y1 = Path[i][1] - currentY; 

  double x2 = Path[i+1][0] - currentX;
  double y2 = Path[i+1][1] - currentY;

  double delta_X = x2 - x1; 
  double delta_Y = y2 - y1; 

  double dr = hypot(delta_X, delta_Y); 
        
  double D = (x1 * y2) - (x2 * y1); 
  double discriminant = (pow(lookAheadDistance, 2) * pow(dr, 2)) - pow(D, 2); 
    
  if (discriminant<0){return goal_pt;}
  
  double sol_x1 = (D * delta_Y + sign(delta_Y) * delta_X * sqrt(discriminant)) / pow(dr, 2); 
  double sol_x2 = (D * delta_Y - sign(delta_Y) * delta_X * sqrt(discriminant)) / pow(dr, 2);

  double sol_y1 = (-D * delta_X + abs(delta_Y) * sqrt(discriminant) ) / pow(dr,2 );
  double sol_y2 = (-D * delta_X - abs(delta_Y) * sqrt(discriminant)) / pow(dr, 2);

  std::vector<double>sol_pt1 = { sol_x1 + currentX, sol_y1 + currentY }; 
  std::vector<double>sol_pt2 = { sol_x2 + currentX, sol_y2 + currentY }; 

      
  double minX = std::min(Path[i][0], Path[i+1][0]);
  double minY = std::min(Path[i][1], Path[i+1][1]);

  double maxX = std::max(Path[i][0], Path[i+1][0]); 
  double maxY = std::max(Path[i][1], Path[i+1][1]);
 
  if(!exist_any_solution(minX, minY, maxX, maxY, sol_pt1, sol_pt2)){
    intersectFound = false;
    //Avanzar al ultimo punto guardado
    goal_pt = { Path[lastFoundIndex][0], Path[lastFoundIndex][1] };
    return goal_pt; 
  }

  intersectFound = true;

  if(! exist_both_solution(minX, minY, maxX, maxY, sol_pt1,sol_pt2)){
    goal_pt = check_solution(minX, minY, maxX, maxY, sol_pt1) ? sol_pt1 : sol_pt2; 
    return goal_pt; 
  }

  goal_pt = distance_points(sol_pt1, Path[i + 1][0], Path[i + 1][1]) < distance_points(sol_pt2, Path[i + 1][0], Path[i + 1][1]) ? sol_pt1 : sol_pt2; 
  

  return goal_pt; 
} 





/* primera version 
for (int i = startingIndex; i < Path.size(); i++) {
    currentX = Robot::absGlobalX; 
    currentY = Robot::absGlobalY; 
    
    double x1 = Path[i][0] - currentX; 
    double y1 = Path[i][1] - currentY; 

    double x2 = Path[i+1][0] - currentX;
    double y2 = Path[i+1][1] - currentY;

    double delta_X = x2 - x1; 
    double delta_Y = y2 - y1; 

    double dr = hypot(delta_X, delta_Y); 
        
    double D = (x1 * y2) - (x2 * y1); 
    double discriminant = (pow(lookAheadDistance, 2) * pow(dr, 2) - pow(D, 2)); 

        
    if (discriminant >= 0) {

      //There is a pottencial solution
      double sol_x1 = (D * delta_Y + sign(delta_Y) * delta_X * sqrt(discriminant)) / pow(dr, 2); 
      double sol_x2 = (D * delta_Y - sign(delta_Y) * delta_X * sqrt(discriminant)) / pow(dr, 2);

      double sol_y1 = (-D * delta_X + abs(delta_Y) * sqrt(discriminant) ) / pow(dr,2 );
      double sol_y2 = (-D * delta_X - abs(delta_Y) * sqrt(discriminant)) / pow(dr, 2);

      std::vector<double>sol_pt1 = { sol_x1 + currentX, sol_y1 + currentY }; 
      std::vector<double>sol_pt2 = { sol_x2 + currentX, sol_y2 + currentY }; 

      

      double minX = std::min(Path[i][0], Path[i+1][0]);
      double minY = std::min(Path[i][1], Path[i+1][1]);

      double maxX = std::max(Path[i][0], Path[i+1][0]); 
      double maxY = std::max(Path[i][1], Path[i+1][1]);

      //Si se encuentra al menos que una solucion es correcta

      if  ( exist_any_solution(minX, minY, maxX, maxY, sol_pt1, sol_pt2)){   
        
        //Se encuentra la interseccion 
        intersectFound = true; 

        //si ambas soluciones son correctas
        if (exist_both_solution(minX, minY, maxX, maxY, sol_pt1,sol_pt2) ){

          //Decidir la solucion dependiendo de cual este mas cerca
          if ( distance_points(sol_pt1, Path[i + 1][0], Path[i + 1][1]) < distance_points(sol_pt2, Path[i + 1][0], Path[i + 1][1]) ) {
            goal_pt = sol_pt1; 
          }

          else {
            goal_pt = sol_pt2;
          }

        }

        //Si solo una solucion es la correcta
        else {
          //Asignar la solucion correcta
          if ( check_solution(minX, minY, maxX, maxY, sol_pt1, sol_pt2) ) {
            goal_pt = sol_pt1; 
          }
          
          else {
            goal_pt = sol_pt2; 
          }
        }
                

        //Romper ciclo solo si la solucion encontrada está más cerca del siguiente punto que la posicion actual
        if ( solution_backwards(currentX, currentY, Path, goal_pt, i)) {
          lastFoundIndex = i; 
          break; 
        }

        //De lo contrario aumentar index y seguir avanzando
        else {
          lastFoundIndex = i + 1; 
        }

      }

      //Si ya no hay intersecciones
      else {
        intersectFound = false;
        //Avanzar al ultimo punto guardado
        goal_pt = { Path[lastFoundIndex][0], Path[lastFoundIndex][1] };
      }
            
    }

    return goal_pt;
        
  }
*/