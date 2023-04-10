#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#define M_PI		3.14159265358979323846

//smoothea un path ya prestablecido 
std::vector<std::vector<double>> smoother(std::vector<std::vector<double>>Path, double a, double b, double tolerance) {

    std::vector<std::vector<double>>newPath = Path; 
    double change = tolerance; 
    while (change >= tolerance) {
        change = 0; 
        for (int i = 1; i < Path.size() - 1; i++) {
       
            
            for (int j = 0; j < Path[i].size(); j++) {
               
                double aux = newPath[i][j]; 
                newPath[i][j] += a * (Path[i][j] - newPath[i][j]) + b *
                (newPath[i - 1][j] + newPath[i + 1][j] - (2 * newPath[i][j])); 
                change += std::abs(aux - newPath[i][j]); 
            }
        }
    }
    return newPath; 
}


//Construye un path dandole un vector de coordenadas X y uno de coordenadas Y
std::vector < std::vector<double>> Construct_Path(std::vector<double> X, std::vector<double>Y) {
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

std::vector<std::vector<double>> injecting_points(std::vector < std::vector<double>>Path) {
    int spacing = 6; 
    std::vector<std::vector<double>> new_points;

    std::cout << "\n\n\n\n"; 
    
    for (int i = 0; i <Path.size()-1 ; i++) {

      //  std::cout << "\n Start Point: ["<<i<<"]" << Path[i][0] << ", " << Path[i][1];
      //  std::cout << "\n End Point: ["<<i+1<<"]" << Path[i + 1][0] << ", " << Path[i + 1][1];


double vectorcitoX = Path[i + 1][0] - Path[i][0];

//  std::cout << "\nVectorcitoX: ["<<i<<"] " << vectorcitoX;

double vectorcitoY = Path[i + 1][1] - Path[i][1];
//  std::cout << "\nVectorcitoY: ["<<i<<"] " << vectorcitoY;

double magnitud = sqrt(pow(vectorcitoX, 2) + pow(vectorcitoY, 2));
//  std::cout << "\nMagnitud: ["<<i<<"] " << magnitud;

int num_points_that_fit = ceil(magnitud / spacing);
//  std::cout << "\nPuntos: ["<<i<<"] " << num_points_that_fit;

double vectorcito_x_normalizado = (vectorcitoX) / (magnitud);
//  std::cout << "\nx_normalizado: ["<<i<<"]" << vectorcito_x_normalizado;

double vectorcito_y_normalizado = (vectorcitoY) / (magnitud);
//  std::cout << "\ny_normalizado: ["<<i<<"]" << vectorcito_y_normalizado;

vectorcitoX = vectorcito_x_normalizado * spacing;
vectorcitoY = vectorcito_y_normalizado * spacing;
//  std::cout << "\nVectorcitoX_nuevo: ["<<i<<"] " << vectorcitoX;
//  std::cout << "\nVectorcitoY_nuevo: ["<<i<<"] " << vectorcitoY;

for (int j = 0; j < num_points_that_fit; j++) {
    std::vector<double>vector_aux;
    for (int k = 0; k < 1; k++) {
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

double distance_points(std::vector<double> p1, double p2_x, double p2_y);

double distance_between_points_in_Path(std::vector<std::vector<double>>Path, int index) {
    return hypot(Path[index][0] - Path[index - 1][0], Path[index][1] - Path[index - 1][1]);
}


int sgn(double num);

void goal_search(std::vector < std::vector<double>> Path, std::vector<double>CurrentPosition, double lookAheadDistance, int lastFoundIndex);




std::vector<double> distance_points_velocity(std::vector<std::vector<double>> Path) {
    std::vector<double> distance_at_point(Path.size());
    distance_at_point[0] = 0;
    for (auto i = 1; i < Path.size(); i++) {
        distance_at_point[i] = distance_between_points_in_Path(Path, i) + distance_at_point[i - 1];
    }

    return distance_at_point;
}

double get_curvature(std::vector<double>Q, std::vector<double>P, std::vector<double>R){
    double x1 = P[0]+.001; 
    double y1 = P[1]; 
    
    double x2 = Q[0];
    double y2 = Q[1];

    double x3 = R[0];
    double y3 = R[1];

    double k1 = .5 * (pow(x1, 2) + pow(y1, 2) - pow(x2, 2) - pow(y2, 2)) / (x1 - x2); 
    double k2 = (y1 - y2) / (x1 - x2); 
    double b = .5 * (pow(x2, 2) - 2 * x2 * k1 + pow(y2, 2) - pow(x3, 2) + 2 * x3 * k1 - pow(y3, 2)) / (x3 * k2 - y3 + y2 - x2 * k2); 
    double a = k1 - k2 * b; 
    double r = hypot(x1 - a, y1 - b); 
    
    double curvature = 1 / r;

    curvature = isnan(curvature) ? 0 : curvature; 

    return curvature; 
 }


std::vector<double> get_curvature_at_point(std::vector<std::vector<double>> Path) {
    
    

    std::vector<double>curvature(Path.size()); 
    curvature[0] = 0;
    curvature[Path.size() - 1] =0; 

    for (auto i = 1; i < Path.size()-1; i++) {
        std::vector<double> Q = Path[i-1];
        std::vector<double> P = Path[i];
        std::vector<double> R = Path[i+1];

        curvature[i] = get_curvature(Q, P, R); 
  
    }

    return curvature; 
}
 




void print_vector(std::vector<double>vector) {
    for (int i = 0; i<vector.size(); i++) {
        std::cout << "\n vector[" << i << "] " << vector[i]; 
    }
}

std::vector<double> X_travel = { -0.313043, -2.817387, 8.139118, 19.721709, 30.365171, 41.321676000000004, 41.634719000000004, 40.69559, 42.886891000000006, 43.199934, 43.82602, 55.095568, 64.799901, 64.486858, 55.721654, 41.947762000000004 };
std::vector<double> Y_travel = { 2.191301, 14.713021000000001, 25.356483, 15.026064000000002, 15.965193000000001, 15.339107, 27.234741, 39.443418, 53.217310000000005, 65.425987, 80.139008, 80.765094, 83.582481, 92.660728, 102.99114700000001, 108.31287800000001 };


void path_new_example(void) {
    std::vector<std::vector<double>> path_vex = Construct_Path(X_travel, Y_travel);

    std::cout << "\nPath normal";
    std::cout << "\nsize of path: " << path_vex.size() << "\n";
    for (int i = 0; i < path_vex.size(); i++) {
        std::cout << "\n" << path_vex[i][0] << "," << path_vex[i][1];

    }

    path_vex = injecting_points(path_vex);

    std::cout << "\nPath injecyado";
    std::cout << "\nsize of path: " << path_vex.size() << "\n";
    for (int i = 0; i < path_vex.size(); i++) {
        std::cout << "\n" << path_vex[i][0] << "," << path_vex[i][1];

    }


    std::vector<std::vector<double>> curveando = smoother(path_vex, 1 - .75, .75, .001);

    std::cout << "\nPath smooth";
    for (int i = 0; i < curveando.size(); i++) {
        std::cout << "\n" << curveando[i][0] << "," << curveando[i][1];

    }

}

int main() {

   
     
    std::vector<std::vector<double>> path_vex = Construct_Path(X_travel, Y_travel);

    path_vex = injecting_points(path_vex);

    path_vex= smoother(path_vex, 1 - .75, .75, .001);

    std::cout << path_vex.size() << "\n"; 

    
    std::vector<double> distances_vex = distance_points_velocity(path_vex); 
   
 
    std::vector<double> curvatures_vex = get_curvature_at_point(path_vex);

	return 0; 
}




double distance_points(std::vector<double> p1, double p2_x, double p2_y) {

	return hypot(p2_x - p1[0], p2_y - p1[1]);
	
}

int sgn(double num) {
	return num >= 0 ? 1 : -1; 
}

//Calculos del pure pursuit
void goal_search(std::vector < std::vector<double>> Path, std::vector<double>CurrentPosition, double lookAheadDistance, int lastFoundIndex) {

    double currentX = CurrentPosition[0]; 
    double currentY = CurrentPosition[1]; 

    bool intersectFound = false; 
    int startingIndex = lastFoundIndex; 

    for (int i = startingIndex; i < Path.size(); i++) {
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
            double sol_x1 = (D * delta_Y + sgn(delta_Y) * delta_X * sqrt(discriminant)) / pow(dr, 2); 
            double sol_x2 = (D * delta_Y - sgn(delta_Y) * delta_X * sqrt(discriminant)) / pow(dr, 2);

            double sol_y1 = (-D * delta_X + abs(delta_Y) * sqrt(discriminant) ) / pow(dr,2 );
            double sol_y2 = (-D * delta_X - abs(delta_Y) * sqrt(discriminant)) / pow(dr, 2);

            std::vector<double>sol_pt1 = { sol_x1 + currentX, sol_y1 + currentY }; 
            std::vector<double>sol_pt2 = { sol_x2 + currentX, sol_y2 + currentY }; 

            std::vector<double> goal_pt = { NULL,NULL }; 

            double minX = std::min(Path[i][0], Path[i+1][0]);
            double minY = std::min(Path[i][1], Path[i+1][1]);

            double maxX = std::max(Path[i][0], Path[i+1][0]); 
            double maxY = std::max(Path[i][1], Path[i+1][1]);

            //Si se encuentra al menos que una solucion es correcta

            if  (  ( (minX <= sol_pt1[0] && maxX >= sol_pt1[0]) && (minY <= sol_pt1[1] && maxY >= sol_pt1[1]) ) ||
               ( (minX <= sol_pt2[0] && maxX >= sol_pt2[0]) && (minY <= sol_pt2[1] && maxY >= sol_pt2[1])  )   )
            {   
                //Se encuentra la interseccion 
                intersectFound = true; 

                //si ambas soluciones son correctas
                if (((minX <= sol_pt1[0] && maxX >= sol_pt1[0]) && (minY <= sol_pt1[1] && maxY >= sol_pt1[1])) &&
                    ((minX <= sol_pt2[0] && maxX >= sol_pt2[0]) && (minY <= sol_pt2[1] && maxY >= sol_pt2[1]))) {
                    
                    //Decidir la solucion dependiendo de cual este mas cerca
                    if ( distance_points(sol_pt1, Path[i + 1][0], Path[i + 1][1]) < distance_points(sol_pt2, Path[i + 1][0], Path[i + 1][1]) ) {
                        goal_pt = sol_pt1; 
                    }

                    else {
                        goal_pt = sol_pt2;
                    }

                }

                //Si solo una soluicon es la correcta
                else {
                    //Asignar la solucion correcta
                    if ( ((minX <= sol_pt1[0]) && (maxX >= sol_pt1[0])) && ((minY <= sol_pt1[1]) && (maxY >= sol_pt1[1])) ) {
                        goal_pt = sol_pt1; 
                    }
                    else {
                        goal_pt = sol_pt2; 
                    }
                }
                
                //Romper ciclo solo si la solucion encontrada est� m�s cerca del siguiente punto que la posicion actual
                if (distance_points(goal_pt, Path[i + 1][0], Path[i + 1][1]) < distance_points({ currentX,currentY }, Path[i + 1][0], Path[i + 1][1])) {
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
        

        //Aqu� podriamos poner nuestra funcion de odometria 
        //La funcion que usamos esta correcta, solamente tenemos que quitar las ultimas lineas
        //que paran al chassis 
        

    }


}

