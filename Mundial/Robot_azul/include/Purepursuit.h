
#include <vector>

/**
 * @return: Regresa un valor entero para representar el signo
 */
int sign(double num);



std::vector<std::vector<double>>Construct_Path(std::vector<double> X, std::vector<double>Y); 

std::vector<std::vector<double>>Construct_New_Path(std::vector<std::vector<double>> Path);  


bool solution_backwards(double currentX, double  currentY,std::vector<std::vector<double>> Path, std::vector<double> goal_pt , int iterator); 

std::vector<double> goal_search(double X, double Y, std::vector < std::vector<double>> Path, double lookAheadDistance, int lastFoundIndex, int iterator); 




