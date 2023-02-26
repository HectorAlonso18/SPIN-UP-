#include "main.h"


std::vector<float> Get_Distances(std::vector<float> Current, float Target) {
	for (auto i = 0; i < Current.size(); i++) {
		Current[i] = abs(Current[i] - Target); 
	}

	return Current; 
}

int Get_Index_min(std::vector<float> Vector) {
	float min = Vector[0]; 
	int index_min = 0; 

	for (auto i = 1; i < Vector.size(); i++) {
		if (Vector[i] < min) {
			min = Vector[i]; 
			index_min = i; 
		}
	}

	return index_min; 
}



float In_range (float value, float min , float max){
	value = value> max ? max : value<min ? min : value; 
	return value; 
}