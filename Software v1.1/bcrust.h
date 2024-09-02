#ifndef B_CRUST_H
#define B_CRUST_H

#include <vector>

/*typedef struct my_point_2
{
	double x;
	double y;
} my_point_2;*/

void calculate_b_crust(const std::vector<my_point_2>& pointsin, std::vector<my_point_2>& pointsout);

#endif