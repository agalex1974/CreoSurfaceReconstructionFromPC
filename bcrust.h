#ifndef B_CRUST_H
#define B_CRUST_H

#include <vector>
#include "MCLSEXST.h"

void calculate_b_crust(const std::vector<CPointEx>& pointsin, std::vector<std::vector<CPointEx>>& pointsoutPolygons);

#endif