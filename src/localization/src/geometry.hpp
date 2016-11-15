#ifndef _GEOMETRY_H
#define _GEOMETRY_H

#include <tuple>
#include <vector>

#include "map.hpp"

using namespace std;

const static int OUTSIDE = 0;
const static int INSIDE = 1;
const static int EXTEND = 2;
tuple<int, Wall*> wall_comparison(Wall &a, Wall &b);

vector<Wall> detect_walls(
    vector<tuple<float, float> > coords,
    float robot_x,
    float robot_y,
    float distance_limit
);

#endif
