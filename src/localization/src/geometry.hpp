#ifndef _GEOMETRY_H
#define _GEOMETRY_H

#include <tuple>
#include <vector>

#include "map.hpp"

using namespace std;

/* Returns null if wall is existing, otherwise allocated (Wall*) */
Wall *wall_comparison(Wall &a, Wall &b);

vector<Wall> detect_walls(
    vector<tuple<float, float> > coords,
    float robot_x,
    float robot_y,
    float distance_limit
);

#endif
