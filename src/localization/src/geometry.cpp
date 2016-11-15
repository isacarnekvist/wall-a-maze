#include <cmath> // abs(float x)
#include <math.h>
#include <assert.h>
#include <iostream>

#include "geometry.hpp"

using namespace std;

float euclidean(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

bool point_approx_on_line(float x, float y, float x1, float y1, float x2, float y2, float tol) {
    /*
     * Checks projection of point vertex on wall and wall normal
     *
     *                w (wall)
     *              ^
     *  w_norm     /
     *    < _     /
     *       - _ /
     *          +---> v (point)
     *       (0,0)
     */
    static const float ca = cos(M_PI / 2);
    static const float sa = sin(M_PI / 2);

    float wx = (x2 - x1);
    float wy = (y2 - y1);
    float wall_len = sqrt(wx * wx + wy * wy);
    /* Make wall unit vector */
    wx /= wall_len;
    wy /= wall_len;
    float vx = x - x1;
    float vy = y - y1;

    /* Check if within tolerance from wall */
    float wall_proj = vx * wx + vy * wy;
    if (wall_proj < -tol) return false;

    float wnorm_x = ca * wx - sa * wy;
    float wnorm_y = sa * wx + ca * wy;
    float norm_proj = vx * wnorm_x + vy * wnorm_y;

    if (norm_proj > tol or norm_proj < -tol) return false;
    if (wall_proj > wall_len + tol) return false;

    return true;
}

/* end inclusive */
bool is_line(vector<tuple<float, float> > &points, int start, int end, float max_gap) {
    if (end - start == 0) {
        return true;
    }
    float x1 = get<0>(points[start]);
    float y1 = get<1>(points[start]);
    float x2 = get<0>(points[end]);
    float y2 = get<1>(points[end]);
    for (int i = start + 1; i <= end; i++) {
        if (euclidean(get<0>(points[i-1]), get<1>(points[i-1]), get<0>(points[i]), get<1>(points[i])) > max_gap) {
            return false;
        }
        if (!point_approx_on_line(get<0>(points[i]), get<1>(points[i]), x1, y1, x2, y2, 0.05)) {
            return false;
        }
    }
    return true;
}

vector<Wall> detect_walls(
    vector<tuple<float, float> > coords,
    float robot_x,
    float robot_y,
    float distance_limit
) {
    vector<Wall> walls = vector<Wall>();
    int wall_start = 0;
    int wall_end = 0;
    while (wall_end < coords.size()) {
        if(!is_line(coords, wall_start, wall_end, 0.15)) {
            if(wall_end - wall_start == 1) {
                /* A possible single point that could not be linked with others */
                wall_start++;
            } else {
                /* A multi point wall */
                Wall w;
                w.x1 = get<0>(coords[wall_start]);
                w.y1 = get<1>(coords[wall_start]);
                w.x2 = get<0>(coords[wall_end - 1]);
                w.y2 = get<1>(coords[wall_end - 1]);
                if (euclidean(robot_x, robot_y, w.x1, w.y1) < distance_limit ||
                    euclidean(robot_x, robot_y, w.x2, w.y2) < distance_limit) {
                    walls.push_back(w);
                }
                wall_start = wall_end - 1;
                wall_end--;
            }
        }
        wall_end++;
    }
    if (wall_start != wall_end - 1) {
        Wall w;
        w.x1 = get<0>(coords[wall_start]);
        w.y1 = get<1>(coords[wall_start]);
        w.x2 = get<0>(coords[wall_end - 1]);
        w.y2 = get<1>(coords[wall_end - 1]);
        if (euclidean(robot_x, robot_y, w.x1, w.y1) < distance_limit ||
            euclidean(robot_x, robot_y, w.x2, w.y2) < distance_limit) {
            walls.push_back(w);
        }
    }
    return walls;
}

/* Returns null if wall is existing, otherwise allocated (Wall*) */
tuple<int, Wall*> wall_comparison(Wall &a, Wall &b) {
    double v1x = a.x2 - a.x1;
    double v1y = a.y2 - a.y1;
    double v2x1 = b.x1 - a.x1;
    double v2y1 = b.y1 - a.y1;
    double v2x2 = b.x2 - a.x1;
    double v2y2 = b.y2 - a.y1;

    double alpha = atan2(v1y, v1x);
    double ca = cos(-alpha);
    double sa = sin(-alpha);
    double v1x_rot = ca * v1x - sa * v1y;
    double v1y_rot = sa * v1x + ca * v1y;
    double v2x1_rot = ca * v2x1 - sa * v2y1;
    double v2y1_rot = sa * v2x1 + ca * v2y1;
    double v2x2_rot = ca * v2x2 - sa * v2y2;
    double v2y2_rot = sa * v2x2 + ca * v2y2;
    if (v2x1_rot > v2x2_rot) {
        double tmp = v2x2_rot;
        v2x2_rot = v2x1_rot;
        v2x1_rot = tmp;
    }
    if (abs(v2y1_rot) > 0.1 || abs(v2y2_rot) > 0.1) {
        /* Angles not matching */
        return make_tuple(OUTSIDE, (Wall*)0);
    }
    if (v2x1 > v1x || v2x2 < 0.0) {
        /* Wall aligned but outside */
        return make_tuple(OUTSIDE, (Wall*)0);
    }
    if (v2x1_rot > - 0.03 && v2x2_rot < v1x_rot + 0.03) {
        /* Detected wall inside existing wall */
        return make_tuple(INSIDE, (Wall*)0);
    }

    assert (v1x_rot > 0);
    
    /* Rotate back */
    double new_x1 = min(v2x1_rot, 0.0);
    double new_x2 = max(v2x2_rot, v1x_rot);
    double new_y1 = 0;
    double new_y2 = 0;

    double ca_rev = cos(alpha);
    double sa_rev = sin(alpha);

    Wall *w = new Wall();
    w->x1 = ca_rev * new_x1 - sa_rev * new_y1;
    w->y1 = sa_rev * new_x1 + ca_rev * new_y1;
    w->x2 = ca_rev * new_x2 - sa_rev * new_y2;
    w->y2 = sa_rev * new_x2 + ca_rev * new_y2;
    return make_tuple(EXTEND, w);
}
