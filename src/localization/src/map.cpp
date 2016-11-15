#include <math.h>
#include <fstream>
#include <sstream>
#include <iostream>

#include "map.hpp"
#include "constants.hpp"

using namespace std;

Map::Map() {
    walls = vector<Wall>();
    this->min_x = INF;
    this->max_x = -INF;
    this->min_y = INF;
    this->max_y = -INF;
    readWalls();
}

bool point_approx_on_line(float x, float y, float x1, float y1, float x2, float y2);
bool point_approx_on_line(float x, float y, float x1, float y1, float x2, float y2, float tol);

float euclidean(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

float Map::distance(float x0, float y0, float angle) const {
    float ca = cos(angle);
    float sa = sin(angle);
    float y1, y2; /* Every wall checked will have these y-values */
    float x1, x2; /* Every wall checked will have these x-values */
    float min_dist = INF;

    for (Wall w : walls) {
        /* Rotate beam along positive x-axis */
        y1 = (w.y1 - y0) * ca - (w.x1 - x0) * sa;
        y2 = (w.y2 - y0) * ca - (w.x2 - x0) * sa;
        if (y1 * y2 > 0.0) {
            /* Obstacle does not intersect positive x-axis */
            continue;
        }
        x1 = (w.x1 - x0) * ca + (w.y1 - y0) * sa;
        x2 = (w.x2 - x0) * ca + (w.y2 - y0) * sa;

        if ((x1 * y2 - x2 * y1) * y2 < 0.0) { 
            /* Fancy triangle check */
            continue;
        }
        /* Invariant: Intersection of current wall and beam */
        float dist = (x1 * (y2 - y1) - y1 * (x2 - x1)) / (y2 - y1);
        if (dist > 0.0) {
            min_dist = min(
                min_dist,
                dist
            );
        }
    }
    return min_dist;
}

/* end inclusive */
bool is_line(vector<tuple<float, float> > &points, int start, int end) {
    float x1 = get<0>(points[start]);
    float y1 = get<1>(points[start]);
    float x2 = get<0>(points[end]);
    float y2 = get<1>(points[end]);
    for (int i = start + 1; i < end; i++) {
        if (!point_approx_on_line(get<0>(points[i]), get<1>(points[i]), x1, y1, x2, y2, 0.05)) {
            return false;
        }
    }
    return true;
}

/* Change to return bool if something was updated? */
bool Map::update_from_laser(vector<tuple<float, float> > scans, float x, float y, float theta) {
    bool res = false;
    return res;
}

bool Map::point_approx_on_wall(float x, float y) {
    /* Checks if point (x, y) is on (or almost) on any wall in the map */
    for (Wall &w : walls) {
        if (point_approx_on_line(x, y, w.x1, w.y1, w.x2, w.y2)) return true;
    }
    return false;
}

void Map::readWalls() {
    const char *map_path = getenv("MAP_PATH");
    if (!map_path) {
        cerr << "Environment variable MAP_PATH not set!" << endl;
        exit(-1);
    }
    ifstream myfile (map_path);
    float x;
    if (!myfile.is_open()) {
        cerr << "Could not open map file: " << map_path << endl;
        exit(-1);
    }
    string line;
    float x1, y1, x2, y2;
    while (getline(myfile, line)) {
        if (line[0] == '#') {
            continue;
        }
        istringstream in (line);
        in >> x1 >> y1 >> x2 >> y2;
        this->min_x = min(min(this->min_x, x1), x2);
        this->max_x = max(max(this->max_x, x1), x2);
        this->min_y = min(min(this->min_y, y1), y2);
        this->max_y = max(max(this->max_y, y1), y2);
        Wall w = {x1, y1, x2, y2};
        walls.push_back(w);
    }
    myfile.close();
}

bool point_approx_on_line(float x, float y, float x1, float y1, float x2, float y2) {
    return point_approx_on_line(x, y, x1, y1, x2, y2, 0.1);
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
