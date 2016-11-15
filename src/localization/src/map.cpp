#include <math.h>
#include <fstream>
#include <sstream>
#include <iostream>

#include "map.hpp"
#include "geometry.hpp"
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

/* Change to return bool if something was updated? */
bool Map::update_from_laser(vector<tuple<float, float> > scans, float x, float y, float theta) {
    vector<tuple<float, float> > cartesian_scans = vector<tuple<float, float> >();
    for (tuple<float, float> &t : scans) {
        float alpha = get<0>(t);
        float dist = get<1>(t);
        cartesian_scans.push_back(make_tuple(x + dist * cos(alpha + theta), y + dist * sin(alpha + theta)));
    }
    bool updated = false;
    vector<Wall> new_walls = detect_walls(cartesian_scans, x, y, 1.2);
    for (Wall &nw : new_walls) {
        bool matched_wall = false;
        for (Wall &ow : walls) {
            tuple<int, Wall*> t = wall_comparison(ow, nw);
            int wall_status = get<0>(t);
            Wall *w = get<1>(t);
            switch (wall_status) {
                case EXTEND:
                    cout << "updating earlier wall" << endl;
                    ow = nw;
                    matched_wall = true;
                    updated = true;
                    break;
                case INSIDE:
                    cout << "wall found inside other wall" << endl;
                    matched_wall = true;
                    break;
                case OUTSIDE:
                    break;
            }
            if (matched_wall) break;
        }
        if (matched_wall) {
            break;
        } else {
            cout << "found new wall" << endl;
            walls.push_back(nw);
            updated = true;
        }
    }
    return updated;
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
