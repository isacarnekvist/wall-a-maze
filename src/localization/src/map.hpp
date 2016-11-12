#ifndef _MAP_HPP
#define _MAP_HPP

#include <vector>

typedef struct Wall {
    float x1, y1, x2, y2;
} Wall;

class Map {
public:
    Map();
    float distance(float x, float y, float angle) const;
    bool point_approx_on_wall(float x, float y);
    void update_from_laser(std::vector<std::tuple<float, float> > scans, float x, float y, float theta);
    std::vector<Wall> walls;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
private:
    void readWalls();
};

#endif
