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
    std::vector<Wall> walls;
private:
    void readWalls();
};

#endif
