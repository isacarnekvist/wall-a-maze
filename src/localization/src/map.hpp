#ifndef _MAP_HPP
#define _MAP_HPP

#include <tuple>
#include <vector>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

typedef struct Wall {
    float x1, y1, x2, y2;
} Wall;

typedef struct PickableObject {
    float x, y;
} PickableObject;

class Map {
public:
    Map();
    float distance(float x, float y, float angle) const;
    bool point_approx_on_wall(float x, float y);
    bool update_from_laser(const geometry_msgs::Polygon::ConstPtr &msg);
    std::vector<Wall> walls;
    std::vector<PickableObject> pickable_objects;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
private:
    void readWalls();
};

#endif
