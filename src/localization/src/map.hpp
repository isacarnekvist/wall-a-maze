#ifndef _MAP_HPP
#define _MAP_HPP

#include <map>
#include <tuple>
#include <vector>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

static const int RED = 0;
static const int BLUE = 1;
static const int YELLOW = 2;
static const int ORANGE = 3;
static const int PURPLE = 4;
static const int GREEN = 5;

static const int PLUS = 0;
static const int BALL = 1;
static const int CUBE_SOLID = 2;
static const int CUBE_HOLLOW = 3;
static const int STAR = 4;
static const int TRIANGLE = 5;

typedef struct Wall {
    float x1, y1, x2, y2;
} Wall;

typedef struct PickableObject {
    float x, y;
    bool rubbish;
} PickableObject;

class Map {
public:
    Map();
    float distance(float x, float y, float angle) const;
    bool point_approx_on_wall(float x, float y);
    bool update_from_laser(const geometry_msgs::Polygon::ConstPtr &msg);
    std::vector<Wall> walls;
    std::map<int, PickableObject> pickable_objects;
    int next_pickable_id = 0;
    int add_pickable(float x, float y, bool rubbish);
    void remove_pickable(int id);
    float min_x;
    float max_x;
    float min_y;
    float max_y;
private:
    void readWalls();
};

#endif
