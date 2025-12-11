#ifndef A_STAR_H
#define A_STAR_H

#include <vector>
#include <queue>
#include <cmath>
#include <map>
#include <string>
#include <sstream>

struct Node {
    int x, y;
    float g;
    float h;
    float f;
    Node* parent;

    Node(int x = 0, int y = 0) : x(x), y(y), g(0), h(0), f(0), parent(0) {}

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

class AStarPlanner {
private:
    int grid_width_;
    int grid_height_;
    std::vector<std::vector<int> > grid_;
    std::vector<int> dx_;
    std::vector<int> dy_;

public:
    AStarPlanner(int width, int height);
    ~AStarPlanner();

    void setGrid(const std::vector<std::vector<int> >& grid);
    void setCell(int x, int y, int value);
    float heuristic(int x1, int y1, int x2, int y2) const;
    bool isValid(int x, int y) const;
    std::vector<std::pair<int, int> > findPath(int start_x, int start_y, int goal_x, int goal_y);
    

    // bool isLineOfSightClear(int x1, int y1, int x2, int y2) const;
    
    // std::vector<std::pair<int, int> > smoothPath(const std::vector<std::pair<int, int> >& gridPath);
};
#endif
