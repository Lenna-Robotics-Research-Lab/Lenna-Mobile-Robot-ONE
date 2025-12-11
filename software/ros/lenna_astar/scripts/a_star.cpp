#include "lenna_astar/a_star.h"
#include <algorithm>
#include <iostream>
#include <set>
#include <cmath>

std::string intToString(int num) {
    std::ostringstream oss;
    oss << num;
    return oss.str();
}

AStarPlanner::AStarPlanner(int width, int height) 
    : grid_width_(width), grid_height_(height) {
    grid_.assign(height, std::vector<int>(width, 0));
    
    // dx_.push_back(0); dx_.push_back(0); dx_.push_back(-1); dx_.push_back(1);
    // dy_.push_back(-1); dy_.push_back(1); dy_.push_back(0); dy_.push_back(0);
    
    // diagonal movement instead of normal 
    // 4 cardinal directions
    dx_.push_back(0);  dx_.push_back(0);  dx_.push_back(-1); dx_.push_back(1);
    dy_.push_back(-1); dy_.push_back(1); dy_.push_back(0);  dy_.push_back(0);
    
    // 4 diagonal directions
    dx_.push_back(-1); dx_.push_back(-1); dx_.push_back(1);  dx_.push_back(1);
    dy_.push_back(-1); dy_.push_back(1);  dy_.push_back(-1); dy_.push_back(1);
}

AStarPlanner::~AStarPlanner() {}

void AStarPlanner::setGrid(const std::vector<std::vector<int> >& grid) {
    grid_ = grid;
    grid_height_ = grid.size();
    if (grid.size() > 0) {
        grid_width_ = grid[0].size();
    }
}

void AStarPlanner::setCell(int x, int y, int value) {
    if (isValid(x, y)) {
        grid_[y][x] = value;
    }
}

// for 4 directional movement 
// float AStarPlanner::heuristic(int x1, int y1, int x2, int y2) const {
//     return std::abs(x1 - x2) + std::abs(y1 - y2);
// }

// for 8 directional movement Ecludian
float AStarPlanner::heuristic(int x1, int y1, int x2, int y2) const {
    float dx = static_cast<float>(x1 - x2);
    float dy = static_cast<float>(y1 - y2);
    return std::sqrt(dx*dx + dy*dy);
}



bool AStarPlanner::isValid(int x, int y) const {
    return (x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_ && grid_[y][x] == 0);
}

std::vector<std::pair<int, int> > AStarPlanner::findPath(int start_x, int start_y, 
                                                          int goal_x, int goal_y) {
    std::vector<std::pair<int, int> > path;

    if (!isValid(start_x, start_y) || !isValid(goal_x, goal_y)) {
        std::cerr << "Invalid start or goal position" << std::endl;
        return path;
    }

    // Use std::set instead of priority_queue + std::set instead of unordered_set
    std::priority_queue<Node, std::vector<Node>, std::greater<Node> > openSet;
    std::set<std::string> closedSet;
    std::map<std::string, Node*> nodeMap;

    Node* startNode = new Node(start_x, start_y);
    startNode->g = 0;
    startNode->h = heuristic(start_x, start_y, goal_x, goal_y);
    startNode->f = startNode->h;

    openSet.push(*startNode);
    std::string startKey = intToString(start_x) + "," + intToString(start_y);
    nodeMap[startKey] = startNode;

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        std::string currentKey = intToString(current.x) + "," + intToString(current.y);

        if (current.x == goal_x && current.y == goal_y) {
            Node* node = nodeMap[currentKey];
            while (node != 0) {
                path.push_back(std::make_pair(node->x, node->y));
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());

            // Cleanup
            for (std::map<std::string, Node*>::iterator it = nodeMap.begin(); it != nodeMap.end(); ++it) {
                delete it->second;
            }
            return path;
        }

        closedSet.insert(currentKey);

        for (int i = 0; i < 8; i++) {
            int newX = current.x + dx_[i];
            int newY = current.y + dy_[i];

            if (!isValid(newX, newY)) continue;

            std::string neighborKey = intToString(newX) + "," + intToString(newY);
            
            if (closedSet.find(neighborKey) != closedSet.end()) continue;
            
            // for 4 directional movement 
            // float moveCost = 1.0;

            // for 8 directional movement
            int dx_move = std::abs(newX - current.x);
            int dy_move = std::abs(newY - current.y);
            float moveCost = (dx_move == 1 && dy_move == 1) ? std::sqrt(2.0f) : 1.0f; // sqrt instead of 9

            float tentativeG = current.g + moveCost;

            Node* neighbor = 0;
            if (nodeMap.find(neighborKey) == nodeMap.end()) {
                neighbor = new Node(newX, newY);
                neighbor->g = tentativeG;
                neighbor->h = heuristic(newX, newY, goal_x, goal_y);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = nodeMap[currentKey];
                nodeMap[neighborKey] = neighbor;
                openSet.push(*neighbor);
            } else {
                neighbor = nodeMap[neighborKey];
                if (tentativeG < neighbor->g) {
                    neighbor->g = tentativeG;
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = nodeMap[currentKey];
                    openSet.push(*neighbor);
                }
            }
        }
    }

    // Cleanup
    for (std::map<std::string, Node*>::iterator it = nodeMap.begin(); it != nodeMap.end(); ++it) {
        delete it->second;
    }
    
    return path;
}

// Helper function to check if a line of sight is clear
// bool AStarPlanner::isLineOfSightClear(int x1, int y1, int x2, int y2) const {
//     int dx = std::abs(x2 - x1);
//     int dy = std::abs(y2 - y1);
//     int sx = (x1 < x2) ? 1 : -1;
//     int sy = (y1 < y2) ? 1 : -1;
//     int err = dx - dy;
    
//     int x = x1, y = y1;
//     while (true) {
//         if (!isValid(x, y)) return false;  // Hit obstacle
//         if (x == x2 && y == y2) return true;  // Reached goal
        
//         int e2 = 2 * err;
//         if (e2 > -dy) err -= dy, x += sx;
//         if (e2 < dx) err += dx, y += sy;
//     }
// }

// // Path smoothing function
// std::vector<std::pair<int, int>> AStarPlanner::smoothPath(
//     const std::vector<std::pair<int, int>>& gridPath) {
    
//     if (gridPath.size() < 3) return gridPath;
    
//     std::vector<std::pair<int, int>> smoothed;
//     smoothed.push_back(gridPath[0]);
    
//     int current = 0;
//     while (current < gridPath.size() - 1) {
//         int next = gridPath.size() - 1;
        
//         // Try to find the farthest point we can reach without hitting obstacles
//         while (next > current + 1) {
//             if (isLineOfSightClear(gridPath[current].first, gridPath[current].second,
//                                    gridPath[next].first, gridPath[next].second)) {
//                 break;
//             }
//             next--;
//         }
        
//         if (next != current + 1) {
//             smoothed.push_back(gridPath[next]);
//         } else {
//             smoothed.push_back(gridPath[current + 1]);
//         }
//         current = next;
//     }
    
//     return smoothed;
// }
