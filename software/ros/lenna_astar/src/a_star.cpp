#include "lenna_astar/a_star.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <set>

std::string intToString(int num) {
    std::ostringstream oss;
    oss << num;
    return oss.str();
}

AStarPlanner::AStarPlanner(int width, int height) 
    : grid_width_(width), grid_height_(height) {
    grid_.assign(height, std::vector<int>(width, 0));
    
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

