#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "lenna_astar/a_star.h"
#include <vector>
#include <tf/transform_listener.h>

class AStarNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;

    AStarPlanner* planner_;
    nav_msgs::OccupancyGrid map_;
    bool map_received_;

    ros::Timer plan_timer_;

    bool goal_received_;
    int goal_x_;
    int goal_y_;


    int inflation_radius_cells_;
    float map_resolution_;
    int map_width_;
    int map_height_;
    float robot_x_;
    float robot_y_;

    tf::TransformListener tf_listener_;  // TF listener for transform lookups

public:
    AStarNode() : map_received_(false), map_resolution_(0.05),
                  map_width_(100), map_height_(100),
                  inflation_radius_cells_(3), goal_received_(false) {  
        // Initialize planner with default grid size
        planner_ = new AStarPlanner(map_width_, map_height_);

        // Subscribe to map
        map_sub_ = nh_.subscribe("/map", 1, &AStarNode::mapCallback, this);

        // Subscribe to goal
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &AStarNode::goalCallback, this);

        // Publish path
        path_pub_ = nh_.advertise<nav_msgs::Path>("/astar_path", 1);

        plan_timer_ = nh_.createTimer(ros::Duration(0.1), &AStarNode::planTimerCb, this);

        ROS_INFO("A* Planner Node initialized");
    }

    ~AStarNode() {
        delete planner_;
    }

    // Update robot position using TF lookup
    bool updateRobotPosition() {
        tf::StampedTransform transform;
        try {
            // Lookup transform from map frame to scanmatch_odom frame (latest available)
            tf_listener_.lookupTransform("/map", "scanmatcher_frame", 
                                       ros::Time(0), transform);
            
            // Extract position from transform
            robot_x_ = transform.getOrigin().x();
            robot_y_ = transform.getOrigin().y();
            
            ROS_DEBUG("Robot position from TF: (%.2f, %.2f)", robot_x_, robot_y_);
            return true;
        } catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
            return false;
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ = *msg;
        map_resolution_ = msg->info.resolution;
        map_width_ = msg->info.width;
        map_height_ = msg->info.height;

        // Convert occupancy grid to binary grid for A*
        std::vector<std::vector<int>> grid(map_height_, std::vector<int>(map_width_, 0));

        for (int y = 0; y < map_height_; y++) {
            for (int x = 0; x < map_width_; x++) {
                int index = y * map_width_ + x;
                // Treat cells with value > 50 as obstacles (1), else free (0)
                grid[y][x] = (msg->data[index] > 50) ? 1 : 0;
            }
        }

        // ===== Inflate obstacles =====
        std::vector<std::vector<int>> inflated_grid = grid;  // start from original

        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                if (grid[y][x] == 1) {
                    // Mark neighbors within inflation_radius_cells_ as obstacles
                    for (int dy = -inflation_radius_cells_; dy <= inflation_radius_cells_; ++dy) {
                        for (int dx = -inflation_radius_cells_; dx <= inflation_radius_cells_; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx < 0 || nx >= map_width_ || ny < 0 || ny >= map_height_)
                                continue;
                            // Optional: use circular inflation instead of square
                            if (dx*dx + dy*dy <= inflation_radius_cells_ * inflation_radius_cells_) {
                                inflated_grid[ny][nx] = 1;
                            }
                        }
                    }
                }
            }
        }

        // Use inflated grid for planning
        planner_->setGrid(inflated_grid);
        map_received_ = true;
        ROS_INFO("Map received: %dx%d (inflated with radius %d cells)",
                map_width_, map_height_, inflation_radius_cells_);
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!map_received_) {
            ROS_WARN("Map not yet received. Cannot accept goal.");
            return;
        }

        // Convert goal pose to grid coordinates (store them)
        goal_x_ = static_cast<int>((msg->pose.position.x - map_.info.origin.position.x) / map_resolution_);
        goal_y_ = static_cast<int>((msg->pose.position.y - map_.info.origin.position.y) / map_resolution_);

        goal_received_ = true;

        ROS_INFO("New goal received: world (%.2f, %.2f) -> grid (%d, %d)",
                 msg->pose.position.x, msg->pose.position.y, goal_x_, goal_y_);
    }

    void planTimerCb(const ros::TimerEvent&) {
        if (!map_received_ || !goal_received_) {
            return;  // nothing to do yet
        }

        // Get current robot position using TF lookup
        if (!updateRobotPosition()) {
            ROS_WARN_THROTTLE(1.0, "Failed to get robot position from TF. Skipping planning cycle.");
            return;
        }

        int start_x = static_cast<int>((robot_x_ - map_.info.origin.position.x) / map_resolution_);
        int start_y = static_cast<int>((robot_y_ - map_.info.origin.position.y) / map_resolution_);

        // Simple check that start and goal are within map bounds
        if (start_x < 0 || start_x >= map_width_ ||
            start_y < 0 || start_y >= map_height_) {
            ROS_WARN_THROTTLE(1.0, "Start pose outside map. Skipping planning cycle.");
            return;
        }

        ROS_DEBUG("Replanning from (%d,%d) to (%d,%d)", start_x, start_y, goal_x_, goal_y_);

        std::vector<std::pair<int, int>> gridPath =
            planner_->findPath(start_x, start_y, goal_x_, goal_y_);

        if (gridPath.empty()) {
            ROS_WARN_THROTTLE(1.0, "No path found in current cycle.");
            return;
        }

        // Convert grid path to world coordinates and publish
        nav_msgs::Path path;
        path.header.frame_id = map_.header.frame_id;
        path.header.stamp = ros::Time::now();

        for (const auto& point : gridPath) {
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;

            pose.pose.position.x = point.first  * map_resolution_ + map_.info.origin.position.x;
            pose.pose.position.y = point.second * map_resolution_ + map_.info.origin.position.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;

            path.poses.push_back(pose);
        }

        path_pub_.publish(path);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");

    AStarNode astar_node;

    ros::spin();

    return 0;
}
