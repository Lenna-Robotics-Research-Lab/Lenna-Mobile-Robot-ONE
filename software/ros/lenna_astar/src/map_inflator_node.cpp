#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>
#include <cmath>

class MapInflatorNode {
public:
  MapInflatorNode()
  : pnh_("~")
  {
    // --- Parameters (private namespace: ~) ---
    pnh_.param<std::string>("input_map_topic",  input_map_topic_,  std::string("/map"));
    pnh_.param<std::string>("output_map_topic", output_map_topic_, std::string("/inflated_map"));

    pnh_.param<int>("inflation_radius_cells", inflation_radius_cells_, 2);
    pnh_.param<int>("occupied_threshold",     occupied_threshold_,     50);
    pnh_.param<bool>("preserve_unknown",      preserve_unknown_,      true);
    pnh_.param<bool>("use_circular_inflation",use_circular_inflation_, true);

    // Latched publisher so RViz immediately shows last inflated map
    inflated_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(output_map_topic_, 1, true);
    map_sub_ = nh_.subscribe(input_map_topic_, 1, &MapInflatorNode::mapCb, this);

    // Precompute offsets once (fast + clean)
    rebuildOffsets();

    ROS_INFO_STREAM("MapInflatorNode started."
                    << " input_map_topic=" << input_map_topic_
                    << " output_map_topic=" << output_map_topic_
                    << " inflation_radius_cells=" << inflation_radius_cells_
                    << " occupied_threshold=" << occupied_threshold_
                    << " preserve_unknown=" << (preserve_unknown_ ? "true" : "false")
                    << " circular=" << (use_circular_inflation_ ? "true" : "false"));
  }

private:
  // -------- ROS --------
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber map_sub_;
  ros::Publisher inflated_pub_;

  // -------- Params --------
  std::string input_map_topic_;
  std::string output_map_topic_;
  int inflation_radius_cells_;
  int occupied_threshold_;
  bool preserve_unknown_;
  bool use_circular_inflation_;

  // Precomputed neighbor offsets within inflation radius
  std::vector<std::pair<int,int>> offsets_;

  void rebuildOffsets() {
    offsets_.clear();
    const int r = std::max(0, inflation_radius_cells_);

    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx == 0 && dy == 0) {
          offsets_.push_back({0,0});
          continue;
        }

        if (use_circular_inflation_) {
          if (dx*dx + dy*dy <= r*r) offsets_.push_back({dx,dy});
        } else {
          offsets_.push_back({dx,dy});
        }
      }
    }
  }

  inline bool inBounds(int x, int y, int w, int h) const {
    return (x >= 0 && x < w && y >= 0 && y < h);
  }

  inline int idx(int x, int y, int w) const {
    return y * w + x;
  }

  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    const int w = static_cast<int>(msg->info.width);
    const int h = static_cast<int>(msg->info.height);

    if (w <= 0 || h <= 0) {
      ROS_WARN_THROTTLE(1.0, "Received map with invalid size.");
      return;
    }
    if (static_cast<int>(msg->data.size()) != w * h) {
      ROS_WARN_THROTTLE(1.0, "Received map data size mismatch.");
      return;
    }

    // Output message: copy metadata; rewrite data
    nav_msgs::OccupancyGrid out = *msg;
    out.header.stamp = ros::Time::now();

    // Start with either:
    // - preserve unknown (-1), and treat everything else as free (0)
    // - or just make everything free (0)
    out.data.assign(w * h, 0);
    if (preserve_unknown_) {
      for (int i = 0; i < w*h; ++i) {
        if (msg->data[i] == -1) out.data[i] = -1;
      }
    }

    // Mark inflated obstacles:
    // For every occupied cell in input, set all neighbors within radius to occupied (100),
    // but keep unknown if preserve_unknown_ and target cell is unknown.
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const int i = idx(x, y, w);
        const int8_t v = msg->data[i];

        // input obstacle?
        if (v >= occupied_threshold_) {
          for (const auto& off : offsets_) {
            const int nx = x + off.first;
            const int ny = y + off.second;
            if (!inBounds(nx, ny, w, h)) continue;

            const int ni = idx(nx, ny, w);

            if (preserve_unknown_ && msg->data[ni] == -1) {
              // Keep unknown as unknown if requested
              continue;
            }

            out.data[ni] = 100;
          }
        }
      }
    }

    inflated_pub_.publish(out);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_inflator_node");
  MapInflatorNode node;
  ros::spin();
  return 0;
}