#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "jo_msgs/msg/obstacle_array.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

namespace jo_sim
{

class DynamicObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  DynamicObstacleLayer() = default;

  void onInitialize() override
  {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("obstacle_padding", rclcpp::ParameterValue(0.10));
    declareParameter("tracking_timeout", rclcpp::ParameterValue(1.0));
    declareParameter("velocity_inflation_k", rclcpp::ParameterValue(3.0));
    declareParameter("rear_inflation_k", rclcpp::ParameterValue(0.45));
    declareParameter("lateral_inflation_k", rclcpp::ParameterValue(0.60));
    declareParameter("dynamic_risk_cost", rclcpp::ParameterValue(230));
    declareParameter("dynamic_risk_min_cost", rclcpp::ParameterValue(45));
    declareParameter("risk_velocity_threshold", rclcpp::ParameterValue(0.2));
    declareParameter("use_glim_rejection_obstacles", rclcpp::ParameterValue(true));
    declareParameter("glim_rejection_obstacles_topic", rclcpp::ParameterValue(
      "/glim_ros/rejection_obstacles"));
    declareParameter("glim_rejection_timeout", rclcpp::ParameterValue(0.5));
    declareParameter("glim_rejection_velocity_scale", rclcpp::ParameterValue(0.75));

    auto node = node_.lock();

    if (node) {
      node->get_parameter(name_ + ".enabled", enabled_);
      node->get_parameter(name_ + ".obstacle_padding", obstacle_padding_);
      node->get_parameter(name_ + ".tracking_timeout", tracking_timeout_);
      node->get_parameter(name_ + ".velocity_inflation_k", velocity_inflation_k_);
      node->get_parameter(name_ + ".rear_inflation_k", rear_inflation_k_);
      node->get_parameter(name_ + ".lateral_inflation_k", lateral_inflation_k_);
      node->get_parameter(name_ + ".dynamic_risk_cost", dynamic_risk_cost_);
      node->get_parameter(name_ + ".dynamic_risk_min_cost", dynamic_risk_min_cost_);
      node->get_parameter(name_ + ".risk_velocity_threshold", risk_velocity_threshold_);
      node->get_parameter(name_ + ".use_glim_rejection_obstacles", use_glim_rejection_obstacles_);
      node->get_parameter(name_ + ".glim_rejection_obstacles_topic", glim_rejection_obstacles_topic_);
      node->get_parameter(name_ + ".glim_rejection_timeout", glim_rejection_timeout_);
      node->get_parameter(name_ + ".glim_rejection_velocity_scale", glim_rejection_velocity_scale_);

      dynamic_risk_min_cost_ = std::clamp(dynamic_risk_min_cost_, 1, 252);
      dynamic_risk_cost_ = std::clamp(dynamic_risk_cost_, dynamic_risk_min_cost_, 252);

      clock_ = node->get_clock();

      sub_ = node->create_subscription<jo_msgs::msg::ObstacleArray>(
        "/onboard_detector/tracked_dynamic_obstacles",
        10,
        [this](const jo_msgs::msg::ObstacleArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_msg_ = msg;
          last_msg_time_ = clock_->now();
        });

      if (use_glim_rejection_obstacles_) {
        rejection_sub_ = node->create_subscription<jo_msgs::msg::ObstacleArray>(
          glim_rejection_obstacles_topic_,
          10,
          [this](const jo_msgs::msg::ObstacleArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_rejection_msg_ = msg;
            last_rejection_msg_time_ = clock_->now();
          });
      }

      marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "dynamic_obstacle_footprints",
        rclcpp::QoS(1));
    }

    global_frame_ = layered_costmap_->getGlobalFrameID();
    current_ = true;
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_msg_.reset();
    latest_rejection_msg_.reset();
    previous_bounds_.clear();
    current_bounds_.clear();
    current_ = true;
  }

  bool isClearable() override
  {
    return false;
  }

  void updateBounds(
    double /* robot_x */,
    double /* robot_y */,
    double /* robot_yaw */,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override
  {
    if (!enabled_) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto & b : previous_bounds_) {
      *min_x = std::min(*min_x, b.x - b.r);
      *min_y = std::min(*min_y, b.y - b.r);
      *max_x = std::max(*max_x, b.x + b.r);
      *max_y = std::max(*max_y, b.y + b.r);
    }

    current_bounds_.clear();

    std::unordered_set<uint32_t> onboard_track_ids;
    if (latest_msg_ && isTrackingValidNoLock()) {
      onboard_track_ids.reserve(latest_msg_->obstacles.size());
      for (const auto & obs : latest_msg_->obstacles) {
        onboard_track_ids.insert(obs.track_id);
      }
    }

    auto update_bounds_from_msg = [&](
      const jo_msgs::msg::ObstacleArray::SharedPtr & msg,
      const bool from_rejection) {
      if (!msg) {
        return;
      }
      for (const auto & obs : msg->obstacles) {
        if (from_rejection && onboard_track_ids.find(obs.track_id) != onboard_track_ids.end()) {
          continue;
        }
        // Only inflate bounds for obstacles classified as dynamic or potentially dynamic.
        // STATIC and TRACKED obstacles are not painted by this layer.
        if (obs.status != jo_msgs::msg::Obstacle::STATUS_DYNAMIC &&
            obs.status != jo_msgs::msg::Obstacle::STATUS_POTENTIALLY_DYNAMIC) {
          continue;
        }

        const double ox = obs.pose.position.x;
        const double oy = obs.pose.position.y;

        const double hx = obs.size.x * 0.5 + obstacle_padding_;
        const double hy = obs.size.y * 0.5 + obstacle_padding_;

        const double vx = obs.twist.linear.x;
        const double vy = obs.twist.linear.y;
        const double speed = std::hypot(vx, vy);
        const bool apply_velocity_inflation = (speed > risk_velocity_threshold_);
        const double velocity_scale = from_rejection ? glim_rejection_velocity_scale_ : 1.0;

        const double velocity_inflation =
          apply_velocity_inflation ? speed * velocity_inflation_k_ * velocity_scale : 0.0;
        const double rear_inflation =
          apply_velocity_inflation ? speed * rear_inflation_k_ * velocity_scale : 0.0;
        const double lateral_inflation =
          apply_velocity_inflation ? speed * lateral_inflation_k_ * velocity_scale : 0.0;

        const double radius =
          std::max(hx + velocity_inflation, hy + lateral_inflation) + rear_inflation + 0.20;

        current_bounds_.push_back({ox, oy, radius});

        *min_x = std::min(*min_x, ox - radius);
        *min_y = std::min(*min_y, oy - radius);
        *max_x = std::max(*max_x, ox + radius);
        *max_y = std::max(*max_y, oy + radius);
      }
    };

    if (latest_msg_ && isTrackingValidNoLock()) {
      update_bounds_from_msg(latest_msg_, false);
    }
    if (latest_rejection_msg_ && isRejectionValidNoLock()) {
      update_bounds_from_msg(latest_rejection_msg_, true);
    }
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int /* min_i */,
    int /* min_j */,
    int /* max_i */,
    int /* max_j */) override
  {
    if (!enabled_) {
      return;
    }

    struct LayerObstacle
    {
      jo_msgs::msg::Obstacle obs;
      bool from_rejection;
    };

    std::vector<LayerObstacle> obstacles;
    std::vector<Bound> old_bounds;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      old_bounds = previous_bounds_;
      previous_bounds_ = current_bounds_;

      const bool onboard_valid = latest_msg_ && isTrackingValidNoLock();
      const bool rejection_valid = latest_rejection_msg_ && isRejectionValidNoLock();

      if (onboard_valid) {
        obstacles.reserve(latest_msg_->obstacles.size());
        for (const auto & obs : latest_msg_->obstacles) {
          obstacles.push_back({obs, false});
        }
      }
      if (rejection_valid) {
        std::unordered_set<uint32_t> onboard_track_ids;
        onboard_track_ids.reserve(obstacles.size());
        for (const auto & obstacle : obstacles) {
          onboard_track_ids.insert(obstacle.obs.track_id);
        }

        for (const auto & obs : latest_rejection_msg_->obstacles) {
          if (onboard_track_ids.find(obs.track_id) != onboard_track_ids.end()) {
            continue;
          }
          obstacles.push_back({obs, true});
        }
      }
    }

    const double resolution = master_grid.getResolution();

    // Clear costs written by this layer in the previous cycle.
    // Skip LETHAL_OBSTACLE (254) cells — those belong to the VoxelLayer (static obstacles).
    for (const auto & b : old_bounds) {
      for (double dx = -b.r; dx <= b.r; dx += resolution) {
        for (double dy = -b.r; dy <= b.r; dy += resolution) {
          unsigned int mx = 0;
          unsigned int my = 0;
          if (!master_grid.worldToMap(b.x + dx, b.y + dy, mx, my)) {
            continue;
          }
          if (master_grid.getCost(mx, my) != nav2_costmap_2d::LETHAL_OBSTACLE) {
            master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
          }
        }
      }
    }

    if (obstacles.empty()) {
      publishClearMarkers();
      return;
    }

    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    int marker_id = 0;

    for (const auto & obstacle : obstacles) {
      const auto & obs = obstacle.obs;
      const bool from_rejection = obstacle.from_rejection;

      // Only paint obstacles classified as dynamic or potentially dynamic.
      // STATIC and TRACKED obstacles (low velocity, not classified by detector) are skipped.
      if (obs.status != jo_msgs::msg::Obstacle::STATUS_DYNAMIC &&
          obs.status != jo_msgs::msg::Obstacle::STATUS_POTENTIALLY_DYNAMIC) {
        continue;
      }

      const double ox = obs.pose.position.x;
      const double oy = obs.pose.position.y;
      const double oz = obs.pose.position.z;

      RCLCPP_DEBUG(
        rclcpp::get_logger("DynamicObstacleLayer"),
        "obstacle id=%u age=%u status=%u pos=(%.3f, %.3f, %.3f) size=(%.3f, %.3f) frame=%s",
        obs.track_id, obs.track_age, obs.status,
        ox, oy, oz, obs.size.x, obs.size.y, global_frame_.c_str());

      const double hx = obs.size.x * 0.5 + obstacle_padding_;
      const double hy = obs.size.y * 0.5 + obstacle_padding_;
      const double hz = obs.size.z * 0.5 + obstacle_padding_;

      const double vx = obs.twist.linear.x;
      const double vy = obs.twist.linear.y;
      const double speed = std::hypot(vx, vy);
      const bool apply_velocity_inflation = (speed > risk_velocity_threshold_);
      const double velocity_scale = from_rejection ? glim_rejection_velocity_scale_ : 1.0;

      const double velocity_inflation =
        apply_velocity_inflation ? speed * velocity_inflation_k_ * velocity_scale : 0.0;
      const double rear_inflation =
        apply_velocity_inflation ? speed * rear_inflation_k_ * velocity_scale : 0.0;
      const double lateral_inflation =
        apply_velocity_inflation ? speed * lateral_inflation_k_ * velocity_scale : 0.0;

      const double heading = std::atan2(vy, vx);
      const double cos_h = std::cos(heading);
      const double sin_h = std::sin(heading);

      const double hx_front = hx + velocity_inflation;
      const double hx_back = hx + rear_inflation;
      const double hy_dynamic = hy + lateral_inflation;

      const double safe_hy = std::max(hy, resolution);
      const double safe_hy_dynamic = std::max(hy_dynamic, resolution);
      const double safe_hx_front = std::max(hx_front, resolution);
      const double safe_hx_back = std::max(hx_back, resolution);
      const double safe_hx = std::max(hx, resolution);

      const double search_radius =
        std::max({safe_hx_front, safe_hx_back, safe_hy_dynamic}) + resolution;

      for (double dx = -search_radius; dx <= search_radius; dx += resolution) {
        for (double dy = -search_radius; dy <= search_radius; dy += resolution) {
          const double wx = ox + dx;
          const double wy = oy + dy;

          unsigned int mx = 0;
          unsigned int my = 0;

          if (!master_grid.worldToMap(wx, wy, mx, my)) {
            continue;
          }

          const double u = dx * cos_h + dy * sin_h;
          const double v = -dx * sin_h + dy * cos_h;

          if (std::abs(dx) <= safe_hx && std::abs(dy) <= safe_hy) {
            if (!from_rejection) {
              master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
            } else {
              setMaxCost(master_grid, mx, my, static_cast<unsigned char>(dynamic_risk_cost_));
            }
            continue;
          }

          const double semi_u = u >= 0.0 ? safe_hx_front : safe_hx_back;
          const double risk_eu = u / semi_u;
          const double risk_ev = v / safe_hy_dynamic;
          const double normalized_distance = risk_eu * risk_eu + risk_ev * risk_ev;

          if (normalized_distance > 1.0) {
            continue;
          }

          if (!apply_velocity_inflation) {
            continue;
          }

          const double directional_bias = u >= 0.0 ? 1.0 : 0.35;
          const double falloff = std::max(0.0, 1.0 - normalized_distance);
          const int cost = dynamic_risk_min_cost_ +
            static_cast<int>((dynamic_risk_cost_ - dynamic_risk_min_cost_) * falloff * directional_bias);

          setMaxCost(master_grid, mx, my, static_cast<unsigned char>(
            std::clamp(cost, dynamic_risk_min_cost_, dynamic_risk_cost_)));
        }
      }

      visualization_msgs::msg::Marker physical_marker;
      physical_marker.header.frame_id = global_frame_;
      physical_marker.header.stamp = clock_->now();
      physical_marker.ns = "dynamic_obstacle_padded_box";
      physical_marker.id = marker_id++;
      physical_marker.type = visualization_msgs::msg::Marker::CUBE;
      physical_marker.action = visualization_msgs::msg::Marker::ADD;

      physical_marker.pose.position.x = ox;
      physical_marker.pose.position.y = oy;
      physical_marker.pose.position.z = oz;
      physical_marker.pose.orientation.w = 1.0;

      physical_marker.scale.x = 2.0 * hx;
      physical_marker.scale.y = 2.0 * hy;
      physical_marker.scale.z = 2.0 * hz;

      physical_marker.color.r = 1.0f;
      physical_marker.color.g = 1.0f;
      physical_marker.color.b = 0.0f;
      physical_marker.color.a = 0.45f;

      physical_marker.lifetime.sec = 0;
      physical_marker.lifetime.nanosec = 300000000;

      markers.markers.push_back(physical_marker);

      if (apply_velocity_inflation) {
        visualization_msgs::msg::Marker risk_marker;
        risk_marker.header.frame_id = global_frame_;
        risk_marker.header.stamp = physical_marker.header.stamp;
        risk_marker.ns = "dynamic_obstacle_predicted_risk";
        risk_marker.id = marker_id++;
        risk_marker.type = visualization_msgs::msg::Marker::SPHERE;
        risk_marker.action = visualization_msgs::msg::Marker::ADD;

        risk_marker.pose.position.x = ox + 0.5 * (velocity_inflation - rear_inflation) * cos_h;
        risk_marker.pose.position.y = oy + 0.5 * (velocity_inflation - rear_inflation) * sin_h;
        risk_marker.pose.position.z = oz;
        risk_marker.pose.orientation.z = std::sin(heading * 0.5);
        risk_marker.pose.orientation.w = std::cos(heading * 0.5);

        risk_marker.scale.x =
          2.0 * std::max(hx + 0.5 * (velocity_inflation + rear_inflation), resolution);
        risk_marker.scale.y = 2.0 * safe_hy_dynamic;
        risk_marker.scale.z = 0.05;

        risk_marker.color.r = 1.0f;
        risk_marker.color.g = 0.35f;
        risk_marker.color.b = 0.0f;
        risk_marker.color.a = 0.22f;

        risk_marker.lifetime = physical_marker.lifetime;

        markers.markers.push_back(risk_marker);
      }

    }

    if (marker_pub_) {
      marker_pub_->publish(markers);
    }
  }

private:
  struct Bound
  {
    double x;
    double y;
    double r;
  };

  bool isTrackingValidNoLock() const
  {
    if (!clock_) {
      return false;
    }

    return (clock_->now() - last_msg_time_).seconds() < tracking_timeout_;
  }

  bool isRejectionValidNoLock() const
  {
    if (!clock_) {
      return false;
    }

    return (clock_->now() - last_rejection_msg_time_).seconds() < glim_rejection_timeout_;
  }

  void publishClearMarkers()
  {
    if (!marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;

    markers.markers.push_back(delete_all);
    marker_pub_->publish(markers);
  }

  void setMaxCost(
    nav2_costmap_2d::Costmap2D & master_grid,
    const unsigned int mx,
    const unsigned int my,
    const unsigned char cost) const
  {
    const unsigned char old_cost = master_grid.getCost(mx, my);

    if (old_cost == nav2_costmap_2d::LETHAL_OBSTACLE || old_cost >= cost) {
      return;
    }

    master_grid.setCost(mx, my, cost);
  }

  rclcpp::Subscription<jo_msgs::msg::ObstacleArray>::SharedPtr sub_;
  rclcpp::Subscription<jo_msgs::msg::ObstacleArray>::SharedPtr rejection_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Clock::SharedPtr clock_;

  std::mutex mutex_;

  jo_msgs::msg::ObstacleArray::SharedPtr latest_msg_;
  jo_msgs::msg::ObstacleArray::SharedPtr latest_rejection_msg_;
  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_rejection_msg_time_{0, 0, RCL_ROS_TIME};

  std::string global_frame_{"odom"};

  bool enabled_{true};

  double obstacle_padding_{0.10};
  double tracking_timeout_{1.0};
  double velocity_inflation_k_{3.0};
  double rear_inflation_k_{0.45};
  double lateral_inflation_k_{0.60};
  int dynamic_risk_cost_{230};
  int dynamic_risk_min_cost_{45};
  double risk_velocity_threshold_{0.2};
  bool use_glim_rejection_obstacles_{true};
  std::string glim_rejection_obstacles_topic_{"/glim_ros/rejection_obstacles"};
  double glim_rejection_timeout_{0.5};
  double glim_rejection_velocity_scale_{0.75};

  std::vector<Bound> previous_bounds_;
  std::vector<Bound> current_bounds_;
};

}  // namespace jo_sim

PLUGINLIB_EXPORT_CLASS(jo_sim::DynamicObstacleLayer, nav2_costmap_2d::Layer)
