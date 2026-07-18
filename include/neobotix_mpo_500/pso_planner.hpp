#ifndef NEOBOTIX_MPO_500__PSO_PLANNER_HPP_
#define NEOBOTIX_MPO_500__PSO_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <limits>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace neobotix_mpo_500
{

// A discrete coordinate on our factory floor
struct Point2D {
  double x;
  double y;
};

// Represents an entire path candidate
struct Particle {
  std::vector<Point2D> position;   // Current waypoints
  std::vector<Point2D> velocity;   // Waypoint movement vectors
  std::vector<Point2D> pBest;      // Personal best waypoints
  double pBestCost;                // Personal best score
};

class PsoPlanner : public nav2_core::GlobalPlanner
{
public:
  PsoPlanner() = default;
  ~PsoPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // --- PSO Specific Methods ---
  double calculatePathCost(const std::vector<Point2D>& path);
  bool isStateValid(double x, double y);
  
  // --- PSO Parameters ---
  int swarm_size_ = 30;
  int max_iterations_ = 100;
  int num_waypoints_ = 10;
  double w_ = 0.5;   // Inertia weight
  double c1_ = 1.5;  // Cognitive (personal) weight
  double c2_ = 1.5;  // Social (global) weight

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;
};

}  // namespace neobotix_mpo_500

#endif  // NEOBOTIX_MPO_500__PSO_PLANNER_HPP_
