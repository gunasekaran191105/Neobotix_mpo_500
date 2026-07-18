#include "neobotix_mpo_500/pso_planner.hpp"
#include <cmath>
#include <random>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

namespace neobotix_mpo_500
{

void PsoPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  
  (void)parent; // Suppress unused warning
  RCLCPP_INFO(rclcpp::get_logger("PsoPlanner"), "Configured custom PSO Planner.");
}

void PsoPlanner::cleanup() { RCLCPP_INFO(rclcpp::get_logger("PsoPlanner"), "Cleaning up..."); }
void PsoPlanner::activate() { RCLCPP_INFO(rclcpp::get_logger("PsoPlanner"), "Activating..."); }
void PsoPlanner::deactivate() { RCLCPP_INFO(rclcpp::get_logger("PsoPlanner"), "Deactivating..."); }

// --- COSTMAP CHECKING ---
// Translates physical X/Y coordinates into map pixels to check for walls
bool PsoPlanner::isStateValid(double x, double y)
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return false; // Out of bounds of the map
  }
  unsigned char cost = costmap_->getCost(mx, my);
  
  // Reject paths that enter inscribed or lethal obstacle zones
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return false;
  }
  return true;
}
// --- PATH SCORING ---
// Calculates the fitness of a particle (path). Lower is better.
double PsoPlanner::calculatePathCost(const std::vector<Point2D>& path)
{
  double cost = 0.0;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    // 1. Reward shorter overall paths
    double dist = std::hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y);
    cost += dist;
    
    // 2. Evaluate Costmap safety along the line segment
    int steps = std::max(10, (int)(dist / 0.05)); 
    for (int s = 0; s <= steps; ++s) {
      double interp_x = path[i].x + (path[i+1].x - path[i].x) * (s / (double)steps);
      double interp_y = path[i].y + (path[i+1].y - path[i].y) * (s / (double)steps);
      
      unsigned int mx, my;
      if (!costmap_->worldToMap(interp_x, interp_y, mx, my)) {
        cost += 10000.0; // Out of map bounds
        break; 
      }
      
      unsigned char cell_cost = costmap_->getCost(mx, my);
      
      // LETHAL COLLISION: Reject paths that hit the actual obstacle
      if (cell_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        cost += 10000.0; 
        break; 
      }
      
      // DANGER PENALTY: Push the swarm away from the walls!
      // 'cell_cost' ranges from 1 to 252 in the cyan/pink inflation zones.
      // This applies a heavy score penalty for getting too close to a wall, 
      // forcing the algorithm to route the path through the white space (cost = 0).
      if (cell_cost > 0) {
        cost += (cell_cost / 252.0) * 15.0; 
      }
    }
  }
  
  return cost;
}

// --- PARTICLE SWARM OPTIMIZATION ---
nav_msgs::msg::Path PsoPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  global_path.poses.clear();
  global_path.header.stamp = start.header.stamp;
  global_path.header.frame_id = global_frame_;

  // --- INCREASED SWARM POWER ---
  // Boost the parameters to ensure the swarm can easily navigate around large blocks
  swarm_size_ = 150;
  max_iterations_ = 200;
  num_waypoints_ = 20; 

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> rand_r(0.0, 1.0);
  
  // Use a normal distribution to create an outward "bulge" from the straight line
  std::normal_distribution<> noise(0.0, 2.0); 
  
  // Initialize Swarm Memory
  std::vector<Particle> swarm(swarm_size_);
  std::vector<Point2D> gBest;
  double gBestCost = std::numeric_limits<double>::max();

  // --- ORDERED SCATTERING ---
  for (int i = 0; i < swarm_size_; ++i) {
    swarm[i].position.resize(num_waypoints_);
    swarm[i].velocity.resize(num_waypoints_, {0.0, 0.0});
    
    // Lock the first and last waypoints
    swarm[i].position[0] = {start.pose.position.x, start.pose.position.y};
    swarm[i].position[num_waypoints_ - 1] = {goal.pose.position.x, goal.pose.position.y};
    
    // Distribute intermediate waypoints evenly along the line, then apply noise
    for (int j = 1; j < num_waypoints_ - 1; ++j) {
      double fraction = (double)j / (num_waypoints_ - 1);
      double base_x = start.pose.position.x + fraction * (goal.pose.position.x - start.pose.position.x);
      double base_y = start.pose.position.y + fraction * (goal.pose.position.y - start.pose.position.y);
      
      swarm[i].position[j] = {base_x + noise(gen), base_y + noise(gen)};
    }
    
    // Set initial personal bests
    swarm[i].pBest = swarm[i].position;
    swarm[i].pBestCost = calculatePathCost(swarm[i].position);
    
    if (swarm[i].pBestCost < gBestCost) {
      gBest = swarm[i].pBest;
      gBestCost = swarm[i].pBestCost;
    }
  }

  // Optimization Loop
  for (int iter = 0; iter < max_iterations_; ++iter) {
    for (int i = 0; i < swarm_size_; ++i) {
      for (int j = 1; j < num_waypoints_ - 1; ++j) { 
        double r1 = rand_r(gen);
        double r2 = rand_r(gen);
        
        swarm[i].velocity[j].x = w_ * swarm[i].velocity[j].x + 
                                 c1_ * r1 * (swarm[i].pBest[j].x - swarm[i].position[j].x) + 
                                 c2_ * r2 * (gBest[j].x - swarm[i].position[j].x);
                                 
        swarm[i].velocity[j].y = w_ * swarm[i].velocity[j].y + 
                                 c1_ * r1 * (swarm[i].pBest[j].y - swarm[i].position[j].y) + 
                                 c2_ * r2 * (gBest[j].y - swarm[i].position[j].y);
        
        swarm[i].position[j].x += swarm[i].velocity[j].x;
        swarm[i].position[j].y += swarm[i].velocity[j].y;
      }
      
      double currentCost = calculatePathCost(swarm[i].position);
      if (currentCost < swarm[i].pBestCost) {
        swarm[i].pBestCost = currentCost;
        swarm[i].pBest = swarm[i].position;
        
        if (currentCost < gBestCost) {
          gBestCost = currentCost;
          gBest = swarm[i].position;
        }
      }
    }
  }

// Package the winning Global Best path
  for (int i = 0; i < num_waypoints_; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.x = gBest[i].x;
    pose.pose.position.y = gBest[i].y;
    pose.pose.position.z = 0.0;
    
    // THE FIX: Tell DWB to match the Goal arrow's orientation!
    pose.pose.orientation = goal.pose.orientation;
    
    global_path.poses.push_back(pose);
  }

  RCLCPP_INFO(rclcpp::get_logger("PsoPlanner"), "PSO Planner converged! Path Cost: %f", gBestCost);
  return global_path;
}

}  // namespace neobotix_mpo_500

PLUGINLIB_EXPORT_CLASS(neobotix_mpo_500::PsoPlanner, nav2_core::GlobalPlanner)
