// The setup code for hybrid planner is based on the DWA planner,
// the default local planner for Turtlebot3 gazebo.
// As such, the Software License Agreement of DWA planner is included in this file.


/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/



/*
 * Copyright 2024 Fahim Shahriar
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <base_local_planner/goal_functions.h>
#include <hybrid_planner.h>

#include <cmath>

#include <Edge.h>
#include <Node.h>
#include <Helper.h>
#include <AStar_Node.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <ctime>
#include <fstream>
#include <map>
#include <queue>
#include <string>

namespace hybrid_local_planner {
void HybridPlanner::reconfigure(dwa_local_planner::DWAPlannerConfig &config) {
    boost::mutex::scoped_lock l(configuration_mutex_);

    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        sim_period_);

    double resolution = planner_util_->getCostmap()->getResolution();
    path_distance_bias_ = resolution * config.path_distance_bias;
    // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
    path_costs_.setScale(path_distance_bias_);
    alignment_costs_.setScale(path_distance_bias_);

    goal_distance_bias_ = resolution * config.goal_distance_bias;
    goal_costs_.setScale(goal_distance_bias_);
    goal_front_costs_.setScale(goal_distance_bias_);

    occdist_scale_ = config.occdist_scale;
    obstacle_costs_.setScale(occdist_scale_);

    stop_time_buffer_ = config.stop_time_buffer;
    oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
    forward_point_distance_ = config.forward_point_distance;
    goal_front_costs_.setXShift(forward_point_distance_);
    alignment_costs_.setXShift(forward_point_distance_);

    // obstacle costs can vary due to scaling footprint feature
    obstacle_costs_.setParams(config.max_vel_trans, config.max_scaling_factor, config.scaling_speed);

    twirling_costs_.setScale(config.twirling_scale);

    int vx_samp, vy_samp, vth_samp;
    vx_samp = config.vx_samples;
    vy_samp = config.vy_samples;
    vth_samp = config.vth_samples;

    if (vx_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
        vx_samp = 1;
        config.vx_samples = vx_samp;
    }

    if (vy_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
        vy_samp = 1;
        config.vy_samples = vy_samp;
    }

    if (vth_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
        vth_samp = 1;
        config.vth_samples = vth_samp;
    }

    vsamples_[0] = vx_samp;
    vsamples_[1] = vy_samp;
    vsamples_[2] = vth_samp;
}

HybridPlanner::HybridPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) : planner_util_(planner_util),
                                                                                                     obstacle_costs_(planner_util->getCostmap()),
                                                                                                     path_costs_(planner_util->getCostmap()),
                                                                                                     goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
                                                                                                     goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
                                                                                                     alignment_costs_(planner_util->getCostmap()) {
    ros::NodeHandle private_nh("~/" + name);

    step_count = 0;
    ros::Rate r(1);
    marker_pub = private_nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    astar_path.header.frame_id = "map";
    astar_path.type = visualization_msgs::Marker::LINE_LIST;
    astar_path.id = 1;
    astar_path.color.g = 1.0f;
    astar_path.color.a = 1.0;
    astar_path.scale.x = 0.005;
    astar_path.scale.y = 0.005;
    astar_path.ns = "points_and_lines";

    goal_front_costs_.setStopOnFailure(false);
    alignment_costs_.setStopOnFailure(false);

    // Assuming this planner is being run within the navigation stack, we can
    // just do an upward search for the frequency at which its being run. This
    // also allows the frequency to be overwritten locally.

    std::string controller_frequency_param_name;
    if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
        sim_period_ = 0.05;
    } else {
        double controller_frequency = 0;
        private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
        if (controller_frequency > 0) {
            sim_period_ = 1.0 / controller_frequency;
        } else {
            ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            sim_period_ = 0.05;
        }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    oscillation_costs_.resetOscillationFlags();

    bool sum_scores;
    private_nh.param("sum_scores", sum_scores, false);
    obstacle_costs_.setSumScores(sum_scores);

    private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    map_viz_.initialize(name,
                        planner_util->getGlobalFrame(),
                        [this](int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
                            return getCellCosts(cx, cy, path_cost, goal_cost, occ_cost, total_cost);
                        });

    private_nh.param("global_frame_id", frame_id_, std::string("odom"));

    traj_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1);
    private_nh.param("publish_traj_pc", publish_traj_pc_, false);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    // std::vector<base_local_planner::TrajectoryCostFunction *> critics;
    critics.push_back(&oscillation_costs_);  // discards oscillating motions (assisgns cost -1)
    critics.push_back(&obstacle_costs_);     // discards trajectories that move into obstacles
    critics.push_back(&goal_front_costs_);   // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&alignment_costs_);    // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&path_costs_);         // prefers trajectories on global path
    critics.push_back(&goal_costs_);         // prefers trajectories that go towards (local) goal, based on wave propagation
    critics.push_back(&twirling_costs_);     // optionally prefer trajectories that don't spin

    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator *> generator_list;
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    private_nh.param("cheat_factor", cheat_factor_, 1.0);
}

// used for visualization only, total_costs are not really total costs
bool HybridPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }

    total_cost =
        path_distance_bias_ * path_cost +
        goal_distance_bias_ * goal_cost +
        occdist_scale_ * occ_cost;
    return true;
}

bool HybridPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
    oscillation_costs_.resetOscillationFlags();
    return planner_util_->setPlan(orig_global_plan);
}

/**
 * This function is used when other strategies are to be applied,
 * but the cost functions for obstacles are to be reused.
 */
bool HybridPlanner::checkTrajectory(
    Eigen::Vector3f pos,
    Eigen::Vector3f vel,
    Eigen::Vector3f vel_samples) {
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
                          vel,
                          goal,
                          &limits,
                          vsamples_);
    generator_.generateTrajectory(pos, vel, vel_samples, traj);
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    // if the trajectory is a legal one... the check passes
    if (cost >= 0) {
        return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    // otherwise the check fails
    return false;
}

void HybridPlanner::updatePlanAndLocalCosts(
    const geometry_msgs::PoseStamped &global_pose,
    const std::vector<geometry_msgs::PoseStamped> &new_plan,
    const std::vector<geometry_msgs::Point> &footprint_spec) {
    global_plan_.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
        global_plan_[i] = new_plan[i];
    }

    obstacle_costs_.setFootprint(footprint_spec);

    // costs for going away from path
    path_costs_.setTargetPoses(global_plan_);

    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses(global_plan_);

    // alignment costs
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    double sq_dist =
        (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
        (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
                                               forward_point_distance_ * cos(angle_to_goal);
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
                                                                                              sin(angle_to_goal);

    goal_front_costs_.setTargetPoses(front_global_plan);

    // keeping the nose on the path
    if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) {
        alignment_costs_.setScale(path_distance_bias_);
        // costs for robot being aligned with path (nose on path, not ju
        alignment_costs_.setTargetPoses(global_plan_);
    } else {
        // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
        alignment_costs_.setScale(0.0);
    }
}

bool HybridPlanner::prepareCritics() {
    if (critics.size() == 0) return false;
    for (std::vector<base_local_planner::TrajectoryCostFunction *>::iterator loop_critic = critics.begin(); loop_critic != critics.end(); ++loop_critic) {
        base_local_planner::TrajectoryCostFunction *loop_critic_p = *loop_critic;
        if (loop_critic_p->prepare() == false) {
            ROS_WARN("A scoring function failed to prepare");
            return false;
        }
    }
    return true;
}

/*
* create a graph of points in the close proximity of the robot
* return the best path for the robot to follow using the Dijkstra's algorithm 
*/
std::vector<Node> HybridPlanner::getDijkstraPath(const double &sim_x,
                                                 const double &sim_y,
                                                 const double &sim_th,
                                                 const double &orientation_vel,
                                                 const geometry_msgs::PoseStamped &global_vel) {
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // First the graph's minimum distance(min_dist) and maximum distance (max_dist) is calculated
    double time_to_min_vel = std::abs((limits.min_vel_x - global_vel.pose.position.x) / limits.acc_lim_x);
    double min_dist_time = std::min(sim_period_, time_to_min_vel);
    double min_dist = (global_vel.pose.position.x * min_dist_time) - (0.5 * limits.acc_lim_x * min_dist_time * min_dist_time);

    double min_vel_in_sim_period = std::max(limits.min_vel_x, global_vel.pose.position.x - (limits.acc_lim_x * sim_period_));
    double max_vel_in_sim_period = std::min(limits.max_vel_x, global_vel.pose.position.x + (limits.acc_lim_x * sim_period_));

    double time_to_max_vel = std::abs((limits.max_vel_x - global_vel.pose.position.x) / limits.acc_lim_x);
    double max_dist = 0;

    if (time_to_max_vel < sim_period_) {
        max_dist = (CAP_TIME - time_to_max_vel) * limits.max_vel_x;
        max_dist += (global_vel.pose.position.x * time_to_max_vel) + (0.5 * limits.acc_lim_x * time_to_max_vel * time_to_max_vel);
    } else {
        double max_vel_in_sim_period = global_vel.pose.position.x + (limits.acc_lim_x * sim_period_);
        max_dist = (CAP_TIME - sim_period_) * max_vel_in_sim_period;
        max_dist += (global_vel.pose.position.x * sim_period_) + (0.5 * limits.acc_lim_x * sim_period_ * sim_period_);
    }

    int num_points = (DJK_LAYERS * DJK_LAYER_NODES) + 2;             // Total number of points in the graph
    double delta_dist = (max_dist - min_dist) / (DJK_LAYERS - 1.0);  // Distance between each layer
    double cur_dist = min_dist;

    int edge_count = 0;

    // The graph creation part starts

    std::vector<Node> all_nodes;
    std::map<std::pair<int, int>, int> node_map;
    std::vector<std::vector<Edge>> G(num_points);

    // Single point trajectories are used to calcuate the cost of a point
    // Using existing ROS cost calculating methods
    base_local_planner::Trajectory tj_first;
    tj_first.addPoint(sim_x, sim_y, sim_th);
    double obs_cost_first = obstacle_costs_.scoreTrajectory(tj_first) * obstacle_costs_.getScale();

    Node start_node(0, -1, 0, sim_x, sim_y, sim_th, obs_cost_first, false);
    all_nodes.push_back(start_node);

    for (int layer = 0; layer < DJK_LAYERS; layer++) {
        double left_th, right_th;
        double st = sim_period_ * (layer + 1);

        // For each layer, we calculate the left most theta (left_th) and right most theta (right_th) the mobile robot can move to
        Helper::getLeftRightTh(st, sim_th, orientation_vel, limits.max_vel_theta, limits.acc_lim_theta, left_th, right_th);

        // left_th and right_th is capped to a allowed max value (allowed_max_th)
        double allowed_max_th = (M_PI_2 / DJK_LAYERS) * layer;
        if (Helper::getAngleDist(sim_th, left_th) > allowed_max_th) left_th = Helper::getInRangeAngle(sim_th + allowed_max_th);
        if (Helper::getAngleDist(sim_th, right_th) > allowed_max_th) right_th = Helper::getInRangeAngle(sim_th - allowed_max_th);

        // delta_angle is the angular distance between each point in the graph, for a given layer
        double delta_angle = Helper::getAngleDist(right_th, left_th) / (DJK_LAYER_NODES - 1.0);

        double cur_angle = left_th;

        for (int point_n = 0; point_n < DJK_LAYER_NODES; point_n++) {
            geometry_msgs::Point p;
            // Calculate the coordinate of the new point
            p.x = sim_x + (cur_dist * cos(cur_angle));
            p.y = sim_y + (cur_dist * sin(cur_angle));
            p.z = 0;

            base_local_planner::Trajectory tj;
            tj.addPoint(p.x, p.y, cur_angle);
            // Calculate the cost of the new point
            double obs_cost = obstacle_costs_.scoreTrajectory(tj) * obstacle_costs_.getScale();
            double pth_cost = path_costs_.scoreTrajectory(tj) * path_costs_.getScale();
            double goal_cost = goal_costs_.scoreTrajectory(tj) * goal_costs_.getScale();

            // Check if the node is dead (On an obstacle or such)
            bool dead = false;
            if (obs_cost < 0 || pth_cost < 0 || goal_cost < 0) dead = true;

            int index = all_nodes.size();

            int dead_count = 0;

            if (layer == 0) {
                // For layer 0, the edges are connected to one node in the previous layer

                // Calculate edge cost and add the edge to the graph
                double edge_cost = obs_cost + pth_cost + goal_cost +
                                   (Helper::getScaledDist(sim_x, p.x, sim_y, p.y) * DIST_COST_SCALE);
                G[0].push_back(Edge(index, edge_cost));

                edge_count++;
            } else {
                // For other layers, one node is connected to 3 nodes in the previous layer (if possible)
                int point_1 = point_n;
                int point_0 = point_n - 1;
                int point_2 = point_n + 1;

                std::vector<int> pts;
                pts.push_back(point_1);
                if (point_0 >= 0) pts.push_back(point_0);
                if (point_2 < DJK_LAYER_NODES) pts.push_back(point_2);

                for (int itr = 0; itr < pts.size(); itr++) {
                    int point_prv = pts[itr];
                    int point_prv_index = node_map[{layer - 1, point_prv}];
                    Node prv_node = all_nodes[point_prv_index];

                    // Calculate edge cost and add the edge to the graph
                    double dist = Helper::getScaledDist(p.x, prv_node.pos_x, p.y, prv_node.pos_y);
                    double edge_cost = obs_cost + pth_cost + goal_cost + (dist * DIST_COST_SCALE);
                    if (prv_node.dead) {
                        edge_cost = DEAD_EDGE_COST;
                        dead_count++;
                    }
                    G[point_prv_index].push_back(Edge(index, edge_cost));

                    edge_count++;
                }
                if (layer == DJK_LAYERS - 1) {
                    // The last layer nodes are connected to the destination node
                    if (!dead)
                        G[index].push_back(Edge(num_points - 1, 0));
                    else
                        G[index].push_back(Edge(num_points - 1, DEAD_EDGE_COST));
                }
            }

            if (dead_count == 3) dead = true;
            Node node(index, layer, point_n, p.x, p.y, cur_angle, obs_cost, dead);
            all_nodes.push_back(node);
            node_map.insert({{layer, point_n}, index});

            cur_angle -= delta_angle;
            cur_angle = Helper::getInRangeAngle(cur_angle);
        }
        cur_dist += delta_dist;
    }

    // After creating the graph, Dijkstra's algorithm is used to find the best path

    std::priority_queue<Edge> Q;

    std::vector<double> dist(num_points, 100000000.0);
    std::vector<bool> visited(num_points, false);
    std::vector<int> parent(num_points, -1);

    int start = 0, end = num_points - 1;

    dist[start] = 0;

    Q.push(Edge(start, 0));

    while (!Q.empty()) {
        Edge u = Q.top();
        Q.pop();
        if (u.get_node_index() == end) break;
        if (visited[u.get_node_index()]) continue;

        for (int i = 0; i < G[u.get_node_index()].size(); i++) {
            Edge v = G[u.get_node_index()][i];
            if (visited[v.get_node_index()]) continue;

            if (u.get_cost() + v.get_cost() < dist[v.get_node_index()]) {
                dist[v.get_node_index()] = u.get_cost() + v.get_cost();
                Q.push(Edge(v.get_node_index(), dist[v.get_node_index()]));
                parent[v.get_node_index()] = u.get_node_index();
            }
        }
        visited[u.get_node_index()] = true;
    }

    // Add all the nodes in the shortest path to the nodes_in_path vector
    int current_node = parent[end];

    std::vector<Node> nodes_in_path;

    while (current_node != -1) {
        Node u = all_nodes[current_node];
        nodes_in_path.push_back(u);
        current_node = parent[current_node];
    }

    // Reverse the order of the nodes
    std::reverse(nodes_in_path.begin(), nodes_in_path.end());

    return nodes_in_path;
}

/*
* This function takes the path (vector of Node) from the Dijkstra's algorithm and calculates
* the allowed velocity for the mobile robot, based on the distance to obstacles
*/
void HybridPlanner::getAllowedVelocityAndGoalNodeIndex(std::vector<Node> &nodes_in_path,
                                                       const geometry_msgs::PoseStamped &global_vel,
                                                       const double sim_period_,
                                                       double &allowed_velocity,
                                                       int &goal_node_index) {
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    int itr = 0;
    double max_obs_cost = 0;
    int last_node_index = 0;

    // First find the valid path length and also the max obstacle cost of the path
    for (itr = 0; itr < nodes_in_path.size(); itr++) {
        Node u = nodes_in_path[itr];
        if (u.dead) {
            break;
        }
        last_node_index = itr;
        double obs_cost = u.obstacle_cost;
        if (obs_cost > max_obs_cost) max_obs_cost = obs_cost;
    }

    double velocity_reduction_multiplyer = 0;
    double obs_cost_max_lim = 254.0 * obstacle_costs_.getScale();
    double scaled_max_obs_cost = max_obs_cost / obs_cost_max_lim;

    // Closer the robot is to an obstacle, higher the velocity_reduction_multiplyer will be
    // This is used to reduce the speed of the robot in close proximity to obstacles
    if (scaled_max_obs_cost >= 0.25 && scaled_max_obs_cost <= 0.5) {
        velocity_reduction_multiplyer = ((15 / 0.25) * scaled_max_obs_cost) - 15;
    } else if (scaled_max_obs_cost >= 0.5 && scaled_max_obs_cost <= 0.75) {
        velocity_reduction_multiplyer = ((25 / 0.25) * scaled_max_obs_cost) - 35;
    } else if (scaled_max_obs_cost >= 0.75) {
        velocity_reduction_multiplyer = ((35 / 0.25) * scaled_max_obs_cost) - 65;
    }
    velocity_reduction_multiplyer *= 0.01;

    allowed_velocity = limits.max_vel_x - (limits.max_vel_x * velocity_reduction_multiplyer);

    if (allowed_velocity < global_vel.pose.position.x) {
        double tm = (global_vel.pose.position.x - allowed_velocity) / limits.acc_lim_x;
        if (tm > sim_period_) {
            allowed_velocity = global_vel.pose.position.x - (limits.acc_lim_x * sim_period_);
        }
    } else {
        double tm = (allowed_velocity - global_vel.pose.position.x) / limits.acc_lim_x;
        if (tm > sim_period_) {
            allowed_velocity = global_vel.pose.position.x + (limits.acc_lim_x * sim_period_);
        }
    }

    // Using the allowed velocity, calculate the allowed distance and the index to the goal node
    double allowed_dist = allowed_velocity * CAP_TIME;


    for (itr = 1; itr <= last_node_index; itr++) {
        double m_dist = Helper::getScaledDist(nodes_in_path[itr], nodes_in_path[itr - 1]);
        allowed_dist -= m_dist;
        if (allowed_dist < 0) {
            break;
        }
    }
    goal_node_index = itr;
}

/*
 * Given the current state of the robot, find a good trajectory 
 * Also provide the velocities for robot to follow
 */
base_local_planner::Trajectory HybridPlanner::findBestPath(
    const geometry_msgs::PoseStamped &global_pose,
    const geometry_msgs::PoseStamped &global_vel,
    geometry_msgs::PoseStamped &drive_velocities) {
    // make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(configuration_mutex_);

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // // prepare cost functions and generators for this run
    generator_.initialise(pos, vel, goal, &limits, vsamples_);

    bool invalid = false;

    if (!prepareCritics()) {
        invalid = true;
    } else {
        double v_x, v_th;

        astar_path.points.clear();

        // First, predict where the robot will be after the PROCESSING_TIME
        double orientation = Helper::getInRangeAngle(tf2::getYaw(global_pose.pose.orientation));
        double orientation_vel = tf2::getYaw(global_vel.pose.orientation);

        double sim_dist = global_vel.pose.position.x * PROCESSING_TIME;
        double sim_th_dist = orientation_vel * PROCESSING_TIME;
        double sim_th = Helper::getInRangeAngle(orientation + sim_th_dist);
        double sim_x = global_pose.pose.position.x + (sim_dist * cos(sim_th));
        double sim_y = global_pose.pose.position.y + (sim_dist * sin(sim_th));

        // Find the best path for the robot to follow
        std::vector<Node> nodes_in_path = getDijkstraPath(sim_x, sim_y, sim_th, orientation_vel, global_vel);

        double allowed_velocity = 0;
        int goal_node_index = 0;

        double goal_x = goal_pose.pose.position.x;
        double goal_y = goal_pose.pose.position.y;
        double goal_th = Helper::getInRangeAngle(tf2::getYaw(goal_pose.pose.orientation));

        double goal_dist = std::hypot(sim_x - goal_x, sim_y - goal_y);
        double dec_dist = limits.max_vel_x * GOAL_DEC_TIME;

        Node goal_node = nodes_in_path[0];
        double goal_head_angle = 0;
        double allowed_dist;
        double delta_th = M_PI / (NUM_ANGLES * 0.5);

        // Find out the goal position and heading for the mobile robot, and the allowed velocity 

        // If the actual goal is within reach, use that goal as destination. The velocity is selected based on the distance to the goal
        if (dec_dist > goal_dist) {
            double dec_multiplyer = (goal_dist / dec_dist);
            allowed_velocity = limits.max_vel_x * dec_multiplyer;
            allowed_dist = goal_dist;

            goal_node.pos_x = goal_x;
            goal_node.pos_y = goal_y;
            goal_node.angle = goal_th;
            goal_head_angle = goal_th;
        } else {
            // If the actual goal is not within reach, select a velcity and a suitable point in the Dijkstra's path as goal
            getAllowedVelocityAndGoalNodeIndex(nodes_in_path, global_vel, sim_period_, allowed_velocity, goal_node_index);
            goal_node = nodes_in_path[goal_node_index];
            allowed_dist = allowed_velocity * CAP_TIME;

            // After we have the goal point, calculate the best heading for that point

            double min_cost_head_angle = 20000000.0;

            double goal_head_th = goal_node.angle;
            goal_head_th = Helper::getInRangeAngle(goal_head_th);

            double PI_3 = (M_PI / 3);

            double temp_left_angle = Helper::getInRangeAngle(goal_head_th + PI_3);
            double temp_right_angle = Helper::getInRangeAngle(goal_head_th - PI_3);

            double goal_left_angle = std::floor(temp_left_angle / delta_th) * delta_th;
            double goal_right_angle = std::ceil(temp_right_angle / delta_th) * delta_th;

            int num_iters = Helper::getAngleDist(goal_left_angle, goal_right_angle) / delta_th;
            double head_angle = goal_left_angle;

            for (int itr = 0; itr <= num_iters; itr++, head_angle -= delta_th) {
                head_angle = Helper::getInRangeAngle(head_angle);
                base_local_planner::Trajectory tj;
                tj.addPoint(goal_node.pos_x, goal_node.pos_y, head_angle);
                double alignment_cost = alignment_costs_.scoreTrajectory(tj) * alignment_costs_.getScale();
                double total_cost = alignment_cost;
                if (total_cost < min_cost_head_angle) {
                    min_cost_head_angle = total_cost;
                    goal_head_angle = head_angle;
                }
            }

            double min_vel_in_sim_period = std::max(limits.min_vel_x, global_vel.pose.position.x - (limits.acc_lim_x * sim_period_));
            double max_vel_in_sim_period = std::min(limits.max_vel_x, global_vel.pose.position.x + (limits.acc_lim_x * sim_period_));

            if (allowed_velocity > max_vel_in_sim_period) allowed_velocity = max_vel_in_sim_period;
            if (allowed_velocity < min_vel_in_sim_period) allowed_velocity = min_vel_in_sim_period;
        }

        // At this point, we have the goal node, the desired heading for the goal node and the X velocity of the robot
        // Now, we will use the Hybrid A star algorithm to find the Theta velocity for the robot.   

        double min_x = std::min(sim_x, goal_node.pos_x) - 0.1;
        double min_y = std::min(sim_y, goal_node.pos_y) - 0.1;
        double max_x = std::max(sim_x, goal_node.pos_x) + 0.1;
        double max_y = std::max(sim_y, goal_node.pos_y) + 0.1;

        // The X, Y and Theta values are converted to discrete cells

        double delta_x = (max_x - min_x) / NUM_X_DIVS;
        double delta_y = (max_y - min_y) / NUM_Y_DIVS;

        int begin_cell_x = (sim_x - min_x) / delta_x;
        int begin_cell_y = (sim_y - min_y) / delta_y;
        int begin_cell_th = sim_th / delta_th;
        // Each cell is represented using a hash value. This helps to keep track which cells are visited
        std::string begin_hash = Helper::getHash(begin_cell_x, begin_cell_y, begin_cell_th);

        double begin_x = min_x + (begin_cell_x * delta_x);
        double begin_y = min_y + (begin_cell_y * delta_y);
        double begin_th = begin_cell_th * delta_th;

        int end_cell_x = (goal_node.pos_x - min_x) / delta_x;
        int end_cell_y = (goal_node.pos_y - min_y) / delta_y;
        int end_cell_th = goal_head_angle / delta_th;
        std::string end_hash = Helper::getHash(end_cell_x, end_cell_y, end_cell_th);

        double end_x = min_x + (end_cell_x * delta_x);
        double end_y = min_y + (end_cell_y * delta_y);
        double end_th = end_cell_th * delta_th;

        AStar_Node last, lowest;
        double lowest_hr_cost = 20000000;

        double vel_th_min = -limits.max_vel_theta;
        double vel_th_max = limits.max_vel_theta;
        double astar_sim_period = allowed_dist / allowed_velocity;
        
        // At the start, we consider time period of larger values. This results in the A Star algorithm having 
        // larger edges in the beginning
        double cur_sim_period = astar_sim_period / 2;
        double min_sim_period = astar_sim_period / std::pow(2, 5);

        int th_dist_divs = 30;

        std::clock_t hastar_start, hastar_ctime;
        hastar_start = std::clock();

        // The Hybrid A star algorithm is allowed to run for a specific period of time (allowed_time).
        // We consider the closes node the Hybrid A star algorithm reaches in this time period
        double allowed_time = 0.03;
        int node_count = 0;

        AStar_Node begin_node(0, begin_hash, begin_x, begin_y, begin_th, cur_sim_period, 0, -1, 0, orientation_vel);

        // The Hybrid A star algorithm begins to find the best path between the starting node and the goal node

        std::priority_queue<AStar_Node> Q2;

        std::map<std::string, int> visited_map;
        std::map<std::string, double> cost_map;
        std::vector<AStar_Node> nodes;

        nodes.push_back(begin_node);
        Q2.push(begin_node);

        bool first_node = true;
        bool found = false;

        while (!Q2.empty()) {
            AStar_Node u = Q2.top();
            Q2.pop();

            node_count++;
            if (node_count % 10 == 0) {
                hastar_ctime = std::clock();
                double hastar_time = double(hastar_ctime - hastar_start) / double(CLOCKS_PER_SEC);
                if (hastar_time > allowed_time) break;
            }

            if (u.hash == end_hash) {
                last = u;
                found = true;
                break;
            }

            if (visited_map.find(u.hash) != visited_map.end()) continue;
            visited_map[u.hash] = 1;

            if (!first_node && u.hr_cost < lowest_hr_cost) {
                lowest_hr_cost = u.hr_cost;
                lowest = u;
            }

            cur_sim_period = u.sim_time;

            double left_th, right_th;

            double vel_th = u.number == 0 ? u.vel_th : 0;

            // For each node, find the left-most and the right-most nodes it can reach
            Helper::getLeftRightTh(cur_sim_period, u.heading_angle, vel_th, limits.max_vel_theta, limits.acc_lim_theta, left_th, right_th);

            double delta_dist = Helper::getAngleDist(left_th, right_th) / th_dist_divs;

            int ni = -1;
            double cur_dist = cur_sim_period * allowed_velocity;

            // Sample nodes which are in between the left-most and the right-most nodes.
            for (int i = th_dist_divs / 2; i <= th_dist_divs; i += ni) {
                if (i < 0) {
                    ni = 1;
                    i = (th_dist_divs / 2);
                    continue;
                }

                double new_th = Helper::getInRangeAngle(left_th - (i * delta_dist));
                int new_cell_th = new_th / delta_th;
                new_th = new_cell_th * delta_th;
                double new_x = u.pos_x + (cur_dist * cos(new_th));
                double new_y = u.pos_y + (cur_dist * sin(new_th));
                int new_cell_x = (new_x - min_x) / delta_x;
                int new_cell_y = (new_y - min_y) / delta_y;
                new_x = min_x + (new_cell_x * delta_x);
                new_y = min_y + (new_cell_y * delta_y);

                // if(new_x < min_x || new_y < min_y || new_x > max_x || new_y > max_y) continue;

                // Calculate the hash of the new node
                std::string new_hash = Helper::getHash(new_cell_x, new_cell_y, new_cell_th);

                base_local_planner::Trajectory tj2;
                tj2.addPoint(new_x, new_y, new_th);
                double obs_cost = obstacle_costs_.scoreTrajectory(tj2) * 0.001;

                if (obs_cost < 0) continue;

                // Calculate the cost of the new node
                double parent_th_dist = Helper::getAngleDist(new_th, u.heading_angle);
                double parent_dist_cost = std::hypot(new_x - u.pos_x, new_y - u.pos_y) + (parent_th_dist * 0.5);
                double hr_cost = std::hypot(new_x - end_x, new_y - end_y) + (Helper::getAngleDist(new_th, goal_head_angle) * 0.1);

                double vel_th = 0;

                double total_cost = u.cost - u.hr_cost + obs_cost + parent_dist_cost + hr_cost;

                bool add_node = false;

                if (cost_map.find(new_hash) == cost_map.end()) {
                    // Add the new node to the priority queue, if the node is not yet visited
                    cost_map[new_hash] = total_cost - hr_cost;
                    add_node = true;
                } else {
                    // Add the node to the priority queue if the new cost of the node is lower than the preivous cost
                    double prv_cost = cost_map[new_hash];
                    if (prv_cost < total_cost) {
                        continue;
                    }
                    add_node = true;
                    cost_map[new_hash] = total_cost - hr_cost;
                }

                if (add_node) {
                    double tm = std::max(cur_sim_period / 2, min_sim_period);
                    AStar_Node new_node(nodes.size(), new_hash, new_x, new_y, new_th, tm, total_cost, u.number, hr_cost, vel_th);
                    nodes.push_back(new_node);
                    Q2.push(new_node);
                }
            }

            if (first_node) {
                first_node = false;
                th_dist_divs = 4;
            }
        }

        // Detect the best path found by the Hybrid A star algorithm and calculate the Theta velocity using the path

        AStar_Node final_node, node_itr;
        if (found)
            final_node = last;
        else
            final_node = lowest;

        node_itr = final_node;

        result_traj_.resetPoints();

        while (true) {
            geometry_msgs::Point px;
            px.x = node_itr.pos_x;
            px.y = node_itr.pos_y;
            px.z = 0;

            result_traj_.addPoint(node_itr.pos_x, node_itr.pos_y, node_itr.heading_angle);

            astar_path.points.push_back(px);  // For path visualization
            if (node_itr.number != final_node.number && node_itr.parent != -1) {
                astar_path.points.push_back(px); // For path visualization
            }
            if (node_itr.parent == -1) break;
            if (nodes[node_itr.parent].parent == -1) { 
                double node_orientation = Helper::getInRangeAngle(node_itr.heading_angle);
                double ori_dist = std::abs(Helper::getAngleDist(sim_th, node_orientation));

                double o1 = Helper::getInRangeAngle(ori_dist + sim_th);

                if (Helper::getAngleDist(o1, node_orientation) < 1e-5) {
                    v_th = (ori_dist / (astar_sim_period / 2));
                } else {
                    v_th = (-ori_dist / (astar_sim_period / 2));
                }
                v_th = v_th * VEL_TH_MULTIPLYER;
            }
            node_itr = nodes[node_itr.parent];
        }

        v_x = allowed_velocity;

        astar_path.header.stamp = ros::Time::now();
        marker_pub.publish(astar_path);

        // Publish the calculated velocities to ROS
        result_traj_.xv_ = v_x;
        result_traj_.yv_ = 0;
        result_traj_.thetav_ = v_th;
        result_traj_.cost_ = 100;
    }

    // verbose publishing of point clouds
    if (publish_cost_grid_pc_) {
        // we'll publish the visualization of the costs to rviz before returning our best trajectory
        map_viz_.publishCostCloud(planner_util_->getCostmap());
    }

    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_vel_trans);

    // if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0 || invalid) {
        drive_velocities.pose.position.x = 0;
        drive_velocities.pose.position.y = 0;
        drive_velocities.pose.position.z = 0;
        drive_velocities.pose.orientation.w = 1;
        drive_velocities.pose.orientation.x = 0;
        drive_velocities.pose.orientation.y = 0;
        drive_velocities.pose.orientation.z = 0;
    } else {
        drive_velocities.pose.position.x = result_traj_.xv_;
        drive_velocities.pose.position.y = result_traj_.yv_;
        drive_velocities.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, result_traj_.thetav_);
        tf2::convert(q, drive_velocities.pose.orientation);
    }

    return result_traj_;
}
};  // namespace hybrid_local_planner
