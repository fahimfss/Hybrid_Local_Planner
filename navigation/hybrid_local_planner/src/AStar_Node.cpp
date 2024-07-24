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


#include <AStar_Node.h>

namespace hybrid_local_planner {
AStar_Node::AStar_Node(int p_number, std::string p_hash, double p_pos_x, double p_pos_y, double p_heading_angle, double p_sim_time, double p_cost, int p_parent, double p_hr_cost, double p_vel_th) {
    number = p_number;
    hash = p_hash;
    pos_x = p_pos_x;
    pos_y = p_pos_y;
    heading_angle = p_heading_angle;
    sim_time = p_sim_time;
    cost = p_cost;
    parent = p_parent;
    hr_cost = p_hr_cost;
    vel_th = p_vel_th;
}

AStar_Node::AStar_Node() {
}

bool AStar_Node::operator<(const AStar_Node &e) const {
    return cost > e.cost;
}

};  