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