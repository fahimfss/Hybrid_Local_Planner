#include <Node.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace hybrid_local_planner {
Node::Node(int p_index, int p_layer_no, int p_node_no, double p_pos_x, double p_pos_y, double p_angle, double p_obstacle_cost, bool p_dead) {
    index = p_index;
    layer_no = p_layer_no;
    node_no = p_node_no;
    pos_x = p_pos_x;
    pos_y = p_pos_y;
    angle = p_angle;
    obstacle_cost = p_obstacle_cost;
    dead = p_dead;
}
};  // namespace hybrid_local_planner