#ifndef HYBRID_NODE_H_
#define HYBRID_NODE_H_

#include <string>

namespace hybrid_local_planner {
  class Node {
    public:
      Node(int p_index, int p_layer_no, int p_node_no, double p_pos_x, double p_pos_y, double p_angle, double p_obstacle_cost, bool p_dead);

      int index;
      int layer_no;
      int node_no;
      double pos_x;
      double pos_y;
      double angle;
      double obstacle_cost;
      bool dead;

  };
};
#endif
