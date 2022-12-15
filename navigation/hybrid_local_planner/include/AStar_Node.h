#ifndef HYBRID_ASTR_NODE_H_
#define HYBRID_ASTR_NODE_H_

#include <string>

namespace hybrid_local_planner {
  class AStar_Node{
    public:
      AStar_Node(int p_number, std::string p_hash, double p_pos_x, double p_pos_y, double p_heading_angle, double p_sim_time, double p_cost, int p_parent, double p_hr_cost, double p_vel_th);
      AStar_Node();
      int number;
      std::string hash;
      double pos_x;
      double pos_y;
      double heading_angle;
      double sim_time;
      double cost;
      int parent;
      double hr_cost;
      double vel_th;

      bool operator<(const AStar_Node &e) const;
  };
};
#endif
