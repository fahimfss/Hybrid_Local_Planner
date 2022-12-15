#ifndef HYBRID_EDGE_H_
#define HYBRID_EDGE_H_

namespace hybrid_local_planner {
  class Edge {
    public:
      Edge(int p_node_index, double p_cost);

      int get_node_index();
      double get_cost();

      bool operator<(const hybrid_local_planner::Edge &e) const;

    private:
      int node_index;
      double cost;


  };
};
#endif
