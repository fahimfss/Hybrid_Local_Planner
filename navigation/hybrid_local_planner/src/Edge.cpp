#include <Edge.h>

namespace hybrid_local_planner {
    Edge::Edge(int p_node_index, double p_cost){
        node_index = p_node_index;
        cost = p_cost;
    }

    int Edge::get_node_index(){
        return node_index;
    }
    
    double Edge::get_cost(){
        return cost;
    }

    bool Edge::operator<(const hybrid_local_planner::Edge &e)const{
        return cost > e.cost;
    }
};