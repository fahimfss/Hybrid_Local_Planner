#ifndef HYBRID_HELPER_H_
#define HYBRID_HELPER_H_

#include <Node.h>
#include <string>

namespace hybrid_local_planner {
  class Helper {
    public:
      static double getScaledDist(double x1, double x2, double y1, double y2);
      static double getScaledDist(Node &n1, Node &n2);
      static double getInRangeAngle(double angle);
      static std::string getHash(int n1, int n2, int n3);
      static void getLeftRightTh(double time, double orientation, double cur_vel, double max_velocity, double max_acc, double &left_th, double &right_th);
      static double getAngleDist(double a1, double a2);
  };
};
#endif
