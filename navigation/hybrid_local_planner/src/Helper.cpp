#include <Helper.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace hybrid_local_planner {

// Given two node objects, return the distance between the nodes
double Helper::getScaledDist(hybrid_local_planner::Node &n1, hybrid_local_planner::Node &n2) {
    double dist = std::hypot(n1.pos_x - n2.pos_x, n1.pos_y - n2.pos_y);
    return dist;
}

// Given the x and y coordinates of two points, return the distance between the two points
double Helper::getScaledDist(double x1, double x2, double y1, double y2) {
    double dist = std::hypot(x1 - x2, y1 - y2);
    return dist;
}

// Find the (smallest) angular distance between two points 
double Helper::getAngleDist(double a1, double a2) {
    return std::min(std::abs(a1 - a2),
                    std::min(std::abs(a1 - a2 - (M_PI * 2)), std::abs(a1 - a2 + (M_PI * 2))));
}

// Given an angle in radian, return the corresponding angle value in the range 0 to 2 * PI
double Helper::getInRangeAngle(double angle) {
    double m_pi_2 = M_PI * 2;
    if (angle < 0) return angle + m_pi_2;
    if (angle > m_pi_2) return angle - m_pi_2;
    return angle;
}

// Given three integers, return their hash 
std::string Helper::getHash(int n1, int n2, int n3) {
    std::stringstream ss;
    ss << n1 << n2 << n3;
    return ss.str();
}

// Given the state of the robot, return the left most and right most angle (theta) it can achieve in the given time period
void Helper::getLeftRightTh(double time, double orientation, double cur_vel, double max_velocity, double acc, double &left_th, double &right_th) {
    double time_to_max_vel_th_left = std::abs((max_velocity - cur_vel) / acc);
    double time_to_max_vel_th_right = std::abs((-max_velocity - cur_vel) / acc);

    if (time_to_max_vel_th_left < time) {
        left_th = orientation + ((cur_vel * time_to_max_vel_th_left) + (0.5 * acc * time_to_max_vel_th_left * time_to_max_vel_th_left));
        left_th = left_th + (max_velocity * (time - time_to_max_vel_th_left));
    } else {
        left_th = orientation + ((cur_vel * time) + (0.5 * acc * time * time));
    }

    if (time_to_max_vel_th_right < time) {
        right_th = orientation + ((cur_vel * time_to_max_vel_th_right) - (0.5 * acc * time_to_max_vel_th_right * time_to_max_vel_th_right));
        right_th = right_th - (max_velocity * (time - time_to_max_vel_th_right));
    } else {
        right_th = orientation + ((cur_vel * time) - (0.5 * acc * time * time));
    }

    left_th = Helper::getInRangeAngle(left_th);
    right_th = Helper::getInRangeAngle(right_th);
}
}; 
