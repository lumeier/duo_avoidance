
/**
 *  duo_avoidance_node.cpp
 *  author: Lukas Meier <lukasdanmeier@gmail.com>
 *  node for duo_avoidance
 */

#include "duo_avoidance.hpp"

#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "duo_avoidance");

  duo_avoidance::DuoAvoidance duo_avoidance;

  ros::spin();
  return 0;
}
