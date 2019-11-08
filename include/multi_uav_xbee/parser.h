#ifndef PARSER_H
#define PARSER_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <multi_uav_xbee/MUXPose.h>

namespace multi_uav_xbee{

class Parser{
public:
  Parser();
  std::string serialize(multi_uav_xbee::MUXPose muxPose);
  multi_uav_xbee::MUXPose deserialize(std::string message);
};

}

#endif // PARSER_H
