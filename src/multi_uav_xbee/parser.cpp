#include "multi_uav_xbee/parser.h"

namespace multi_uav_xbee{

Parser::Parser(){

}

std::string Parser::serialize(multi_uav_xbee::MUXPose muxPose){

  //Serialize the "raw" message into a buffer
  size_t msg_length = ros::serialization::serializationLength(muxPose);
  std::vector<uint8_t> buffer(msg_length);
  ros::serialization::OStream stream(buffer.data(), buffer.size());
  ros::serialization::serialize(stream, muxPose);

  std::string data;
  for (int i = 0; i < buffer.size(); ++i) {
    data += (char) buffer[i];
  }

  return data;
}

multi_uav_xbee::MUXPose Parser::deserialize(std::string message){
  // populate the buffer
  std::vector<uint8_t> buffer;

  for (int i = 0; i < message.length(); ++i) {
    buffer.push_back(message[i]);
  }

  //deserealize the message
  multi_uav_xbee::MUXPose muxPose;
  ros::serialization::IStream stream(buffer.data(), buffer.size());
  ros::serialization::deserialize(stream, muxPose);

  return muxPose;

}

}
