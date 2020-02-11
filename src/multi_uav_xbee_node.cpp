#include <string>
#include <thread>
#include <mutex>
#include <map>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <multi_uav_xbee/parser.h>
#include <multi_uav_xbee/cserial.h>
#include <multi_uav_xbee/MUXPose.h>
#include <multi_uav_xbee/MUXStatistics.h>

// statistics
int packetLoss = 0;
int packetReceived = 0;
int sentMessageCount = 0;
int numberOfNodes = 0;

// node
int droneId = 0;
multi_uav_xbee::CSerial *serial;
multi_uav_xbee::MUXPose localMUXPose;

enum MSG_TYPE {
  MSG_TYPE_BEACON,
  MSG_TYPE_SET_POSITION
};

enum DESTINATION_ID {
  BROADCAST_DESTINATION_ID = -1
};

void mavrosLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  geometry_msgs::PoseStamped current = *msg;

  //position
  localMUXPose.position.x = current.pose.position.x;
  localMUXPose.position.y = current.pose.position.y;
  localMUXPose.position.z = current.pose.position.z;

  // convert quaterniun to euler
  tf::Quaternion q(current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //orientation
  localMUXPose.orientation.roll = roll;
  localMUXPose.orientation.pitch = pitch;
  localMUXPose.orientation.yaw = yaw;

}

void rosLoop(ros::NodeHandle nh){
  std::stringstream topicName;
  topicName << "/uav";
  topicName << droneId;
  topicName << "/mavros/local_position/pose";
  ros::Subscriber localPositionPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>(topicName.str(), 1, mavrosLocalPositionCallback);

  std::stringstream statisticsTopicName;
  statisticsTopicName << "/uav_network/statistics";

  ros::Publisher statisticsPublisher = nh.advertise<multi_uav_xbee::MUXStatistics>(statisticsTopicName.str(), 1);

  ros::Rate rate(20.0);
  while(ros::ok()){
    multi_uav_xbee::MUXStatistics muxStatistics;
    muxStatistics.packetLoss = packetLoss;
    muxStatistics.packetReceived = packetReceived;
    muxStatistics.sentMessageCount = sentMessageCount;
    muxStatistics.numberOfNodes = numberOfNodes;
    statisticsPublisher.publish(muxStatistics);

    ros::spinOnce();
    rate.sleep();
  }

  localPositionPoseSubscriber.shutdown();
  statisticsPublisher.shutdown();

  serial->closePort();
}

void publishLocalNodeDataInLocalUAVROS(ros::NodeHandle nh){

  std::stringstream positionTopic;
  positionTopic << "/uav_network/";
  positionTopic << droneId;
  positionTopic << "/pose";

  ros::Publisher localPosePublisher = nh.advertise<geometry_msgs::Pose>(positionTopic.str(), 1);

  ros::Rate rate(1);
  while(ros::ok()){

    geometry_msgs::Pose pose;

    pose.position.x = localMUXPose.position.x;
    pose.position.y = localMUXPose.position.y;
    pose.position.z = localMUXPose.position.z;

    // transforming from RPY to Quaternion (http://wiki.ros.org/tf2/Tutorials/Quaternions)
    tf2::Quaternion myQuaternion;
    // Create this quaternion from roll/pitch/yaw (in radians)
    myQuaternion.setRPY(localMUXPose.orientation.roll, localMUXPose.orientation.pitch, localMUXPose.orientation.yaw);

    pose.orientation = tf2::toMsg(myQuaternion);

    localPosePublisher.publish(pose);

    rate.sleep();
  }

  localPosePublisher.shutdown();
}

void sendBeaconMessageByXbee(){

  multi_uav_xbee::Parser *parser = new multi_uav_xbee::Parser();

  ros::Rate rate(1);
  while(ros::ok()){

    multi_uav_xbee::MUXPose muxPose;

    // copy local message to mux message
    muxPose = localMUXPose;

    //header
    muxPose.header.id = sentMessageCount++;
    muxPose.header.source_node_id = droneId;
    muxPose.header.destination_node_id = BROADCAST_DESTINATION_ID;
    muxPose.header.message_id = MSG_TYPE_BEACON;

    //send message to xbee
    serial->writeData(parser->serialize(muxPose));

    rate.sleep();
  }
}

void setPositionCallback(const ros::MessageEvent<geometry_msgs::Pose const> & event){

  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");

  const geometry_msgs::Pose::ConstPtr& msg = event.getMessage();
  geometry_msgs::Pose current = *msg;

  //get destination node id by topic name
  std::stringstream sstrDroneId;
  size_t foundStart = topic.find("uav_network");

  size_t found = topic.find_first_of('/', foundStart);
  size_t found2 = topic.find('/', found+1);

  for(size_t i = found+1; i<found2; i++){
      sstrDroneId << topic[i];
  }

  int destinationNodeId;

  sstrDroneId >> destinationNodeId;

  if(destinationNodeId != droneId){

    multi_uav_xbee::MUXPose muxPose;
    //header
    muxPose.header.id = sentMessageCount++;
    muxPose.header.source_node_id = droneId;
    muxPose.header.destination_node_id = destinationNodeId;
    muxPose.header.message_id = MSG_TYPE_BEACON;
    // position
    muxPose.position.x = current.position.x;
    muxPose.position.y = current.position.y;
    muxPose.position.z = current.position.z;

    //get euler angles from quaternium
    tf::Quaternion q(current.orientation.x, current.orientation.y, current.orientation.z, current.orientation.w);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //orientation
    muxPose.orientation.roll = roll;
    muxPose.orientation.pitch = pitch;
    muxPose.orientation.yaw = yaw;

    multi_uav_xbee::Parser *parser = new multi_uav_xbee::Parser();

    //send message to xbee
    serial->writeData(parser->serialize(muxPose));
  }
}

void receiveMessageByXbee(ros::NodeHandle nh){

  multi_uav_xbee::Parser *parser = new multi_uav_xbee::Parser();

  std::map<int, int> nodes;
  std::map<int, ros::Publisher> posePublisherMap;
  std::map<int, ros::Publisher> setPosePublisherMap;
  std::map<int, ros::Subscriber> setPoseSubscriberMap;

  // attach set position subscriber to droneId
  if(!setPoseSubscriberMap[droneId]){
    nodes[droneId] = droneId;
    std::stringstream setPositionTopic;
    setPositionTopic << "/uav_network/";
    setPositionTopic << droneId;
    setPositionTopic << "/set_pose";
    setPoseSubscriberMap[droneId] = nh.subscribe(setPositionTopic.str(), 1, setPositionCallback);
  }

  // ros::Rate rate(1);
  while(ros::ok()){

    std::string msgData = serial->readData();

    //std::cout << "Received data: " << msgData << std::endl;

    try{

      multi_uav_xbee::MUXPose msg = parser->deserialize(msgData);

      if(msg.header.message_id == MSG_TYPE_BEACON){

        std::stringstream positionTopic;
        positionTopic << "/uav_network/";
        positionTopic << msg.header.source_node_id;
        positionTopic << "/pose";

        //create a publisher if it not exists
        if(!posePublisherMap[msg.header.source_node_id]){
          nodes[msg.header.source_node_id] = msg.header.source_node_id;
          posePublisherMap[msg.header.source_node_id] = nh.advertise<geometry_msgs::Pose>(positionTopic.str(), 1);
        }

        // http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html
        geometry_msgs::Pose pose;

        pose.position.x = msg.position.x;
        pose.position.y = msg.position.y;
        pose.position.z = msg.position.z;

        // transforming from RPY to Quaternion (http://wiki.ros.org/tf2/Tutorials/Quaternions)
        tf2::Quaternion myQuaternion;
        // Create this quaternion from roll/pitch/yaw (in radians)
        myQuaternion.setRPY(msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw);

        pose.orientation = tf2::toMsg(myQuaternion);

        posePublisherMap[msg.header.source_node_id].publish(pose);

        // attach callback to source_node
        if(!setPoseSubscriberMap[msg.header.source_node_id]){
          std::stringstream setPositionTopic;
          setPositionTopic << "/uav_network/";
          setPositionTopic << msg.header.source_node_id;
          setPositionTopic << "/set_pose";
          setPoseSubscriberMap[msg.header.source_node_id] = nh.subscribe(setPositionTopic.str(), 1, setPositionCallback);
        }

      }
      else if(msg.header.message_id == MSG_TYPE_SET_POSITION){

        // create set_position ros topic callback if it not exists

        if(msg.header.destination_node_id == droneId){
          std::stringstream setPositionTopic;
          setPositionTopic << "/uav_network/";
          setPositionTopic << droneId;
          setPositionTopic << "/set_pose";

          //create a publisher if it not exists
          if(!setPosePublisherMap[msg.header.source_node_id]){
            setPosePublisherMap[msg.header.source_node_id] = nh.advertise<geometry_msgs::Pose>(setPositionTopic.str(), 1);
          }

          // http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html
          geometry_msgs::Pose pose;

          pose.position.x = msg.position.x;
          pose.position.y = msg.position.y;
          pose.position.z = msg.position.z;

          // transforming from RPY to Quaternion (http://wiki.ros.org/tf2/Tutorials/Quaternions)
          tf2::Quaternion myQuaternion;
          // Create this quaternion from roll/pitch/yaw (in radians)
          myQuaternion.setRPY(msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw);

          pose.orientation = tf2::toMsg(myQuaternion);

          setPosePublisherMap[droneId].publish(pose);

        }
        else{
          std::cout << "Set position message: different destination node." << std::endl;
        }

      }
      else{
        std::cout << "Invalid message ID." << std::endl;
      }

      packetReceived++;

    }catch(std::exception &e){
      packetLoss++;
      std::cout << "Message deserealization error." << std::endl;
    }

    numberOfNodes = nodes.size();

    // rate.sleep();
  }

  //shutdown all position publishers
  for(std::map<int, ros::Publisher>::iterator it=posePublisherMap.begin(); it!=posePublisherMap.end(); it++){
    posePublisherMap[it->first].shutdown();
  }

  //shutdown all setposition publishers
  for(std::map<int, ros::Publisher>::iterator it=setPosePublisherMap.begin(); it!=setPosePublisherMap.end(); it++){
    setPosePublisherMap[it->first].shutdown();
  }

  //shutdown all setposition subscribers
  for(std::map<int, ros::Subscriber>::iterator it=setPoseSubscriberMap.begin(); it!=setPoseSubscriberMap.end(); it++){
    setPoseSubscriberMap[it->first].shutdown();
  }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "multi_uav_xbee_node");
  ros::NodeHandle nh("~");

  if(nh.hasParam("droneId")){

    nh.getParam("droneId", droneId);

    if(nh.hasParam("serialPort")){

      std::string port;
      nh.getParam("serialPort", port);

      serial = new multi_uav_xbee::CSerial();

      int baud = 9600;

      if(nh.hasParam("baud")){
        nh.getParam("baud", baud);
        std::cout << "Baud rate set to " << baud << std::endl;
      }
      else{
        std::cout << "Baud rate set to " << baud << ". Please use _baud:=BAUD at the end of the command to change baud value." << std::endl;
      }

      if(serial->openPort(port, baud)){

        // create all threads
        std::thread *rosLoopThread = new std::thread(rosLoop, nh);
        std::thread *publishLocalNodeDataInLocalUAVROSThread = new std::thread(publishLocalNodeDataInLocalUAVROS, nh);
        std::thread *sendBeaconMessageByXbeeThread = new std::thread(sendBeaconMessageByXbee);
        std::thread *receiveMessageByXbeeThread = new std::thread(receiveMessageByXbee, nh);

        rosLoopThread->join();
        publishLocalNodeDataInLocalUAVROSThread->join();
        sendBeaconMessageByXbeeThread->join();
        receiveMessageByXbeeThread->join();
      }
      else{
        std::cout << "Could not connect to serial port!" << std::endl;
      }
    }
    else{
      std::cout << "Serial port not found. Please use _serialPort:=PORT at the end of the command." << std::endl;
    }
  }
  else{
    std::cout << "Drone ID not found. Please use _droneId:=ID at the end of the command." << std::endl;
  }

  return 0;
}
