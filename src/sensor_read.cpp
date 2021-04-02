#include "sensor_read.h"
#include "utils.h"
#include <array>

#include <std_srvs/Trigger.h>


////////////////////////
sensor_read::sensor_read(ros::NodeHandle & node)
{
  node_ = node;
}

////////////////////////
void sensor_read::init() {

  // Subscribe to the '/ariac/competition_state' topic.
  ROS_INFO("Subscribe to the /ariac/logical_camera_0 topic...");
  // Subscribe to the '/ariac/logical_camera_0' Topic.
  logical_camera_0_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_0", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 0));

  // Subscribe to the '/ariac/logical_camera_1' Topic.
  logical_camera_1_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_1", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 1));

    // Subscribe to the '/ariac/logical_camera_2' Topic.
  logical_camera_2_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_2", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 2));

    // Subscribe to the '/ariac/logical_camera_3' Topic.
  logical_camera_3_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_3", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 3));

    // Subscribe to the '/ariac/logical_camera_4' Topic.
  logical_camera_4_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_4", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 4));

    // Subscribe to the '/ariac/logical_camera_5' Topic.
  logical_camera_5_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_5", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 5));

    // Subscribe to the '/ariac/logical_camera_6' Topic.
  logical_camera_6_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_6", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 6));

    // Subscribe to the '/ariac/logical_camera_7' Topic.
  logical_camera_7_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_7", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 7));

    // Subscribe to the '/ariac/logical_camera_8' Topic.
  logical_camera_8_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_8", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 8));

    // Subscribe to the '/ariac/logical_camera_9' Topic.
  logical_camera_9_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_9", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 9));

    // Subscribe to the '/ariac/logical_camera_10' Topic.
  logical_camera_10_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_10", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 10));

    // Subscribe to the '/ariac/logical_camera_11' Topic.
  logical_camera_11_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_11", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 11));

    // Subscribe to the '/ariac/logical_camera_12' Topic.
  logical_camera_12_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_12", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 12));

      // Subscribe to the '/ariac/logical_camera_13' Topic.
  logical_camera_13_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_13", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 13));

  logical_camera_14_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_14", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 14));

  logical_camera_15_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_15", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 15));

  logical_camera_16_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_16", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 16));

  logical_camera_17_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_17", 1,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 17));

  quality_sensor_subscriber_1 = node_.subscribe(
       "/ariac/quality_control_sensor_1", 1, &sensor_read::quality_control_sensor_callback1,this
       );
  
  quality_sensor_subscriber_2 = node_.subscribe(
       "/ariac/quality_control_sensor_2", 1, &sensor_read::quality_control_sensor_callback2,this
       );

  
}

////////////////////////
void sensor_read::quality_control_sensor_callback1(const nist_gear::LogicalCameraImage::ConstPtr &msg) {
  if(msg->models.size() > 0) {
    is_faulty1 = true;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration timeout(5.0);

    geometry_msgs::TransformStamped transformStamped1;
    geometry_msgs::PoseStamped pose_in_world, pose_in_reference;

    pose_in_reference.pose = msg->models[0].pose;
    transformStamped1 = tfBuffer.lookupTransform("world", "quality_control_sensor_1_frame", ros::Time(0), timeout);
    tf2::doTransform(pose_in_reference, pose_in_world, transformStamped1);

    faulty_pose1.pose = pose_in_world.pose;
  }
  else{
    is_faulty1 = false;
  }
}

////////////////////////
void sensor_read::quality_control_sensor_callback2(const nist_gear::LogicalCameraImage::ConstPtr &msg){
  if(msg->models.size() > 0) {
    is_faulty2 = true;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration timeout(5.0);

    geometry_msgs::TransformStamped transformStamped2;
    geometry_msgs::PoseStamped pose_in_world, pose_in_reference;

    pose_in_reference.pose = msg->models[0].pose;
    transformStamped2 = tfBuffer.lookupTransform("world", "quality_control_sensor_2_frame", ros::Time(0), timeout);
    tf2::doTransform(pose_in_reference, pose_in_world, transformStamped2);

    faulty_pose2.pose = pose_in_world.pose;
  }
  else{
    is_faulty2 = false;
  }
}

////////////////////////
void sensor_read::logical_camera_callback(
      const nist_gear::LogicalCameraImage::ConstPtr &msg, int id)
  {
    //ROS_INFO("SUCCESS");
    if(msg->models.size() > 0) {
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);

      ros::Rate rate(10);
      ros::Duration timeout(5.0);
      
      geometry_msgs::TransformStamped transformStamped;
      geometry_msgs::PoseStamped pose_in_world, pose_in_reference;

      std::vector<part> new_list;

      try {
          transformStamped = tfBuffer.lookupTransform("world", "logical_camera_" + std::to_string(id) + "_frame",
                                ros::Time(0), timeout);
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      if(logicam_update[id]==0) {
        for(int i=0; i<msg->models.size(); i++) {
          pose_in_reference.pose = msg->models[i].pose;
          tf2::doTransform(pose_in_reference, pose_in_world, transformStamped);

          part_info[id][i].type = msg->models[i].type;
          part_info[id][i].pose = pose_in_world.pose;  
          part_info[id][i].is_picked = false;
        }
        logicam_update[id]=1;
      }

    }
  }

void sensor_read::reset_faulty(){
  is_faulty1 = false;
  is_faulty2 = false;
}

bool sensor_read::get_is_faulty(std::string agv) {
    if(agv == "agv2")
        return is_faulty1;
    return is_faulty2;
}

part sensor_read::get_faulty_pose(std::string agv) {
    if(agv == "agv2")
        return faulty_pose1;
    return faulty_pose2;
}

std::array<std::array<part, 36>, 18> sensor_read::get_part_info(){
  return part_info;
}

std::string sensor_read::find_part(std::string part_type){
  ROS_INFO_STREAM("Sensors finding part");
  ROS_INFO_STREAM("Part to find" << part_type);
  
  for(int i = 0; i<10; i++){
    // ROS_INFO_STREAM("FOUND1");
    for(int j=0; j<part_info[i].size(); j++){
      //ROS_INFO_STREAM("FOUND2");
      //ROS_INFO_STREAM(part_info[i][j].type);
      // ros::Duration(1.0).sleep();
      if (part_info[i][j].type == part_type){
        if (part_info[i][j].is_picked == false){
          ROS_INFO_STREAM("FOUND");
          ROS_INFO_STREAM(part_info[i][j].is_picked);
          // ROS_INFO_STREAM(this->part_info[i][j].type);
          // ROS_INFO_STREAM(part_type);
          int camera_coverage = camera_locations[i].size();
          if(camera_coverage==4){
            ROS_INFO_STREAM("coverage");
            for(auto bin: camera_locations[i]) {
              ROS_INFO_STREAM(bin);
              ROS_INFO_STREAM("comparing to find part, loop 1");
              std::vector<float> bin_pose = bin_locations[bin];
              if (part_info[i][j].pose.position.x >= bin_pose[0] 
                && part_info[i][j].pose.position.y <= bin_pose[1]
                && part_info[i][j].pose.position.x <= bin_pose[2]
                && part_info[i][j].pose.position.y >= bin_pose[3]){
                  logi_cam_id = i;
                  //ROS_INFO_STREAM(this->part_info[i][j].type);
                  ROS_INFO_STREAM("found the bin");
                  found_part = part_info[i][j];
                  ROS_INFO_STREAM("seg check-1");
                  // part empty_part;
                  part_info[i][j].is_picked = true;
                  ROS_INFO_STREAM("seg check-2");
                  // ROS_INFO_STREAM(this->part_info[i][j].type);
                  return bin;
                }
            }

          } 
          else {
            for(auto bin: camera_locations[i]) {
              ROS_INFO_STREAM(bin);
              ROS_INFO_STREAM("comparing to find part, loop 2");
              std::vector<float> shelf_pose = bin_locations[bin];
              ROS_INFO_STREAM("checking x> " << shelf_pose[0] << "checking x< " << shelf_pose[2] <<"checking y< " << shelf_pose[1] << "checking y> " << shelf_pose[3]);
              ROS_INFO_STREAM("checking x " << part_info[i][j].pose.position.x  <<"checking y< " << part_info[i][j].pose.position.y);
              if (part_info[i][j].pose.position.x <= shelf_pose[0] 
                && part_info[i][j].pose.position.y <= shelf_pose[1]
                && part_info[i][j].pose.position.x >= shelf_pose[2]
                && part_info[i][j].pose.position.y >= shelf_pose[3]){
                  found_part = part_info[i][j];
                  ROS_INFO_STREAM("found the bin");
                  part_info[i][j].is_picked = true;
                  return bin;
                }
            }
          }
        }
      }
    }
  }
}

int sensor_read::get_logi_cam(){
  return logi_cam_id;
}