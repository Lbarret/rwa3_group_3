#include "sensor_read.h"
#include "utils.h"

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
      "/ariac/logical_camera_0", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 0));

  // Subscribe to the '/ariac/logical_camera_1' Topic.
  logical_camera_1_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_1", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 1));

    // Subscribe to the '/ariac/logical_camera_2' Topic.
  logical_camera_2_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_2", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 2));

    // Subscribe to the '/ariac/logical_camera_3' Topic.
  logical_camera_3_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_3", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 3));

    // Subscribe to the '/ariac/logical_camera_4' Topic.
  logical_camera_4_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_4", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 4));

    // Subscribe to the '/ariac/logical_camera_5' Topic.
  logical_camera_5_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_5", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 5));

    // Subscribe to the '/ariac/logical_camera_6' Topic.
  logical_camera_6_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_6", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 6));

    // Subscribe to the '/ariac/logical_camera_7' Topic.
  logical_camera_7_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_7", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 7));

    // Subscribe to the '/ariac/logical_camera_8' Topic.
  logical_camera_8_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_8", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 8));

    // Subscribe to the '/ariac/logical_camera_9' Topic.
  logical_camera_9_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_9", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 9));

    // Subscribe to the '/ariac/logical_camera_10' Topic.
  logical_camera_10_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_10", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 10));

    // Subscribe to the '/ariac/logical_camera_11' Topic.
  logical_camera_11_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_11", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 11));

    // Subscribe to the '/ariac/logical_camera_12' Topic.
  logical_camera_12_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_12", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 12));

      // Subscribe to the '/ariac/logical_camera_13' Topic.
  logical_camera_13_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_13", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 13));

  logical_camera_14_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_14", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 13));

  logical_camera_15_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_15", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 15));

  logical_camera_16_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_16", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 16));

  logical_camera_17_subscriber = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_17", 1000,
      boost::bind(&sensor_read::logical_camera_callback, this, _1, 17));

  
}

////////////////////////
void sensor_read::logical_camera_callback(
      const nist_gear::LogicalCameraImage::ConstPtr &msg, int id)
  {

    if(msg->models.size() > 0) {
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);

      ros::Rate rate(10);
      ros::Duration timeout(5.0);
      
      geometry_msgs::TransformStamped transformStamped;
      geometry_msgs::PoseStamped pose_in_world, pose_in_reference;

      try {
          transformStamped = tfBuffer.lookupTransform("world", "logical_camera_" + std::to_string(id) + "_frame",
                                ros::Time(0), timeout);
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      for(int i=0; i<msg->models.size(); i++) {
        pose_in_reference.pose = msg->models[i].pose;
        tf2::doTransform(pose_in_reference, pose_in_world, transformStamped);

        tf2::Quaternion q(
          pose_in_world.pose.orientation.x,
          pose_in_world.pose.orientation.y,
          pose_in_world.pose.orientation.z,
          pose_in_world.pose.orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        camera_info[id][i] = msg->models[i].type.c_str();
        part_pose[id][i].pose = pose_in_world.pose;
        ROS_INFO_STREAM(camera_info[id][i]);

      }
      
    }
  }

  std::string sensor_read::find_part(std::string part){
    ROS_INFO_STREAM("finding part");
    camera_locations[0] = {"bin3_", "bin4_", "bin7_", "bin8_"};
    camera_locations[1] = {"bin11_", "bin12_","bin15_", "bin16_"};
    camera_locations[2] = { "bin1_", "bin2_", "bin5_", "bin6_"};
    camera_locations[3] = { "bin9_", "bin10_", "bin13_", "bin14_"};
    for(int i = 0; i<camera_info.size(); i++){
      for(int j=0; j<camera_info[i].size(); j++){
        if (camera_info[i][j] == part){
          for(int k=0; k<3; k+2) {
            std::vector<float> bin_pose = bin_locations[camera_locations[i][k]];
            if (part_pose[i][j].pose.position.x >= bin_pose[0] 
            && part_pose[i][j].pose.position.y <= bin_pose[1]
            && part_pose[i][j].pose.position.x <= bin_pose[2]
            && part_pose[i][j].pose.position.y >= bin_pose[3])
              return camera_locations[i][k];
          }
        }
      }
    }
          
  }