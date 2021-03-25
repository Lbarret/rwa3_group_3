#pragma once


#include <list>
#include <map>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nist_gear/LogicalCameraImage.h>

#include "Parts.h"

class sensor_read {
public:
    sensor_read();
    ~sensor_read();
    void Logical_Camera_1Callback(const nist_gear::LogicalCameraImage::ConstPtr&);
    void Logical_Camera_2Callback(const nist_gear::LogicalCameraImage::ConstPtr &);
    void Logical_Camera_3Callback(const nist_gear::LogicalCameraImage::ConstPtr &);


    geometry_msgs::Pose Part_Poses(const std::string& src_frame,
                                    const std::string& target_frame);
    std::map<std::string, std::vector<std::string>> get_product_frame_list(){
        return product_frame_list_;
    }
    
    void ProductFrames(int);

private:
    ros::NodeHandle sensor_nh_;
    ros::Subscriber camera_1_subscriber_;
    ros::Subscriber camera_2_subscriber_;
    ros::Subscriber camera_3_subscriber_;


    tf::TransformListener camera_tf_listener_;
    tf::StampedTransform camera_tf_transform_;

    nist_gear::LogicalCameraImage current_parts_1_;
    nist_gear::LogicalCameraImage current_parts_2_;
    nist_gear::LogicalCameraImage current_parts_3_;
    std::map<std::string, std::vector<geometry_msgs::Pose>> part_list_;
    std::vector<Parts> camera1_part_list,camera2_part_list,camera3_part_list;

     
    std::map<std::string, std::vector<std::string>> product_frame_list_;

    bool init_, cam_1_, cam_2_,cam_3_;
    int camera1_frame_counter_, camera2_frame_counter_, camera3_frame_counter_;
};