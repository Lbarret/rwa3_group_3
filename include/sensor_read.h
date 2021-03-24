#ifndef SENSORREAD_H
#define SENSORREAD_H

#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3
#include <boost/bind.hpp>
#include "utils.h"


/**
 * @brief Competition class
 * 
 */
class sensor_read
{
public:
    /**
     * @brief Construct a new Competition object
     * 
     * @param node Node handle
     */
    explicit sensor_read(ros::NodeHandle & node);
    /**
     * @brief Initialize components of the class (suscribers, publishers, etc)
     * 
     */
    void init();
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int id);
    std::string find_part(std::string part);


private:
    ros::NodeHandle node_;/*!< node handle for this class */
    
    ros::Subscriber logical_camera_0_subscriber;/*!< subscriber to the topic /logical_camera_0 */
    ros::Subscriber logical_camera_1_subscriber;/*!< subscriber to the topic /logical_camera_1 */
    ros::Subscriber logical_camera_2_subscriber;/*!< subscriber to the topic /logical_camera_2 */
    ros::Subscriber logical_camera_3_subscriber;/*!< subscriber to the topic /logical_camera_3 */
    ros::Subscriber logical_camera_4_subscriber;/*!< subscriber to the topic /logical_camera_4 */
    ros::Subscriber logical_camera_5_subscriber;/*!< subscriber to the topic /logical_camera_5 */
    ros::Subscriber logical_camera_6_subscriber;/*!< subscriber to the topic /logical_camera_6 */
    ros::Subscriber logical_camera_7_subscriber;/*!< subscriber to the topic /logical_camera_7 */
    ros::Subscriber logical_camera_8_subscriber;/*!< subscriber to the topic /logical_camera_8 */
    ros::Subscriber logical_camera_9_subscriber;/*!< subscriber to the topic /logical_camera_9 */
    ros::Subscriber logical_camera_10_subscriber;/*!< subscriber to the topic /logical_camera_10 */
    ros::Subscriber logical_camera_11_subscriber;/*!< subscriber to the topic /logical_camera_11 */
    ros::Subscriber logical_camera_12_subscriber;/*!< subscriber to the topic /logical_camera_12 */
    ros::Subscriber logical_camera_13_subscriber;/*!< subscriber to the topic /logical_camera_13 */
    ros::Subscriber logical_camera_14_subscriber;/*!< subscriber to the topic /logical_camera_14 */
    ros::Subscriber logical_camera_15_subscriber;/*!< subscriber to the topic /logical_camera_15 */
    ros::Subscriber logical_camera_16_subscriber;/*!< subscriber to the topic /logical_camera_16 */
    ros::Subscriber logical_camera_17_subscriber;/*!< subscriber to the topic /logical_camera_17 */
    
    std::string part_location;
    std::vector<std::vector<std::string>> camera_info;
    std::vector<std::vector<geometry_msgs::PoseStamped>> part_pose;
    std::vector<std::vector<std::string>> camera_locations;
};

#endif
