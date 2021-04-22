#ifndef SENSORREAD_H
#define SENSORREAD_H

#include <algorithm>
#include <vector>
#include <array>

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
/** \file sensor_read.h
 * Header file for sensor_read.cpp
 */

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
    void quality_control_sensor_callback1(const nist_gear::LogicalCameraImage::ConstPtr &msg);
    void quality_control_sensor_callback2(const nist_gear::LogicalCameraImage::ConstPtr &msg);
    void breakbeam_sensor_callback(const nist_gear::Proximity::ConstPtr &msg);
    void breakbeam_sensor2_callback(const nist_gear::Proximity::ConstPtr &msg);
    void breakbeam_sensor3_callback(const nist_gear::Proximity::ConstPtr &msg);
    void breakbeam_sensor4_callback(const nist_gear::Proximity::ConstPtr &msg);
    void breakbeam_sensor5_callback(const nist_gear::Proximity::ConstPtr &msg);
    void breakbeam_sensor6_callback(const nist_gear::Proximity::ConstPtr &msg);
    int check_human_aisle_one();
    int check_human_aisle_two();
    int check_human_aisle_three();
    int check_human_aisle_four();
    int check_human_hole_one();
    int human_aisle_one = 0;
    int human_aisle_two = 0;
    int human_aisle_three = 0;
    int human_aisle_four = 0;
    int human_hole_one = 0;

    std::string find_part(std::string part_type, int agv);
    // std::vector<std::vector<part>> get_part_info();
    std::array<std::array<part, 36>, 18> get_part_info();
    void reset_faulty();
    bool get_is_faulty(std::string agv);
    part get_faulty_pose(std::string agv);

    int get_logi_cam();
    std::array<std::array<part, 36>, 18> part_info;
    part found_part;
    void reset_logicam_update(){
        std::fill(std::begin(logicam_update), std::end(logicam_update), 0);
    }



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
    ros::Subscriber quality_sensor_subscriber_1;
    ros::Subscriber quality_sensor_subscriber_2;
    ros::Subscriber breakbeam_subscriber;
    ros::Subscriber breakbeam2_subscriber;
    ros::Subscriber breakbeam3_subscriber;
    ros::Subscriber breakbeam4_subscriber;
    ros::Subscriber breakbeam5_subscriber;
    ros::Subscriber breakbeam6_subscriber;



    std::string part_location;
    std::array<int, 18> logicam_update = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<std::vector<std::string>> camera_info;
    int logi_cam_id;
    // std::vector<std::vector<part>> parts_info;
    /**
     * The different parts of the world which fall under which camera
     */
    std::map<int, std::vector<std::string>> camera_locations = { 
        {0, {"bin3_", "bin4_", "bin7_", "bin8_"}},
        {1, {"bin11_", "bin12_","bin15_", "bin16_"}},
        {2, {"bin1_", "bin2_", "bin5_", "bin6_"}},
        {3, {"bin9_", "bin10_", "bin13_", "bin14_"}},
        {4, {"shelf5a_"}},
        {5, {"shelf5b_"}},
        {8, {"shelf8a_"}},
        {9, {"shelf8b_"}},
        {6, {"shelf11a_"}},
        {7, {"shelf11b_"}},
        {10, {"beltm_"}},
        {11, {"beltf_"}},
        {12, {"agv2_"}},
        {13, {"agv1_"}},
        {14, {"shelf1a_"}},
        {15, {"shelf1b_"}},
        {16, {"shelf2a_"}},
        {17, {"shelf2b_"}}
    };
    bool is_faulty1, is_faulty2;
    part faulty_pose1, faulty_pose2;
};

#endif
