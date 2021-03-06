#ifndef GANTRYCONTROL_H
#define GANTRYCONTROL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "utils.h"

/** \file gantry_control.h
 * Header file for gantry_control.cpp
 */
class GantryControl {

  public:
    GantryControl(ros::NodeHandle & node);

    void init();

//    bool moveGantry(std::string waypoints);

//    bool pickPart(part part, std::string arm_name);
    bool pickPart(part part);
    void placePart(part part, std::string agv);
    bool pickPartConveyor(part part);
    bool part_dropped = false;
    
    /// Send command message to robot controller
    bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
    void goToPresetLocation(PresetLocation location);

    void activateGripper(std::string gripper_id);
    void deactivateGripper(std::string gripper_id);
    nist_gear::VacuumGripperState getGripperState(std::string arm_name);
    geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv);
    void flipPart(anytype for_gantry);
    std::vector<std::string> determineGaps();
    double shelfPosition(std::string shelf_id);

    // Variables for gaps
    std::vector<std::string> gap;
    std::vector<double> gapThreshold = {6.299163, 6.299173};
    std::vector<std::string> row_name = {"left_row_","middle_row_", "right_row_"};
    std::vector<double> preset_x_gap;

    //--preset locations;
    start start_;
    bin bin1_;
    bin bin2_;
    bin bin3_;
    bin bin4_;
    bin bin5_;
    bin bin6_;
    bin bin7_;
    bin bin8_;
    bin bin9_;
    bin bin10_;
    bin bin11_;
    bin bin12_;
    bin bin13_;
    bin bin14_;
    bin bin15_;
    bin bin16_;
    agv agv1_;
    agv agv2_;
    agv agv1_faulty;
    agv agv2_faulty;
    agv right_arm_agv1_;
    agv right_arm_agv2_;
    shelf shelf1a_;
    shelf shelf1b_;
    shelf shelf1c_;
    shelf shelf1d_;
    shelf shelf1e_;
    shelf shelf1f_;
    shelf shelf1g_;
    shelf shelf2a_;
    shelf shelf2b_;
    shelf shelf2c_;
    shelf shelf2d_;
    shelf shelf2e_;
    shelf shelf2f_;
    shelf shelf2g_;
    shelf shelf5a_;
    shelf shelf5b_;
    shelf shelf5c_;
    shelf shelf5d_;
    shelf shelf5e_;
    shelf shelf5f_;
    shelf shelf5g_;
    shelf shelf58a_;
    shelf shelf58b_;
    shelf shelf58c_;
    shelf shelf58d_;
    shelf shelf58e_;
    shelf shelf58f_;
    shelf shelf58g_;
    shelf shelf811a_;
    shelf shelf811b_;
    shelf shelf811c_;
    shelf shelf811d_;
    shelf shelf811e_;
    shelf shelf811f_;
    shelf shelf811g_;
    shelf shelf11a_;
    shelf shelf11b_;
    shelf shelf11c_;
    shelf shelf11d_;
    shelf shelf11e_;
    conveyor conveyor_;
    conveyor conveyor_bin1_;
    conveyor clear_bin1_;
    conveyor clear_bin2_;
    conveyor clear_bin3_;
    conveyor clear_bin1_right_;
    conveyor clear_bin2_right_;
    conveyor clear_bin3_right_;
    anytype flippart_;
    bool flip_trig;
    std::vector<conveyor> clear_bins;
    std::vector<conveyor> clear_bins_right; 




  private:
    std::vector<double> joint_group_positions_;
    ros::NodeHandle node_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Options full_robot_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface full_robot_group_;
    moveit::planning_interface::MoveGroupInterface left_arm_group_;
    moveit::planning_interface::MoveGroupInterface right_arm_group_;
    moveit::planning_interface::MoveGroupInterface left_ee_link_group_;
    moveit::planning_interface::MoveGroupInterface right_ee_link_group_;

    double left_ee_roll_;
    double left_ee_pitch_;
    double left_ee_yaw_;
    std::array<float,4> left_ee_quaternion_;

    sensor_msgs::JointState current_joint_states_;


    nist_gear::VacuumGripperState current_left_gripper_state_;
    nist_gear::VacuumGripperState current_right_gripper_state_;

    control_msgs::JointTrajectoryControllerState current_gantry_controller_state_;
    control_msgs::JointTrajectoryControllerState current_left_arm_controller_state_;
    control_msgs::JointTrajectoryControllerState current_right_arm_controller_state_;

    ros::Publisher gantry_joint_trajectory_publisher_;
    ros::Publisher left_arm_joint_trajectory_publisher_;
    ros::Publisher right_arm_joint_trajectory_publisher_;

    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber left_gripper_state_subscriber_;
    ros::Subscriber right_gripper_state_subscriber_;
    ros::Subscriber gantry_controller_state_subscriber_;
    ros::Subscriber left_arm_controller_state_subscriber_;
    ros::Subscriber right_arm_controller_state_subscriber_;

    ros::ServiceClient left_gripper_control_client;
    ros::ServiceClient right_gripper_control_client;

    // ---------- Callbacks ----------
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg);
    void left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);

};

#endif
