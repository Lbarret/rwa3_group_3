#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
/** \file gantry_control.cpp
 * Code for rwa4 which details the different moves the robot should make throughout filling an order
 */

/**
 * @brief Construct a new Gantry Control:: Gantry Control object
 * 
 * @param node 
 */
GantryControl::GantryControl(ros::NodeHandle &node) : node_("/ariac/gantry"),
                                                      planning_group_("/ariac/gantry/robot_description"),
                                                      full_robot_options_("Full_Robot", planning_group_, node_),
                                                      left_arm_options_("Left_Arm", planning_group_, node_),
                                                      right_arm_options_("Right_Arm", planning_group_, node_),
                                                      left_ee_link_options_("Left_Endeffector", planning_group_, node_),
                                                      right_ee_link_options_("Right_Endeffector", planning_group_, node_),
                                                      full_robot_group_(full_robot_options_),
                                                      left_arm_group_(left_arm_options_),
                                                      right_arm_group_(right_arm_options_),
                                                      left_ee_link_group_(left_ee_link_options_),
                                                      right_ee_link_group_(right_ee_link_options_)
{
    ROS_INFO_STREAM("[GantryControl::GantryControl] constructor called... ");
}

////////////////////////////
void GantryControl::init()
{
    ROS_INFO_STREAM("[GantryControl::init] init... ");
    double time_called = ros::Time::now().toSec();

    ROS_INFO_NAMED("init", "Planning frame: %s", left_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", full_robot_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_ee_link_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", left_ee_link_group_.getPlanningFrame().c_str());

    ROS_INFO_NAMED("init", "End effector link: %s", left_arm_group_.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("init", "End effector link: %s", right_arm_group_.getEndEffectorLink().c_str());

    left_arm_group_.setPoseReferenceFrame("world");

    // preset locations

    // joint positions to go to start location
    start_.gantry = {0, 0, 0};
    start_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin1
    bin1_.gantry = {2, -1.3, 0.};
    bin1_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin1_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin2
    bin2_.gantry = {3, -1.3, 0.};
    bin2_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin2_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin3
    bin3_.gantry = {4.0, -1.3, 0.};
    bin3_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin3_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin4
    bin4_.gantry = {5.0, -1.3, 0.};
    bin4_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin4_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin5
    bin5_.gantry = {2, -2.16, 0.};
    bin5_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin5_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin6
    bin6_.gantry = {3, -2.16, 0.};
    bin6_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin6_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin7
    bin7_.gantry = {4.0, -2.16, 0.};
    bin7_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin7_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin8
    bin8_.gantry = {5.0, -2.16, 0.};
    bin8_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin8_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin9
    bin9_.gantry = {2, 1.3, 0.};
    bin9_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin9_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin10
    bin10_.gantry = {3, 1.3, 0.};
    bin10_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin10_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin11
    bin11_.gantry = {4.0, 1.3, 0.};
    bin11_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin11_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin12
    bin12_.gantry = {5.0, 1.3, 0.};
    bin12_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin12_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin13
    bin13_.gantry = {2.15, 2.1, 0.};
    bin13_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin13_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin14
    bin14_.gantry = {3, 2.16, 0.};
    bin14_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin14_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin15
    bin15_.gantry = {4.0, 2.16, 0.};
    bin15_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin15_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin16
    bin16_.gantry = {5.0, 2.16, 0.};
    bin16_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin16_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to agv1
    agv1_.gantry = {0.6, -6.9, PI};
    agv1_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv1_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to agv2
    agv2_.gantry = {0.6, 6.9, PI};
    agv2_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv2_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to discard to faulty part
    agv1_faulty.gantry = {0, -2.0, PI};
    agv1_faulty.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_faulty.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // joint positions to discard to faulty part
    agv2_faulty.gantry = {0, 2.0, PI};
    agv2_faulty.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_faulty.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // joint positions to go to shelf 5
    shelf5a_.gantry = {0.5, -5.8, 0}; // go to aisle
    shelf5a_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
    shelf5a_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf5b_.gantry = {-14.5, -5.8, 0}; // go to shelf
    shelf5b_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
    shelf5b_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf5c_.gantry = {-14.5, -5.8, 0};// move close to self
    shelf5c_.left_arm = {-1.7, -PI/4, 1.5, -0.5, -0.1, 0};
    shelf5c_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf5d_.gantry = {-14.5, -4.25, 0};// move closer for a
    shelf5d_.left_arm = {-1.7, -PI/4, 1.5, -0.5, -0.1, 0};
    shelf5d_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf5e_.gantry = {-15.5, -5.5, 0};// move closer for b
    shelf5e_.left_arm = {-1.7, -PI/4, 1.6, -0.63, -0.1, 0};
    shelf5e_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf5f_.gantry = {-14, -5.8, 0}; // Go to near shelf with part
    shelf5f_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
    shelf5f_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf5g_.gantry = {-14, -5.8, 0}; // Go to near shelf with part
    shelf5g_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
    shelf5g_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // joint positions to go to between shelves 5 and 8
    shelf58a_.gantry = {0.5, -1.9, 0};// go to aisle
    shelf58a_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
    shelf58a_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf58b_.gantry = {-14, -1.9, 0};// go to shelf
    shelf58b_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
    shelf58b_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf58c_.gantry = {-13, -1.9, 0}; //move arm
    shelf58c_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
    shelf58c_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf58d_.gantry = {-14, -1.4, 0};// move closer for a
    shelf58d_.left_arm = {-1.7, -PI/4, 1.5, -0.5, -0.1, 0};
    shelf58d_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf58e_.gantry = {-15, -1.4, 0};// move closer for b
    shelf58e_.left_arm = {-1.7, -PI/4, 1.5, -0.63, -0.1, 0};
    shelf58e_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    if(preset_x_gap[0] < preset_x_gap[1]) {
        shelf58f_.gantry = {preset_x_gap[0] , -1.9, 0}; // go to space
        shelf58f_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
        shelf58f_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

        shelf58g_.gantry = {preset_x_gap[0], -3.5, 0}; // go in space
        shelf58g_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
        shelf58g_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    } else {
        shelf58f_.gantry = {preset_x_gap[1] , -1.9, 0}; // go to space
        shelf58f_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
        shelf58f_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

        shelf58g_.gantry = {preset_x_gap[1], -3.5, 0}; // go in space
        shelf58g_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
        shelf58g_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    }
    
    // joint positions to go between shelves 8 and 11
    shelf811a_.gantry = {0.5, 1.5, 0};// go to aisle
    shelf811a_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
    shelf811a_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    shelf811b_.gantry = {-14.5, 1.5, 0};// go to shelf
    shelf811b_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf811b_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf811c_.gantry = {-14.5, 1.5, 0};//move arm
    shelf811c_.left_arm = {-1.7, -PI/4, 1.5, -0.5, -0.1, 0};
    shelf811c_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf811d_.gantry = {-14.5, 1.75, 0};// move closer for a
    shelf811d_.left_arm = {-1.7, -PI/4, 1.5, -0.5, -0.1, 0};
    shelf811d_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf811e_.gantry = {-15.5, 0.5, 0};// move closer for b
    shelf811e_.left_arm = {-1.7, -PI/4, 1.6, -0.63, -0.1, 0};
    shelf811e_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    if(preset_x_gap[1] < preset_x_gap[2]) {
        shelf811f_.gantry = {preset_x_gap[1], -1.9, 0}; // go to space
        shelf811f_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
        shelf811f_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

        shelf811g_.gantry = {preset_x_gap[1], -3.5, 0}; // go in space
        shelf811g_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
        shelf811g_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    } else {
        shelf811f_.gantry = {preset_x_gap[2], -1.9, 0}; // go to space
        shelf811f_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
        shelf811f_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

        shelf811g_.gantry = {preset_x_gap[2], -3.5, 0}; // go in space
        shelf811g_.left_arm = {-1.7, -PI/4, 2, -0.5, -0.1, 0};
        shelf811g_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    }
    


    // joint positions to go to shelf 11
    shelf11a_.gantry = {0.5, 4.5, 0};// go to aisle
    shelf11a_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf11a_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11b_.gantry = {-14.5, 4.5, 0};// go to shelf
    shelf11b_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf11b_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11c_.gantry = {-14.5, 4.5, 0};//move arm
    shelf11c_.left_arm = {-1.7, -PI/4, 1.5, -0.5, -0.1, 0};
    shelf11c_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11d_.gantry = {-14.5, 3.35, 0};// move closer for a
    shelf11d_.left_arm = {-1.7, -PI/4, 1.5, -0.5, -0.1, 0};
    shelf11d_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11e_.gantry = {-15.5, 3.5, 0};// move closer for b
    shelf11e_.left_arm = {-1.7, -PI/4, 1.6, -0.63, -0.1, 0};
    shelf11e_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    conveyor_.gantry = {.15, -2, PI / 2};//
    conveyor_.left_arm = {0.0, -PI / 4, PI / 2, -3*PI / 4, PI / 2, 0};
    conveyor_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    conveyor_bin1_.gantry = {1.8, -1.3, 0.};
    conveyor_bin1_.left_arm = {0.0, -PI / 4, PI / 2, -3*PI / 4, PI / 2, 0};
    conveyor_bin1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    right_arm_agv1_.gantry = {0.6, -6.9, 0};
    right_arm_agv1_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    right_arm_agv1_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    right_arm_agv2_.gantry = {0.6, 6.9, 0};
    right_arm_agv2_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    right_arm_agv2_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};


    //--Raw pointers are frequently used to refer to the planning group for improved performance.
    //--To start, we will create a pointer that references the current robot’s state.
    const moveit::core::JointModelGroup *joint_model_group =
        full_robot_group_.getCurrentState()->getJointModelGroup("Full_Robot");

    //--Let’s set a joint space goal and move towards it.
    moveit::core::RobotStatePtr current_state = full_robot_group_.getCurrentState();

    //--Next get the current set of joint values for the group.
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

    gantry_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    left_arm_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/left_arm_controller/command", 10);

    right_arm_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/right_arm_controller/command", 10);

    joint_states_subscriber_ = node_.subscribe(
        "/ariac/gantry/joint_states", 10, &GantryControl::joint_states_callback, this);

    left_gripper_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/left_arm/gripper/state", 10, &GantryControl::left_gripper_state_callback, this);

    right_gripper_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/right_arm/gripper/state", 10, &GantryControl::right_gripper_state_callback, this);

    gantry_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/gantry_controller/state", 10, &GantryControl::gantry_controller_state_callback, this);

    left_arm_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/left_arm_controller/state", 10, &GantryControl::left_arm_controller_state_callback, this);

    right_arm_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/right_arm_controller/state", 10, &GantryControl::right_arm_controller_state_callback, this);

    while ((current_gantry_controller_state_.joint_names.size() == 0) || 
    (current_left_arm_controller_state_.joint_names.size() == 0) || 
    (current_right_arm_controller_state_.joint_names.size() == 0))
    {
        ROS_WARN("[GantryControl::init] Waiting for first controller_state callbacks...");
        ros::Duration(0.1).sleep();
    }

    left_gripper_control_client =
        node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/left_arm/gripper/control");
    left_gripper_control_client.waitForExistence();

    right_gripper_control_client =
        node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/right_arm/gripper/control");
    right_gripper_control_client.waitForExistence();

    // Move robot to init position
    ROS_INFO("[GantryControl::init] Init position ready)...");
}


////////////////////////////
geometry_msgs::Pose GantryControl::getTargetWorldPose(geometry_msgs::Pose target,
                                                      std::string agv)
{
    
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    std::string kit_tray;
    if (agv.compare("agv1") == 0)
        kit_tray = "kit_tray_1";
    else
        kit_tray = "kit_tray_2";
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = kit_tray;
    transformStamped.child_frame_id = "target_frame";
    transformStamped.transform.translation.x = target.position.x;
    transformStamped.transform.translation.y = target.position.y;
    transformStamped.transform.translation.z = target.position.z;
    transformStamped.transform.rotation.x = target.orientation.x;
    transformStamped.transform.rotation.y = target.orientation.y;
    transformStamped.transform.rotation.z = target.orientation.z;
    transformStamped.transform.rotation.w = target.orientation.w;

    
    for (int i{0}; i < 15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // ros::Rate rate(10);
    ros::Duration timeout(5.0);

    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_target_tf;
    for (int i = 0; i < 10; i++)
    {
        try
        {
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                       ros::Time(0), timeout);
            // ROS_WARN_STREAM("target in world frame: " << world_target_tf.transform.rotation.x <<" "<<world_target_tf.transform.rotation.y<<" "<<world_target_tf.transform.rotation.z<<" "<<world_target_tf.transform.rotation.w);

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    for (int i = 0; i < 10; i++)
    {
        try
        {
            ee_target_tf = tfBuffer.lookupTransform("world", "left_ee_link",
                                                    ros::Time(0), timeout);
            // ROS_WARN_STREAM("left_ee_link in target frame: " << ee_target_tf.transform.rotation.x <<" "<<ee_target_tf.transform.rotation.y<<" "<<ee_target_tf.transform.rotation.z<<" "<<ee_target_tf.transform.rotation.w);

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

/**
 *ee_target_tf and  world_target_tf are expressed in the same frame: "world"
 We want to find the relative rotation, q_r, to go from ee_target_tf to world_target_tf
 q_r = world_target_tf*ee_target_tf_inverse
 * 
 */
    tf2::Quaternion ee_target_tf_inverse(
        ee_target_tf.transform.rotation.x,
        ee_target_tf.transform.rotation.y,
        ee_target_tf.transform.rotation.z,
        -ee_target_tf.transform.rotation.w);

    tf2::Quaternion part_in_tray(world_target_tf.transform.rotation.x,
                                 world_target_tf.transform.rotation.y,
                                 world_target_tf.transform.rotation.z,
                                 world_target_tf.transform.rotation.w);

    tf2::Quaternion qr = part_in_tray * ee_target_tf_inverse;
    qr.normalize();

    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = ee_target_tf.transform.rotation.x;
    world_target.orientation.y = ee_target_tf.transform.rotation.y;
    world_target.orientation.z = ee_target_tf.transform.rotation.z;
    world_target.orientation.w = ee_target_tf.transform.rotation.w;

    // ros::Duration(10).sleep();
    return world_target;
}

////////////////////////////
bool GantryControl::pickPart(part part)
{
    //--Activate gripper
    auto state = getGripperState("left_arm");
    while(!state.enabled){
        activateGripper("left_arm");
        state = getGripperState("left_arm");
    }
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

    part.pose.position.z = part.pose.position.z + model_height[part.type] + GRIPPER_HEIGHT - EPSILON ; //added calibration factor
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;
    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);
    geometry_msgs::Pose abovePartPose = part.pose;
    abovePartPose.position.z = part.pose.position.z + .1;
    
    if (state.enabled)
    {
        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        left_arm_group_.setPoseTarget(abovePartPose);
        left_arm_group_.move();
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
            //--Move arm to previous position
            left_arm_group_.setPoseTarget(abovePartPose);
            left_arm_group_.move();
            return true;
        }
        else
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            int max_attempts{5};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt<max_attempts && !state.attached)
            {
                left_arm_group_.setPoseTarget(abovePartPose);
                left_arm_group_.move();
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();
                activateGripper("left_arm");
                state = getGripperState("left_arm");
                current_attempt++;
            }
            left_arm_group_.setPoseTarget(abovePartPose);
            left_arm_group_.move();
        }
    }
    else
    {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;
}

bool GantryControl::pickPartConveyor(part part)
{
    //--Activate gripper
    auto state = getGripperState("left_arm");
    while(!state.enabled){
        activateGripper("left_arm");
        state = getGripperState("left_arm");
    }
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

    part.pose.position.x = currentPose.position.x;
    part.pose.position.y = currentPose.position.y;
    part.pose.position.z = part.pose.position.z + model_height[part.type]/2 + GRIPPER_HEIGHT - EPSILON + 0.015; //added calibration factor
    
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;
    geometry_msgs::Pose above = part.pose;
    above.position.z+=.1;
    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);
    
    if (state.enabled)
    {
        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        ros::Duration(1.5).sleep();
        while (!state.attached){
            state = getGripperState("left_arm");
            if (state.attached)
            {
                ROS_INFO_STREAM("[Gripper] = object attached");
                //--Move arm to previous position
                left_arm_group_.setPoseTarget(above);
                left_arm_group_.move();
                ros::Duration(1.0).sleep();
                return true;
            }
            ros::Duration(.1).sleep();
        }
        
    }
    else
    {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;
}


////////////////////////////
void GantryControl::placePart(part part, std::string agv)
{
    auto target_pose_in_tray = getTargetWorldPose(part.pose, agv);

    ros::Duration(2.0).sleep();
    //--TODO: Consider agv1 too
    part_dropped = false;
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[part.type]);

    // tf2::Quaternion q_pitch(0, 0.7071068, 0, 0.7071068);
    // tf2::Quaternion q_pi(0, 0, 1, 0);
    // tf2::Quaternion q_init_part(part.pose.orientation.x,
    //                             part.pose.orientation.y,
    //                             part.pose.orientation.z,
    //                             part.pose.orientation.w);

    // tf2::Quaternion q_final_part(target_pose_in_tray.orientation.x,
    //                             target_pose_in_tray.orientation.y,
    //                             target_pose_in_tray.orientation.z,
    //                             target_pose_in_tray.orientation.w);

    // // tf2::Quaternion q_pi_by_2(0, 0, 0.7071068, 0.7071068);
    // tf2::Quaternion q_rslt = q_init_part.inverse()*q_final_part*q_pi*q_pitch;
    //     target_pose_in_tray.orientation.x = q_rslt.x();
    //     target_pose_in_tray.orientation.y = q_rslt.y();
    //     target_pose_in_tray.orientation.z = q_rslt.z();
    //     target_pose_in_tray.orientation.w = q_rslt.w();


    auto state_left = getGripperState("left_arm");
    auto state_right = getGripperState("right_arm");
    ROS_INFO_STREAM("left:" << state_left.attached << "right: " << state_right.attached);
    if (state_left.attached) {
        if (agv == "agv1")
            goToPresetLocation(agv1_);
        if (agv == "agv2")
            goToPresetLocation(agv2_);
        left_arm_group_.setPoseTarget(target_pose_in_tray);
        left_arm_group_.move();
        state_left = getGripperState("left_arm");
        if(!state_left.attached){
            part_dropped = true;
        }
        deactivateGripper("left_arm");
    } else if(state_right.attached) {
        if (agv == "agv1")
            goToPresetLocation(right_arm_agv1_);
        if (agv == "agv2")
            goToPresetLocation(right_arm_agv2_);
        right_arm_group_.setPoseTarget(target_pose_in_tray);
        right_arm_group_.move();
        state_right = getGripperState("right_arm");
        if(!state_right.attached){
            part_dropped = true;
        }
        deactivateGripper("right_arm");
    }
    
}

////////////////////////////
void GantryControl::goToPresetLocation(PresetLocation location)
{
    //--gantry
    joint_group_positions_.at(0) = location.gantry.at(0);
    joint_group_positions_.at(1) = location.gantry.at(1);
    joint_group_positions_.at(2) = location.gantry.at(2);
    //--left arm
    joint_group_positions_.at(3) = location.left_arm.at(0);
    joint_group_positions_.at(4) = location.left_arm.at(1);
    joint_group_positions_.at(5) = location.left_arm.at(2);
    joint_group_positions_.at(6) = location.left_arm.at(3);
    joint_group_positions_.at(7) = location.left_arm.at(4);
    joint_group_positions_.at(8) = location.left_arm.at(5);
    //--right arm
    joint_group_positions_.at(9) = location.right_arm.at(0);
    joint_group_positions_.at(10) = location.right_arm.at(1);
    joint_group_positions_.at(11) = location.right_arm.at(2);
    joint_group_positions_.at(12) = location.right_arm.at(3);
    joint_group_positions_.at(13) = location.right_arm.at(4);
    joint_group_positions_.at(14) = location.right_arm.at(5);

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
}

////////////////////////////
void GantryControl::activateGripper(std::string arm_name)
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;

    if (arm_name == "left_arm")
    {
        left_gripper_control_client.call(srv);
    }
    else
    {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][activateGripper] DEBUG: srv.response =" << srv.response);
}

////////////////////////////
void GantryControl::deactivateGripper(std::string arm_name)
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;

    if (arm_name == "left_arm")
    {
        left_gripper_control_client.call(srv);
    }
    else
    {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][deactivateGripper] DEBUG: srv.response =" << srv.response);
}

////////////////////////////
nist_gear::VacuumGripperState GantryControl::getGripperState(std::string arm_name)
{
    if (arm_name == "left_arm")
    {
        return current_left_gripper_state_;
    }
    else
    {
        return current_right_gripper_state_;
    }
}

////////////////////////////
void GantryControl::left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_left_gripper_state_ = *gripper_state_msg;
}

////////////////////////////
void GantryControl::right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_right_gripper_state_ = *gripper_state_msg;
}

////////////////////////////
void GantryControl::joint_states_callback(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
{
    if (joint_state_msg->position.size() == 0)
    {
        ROS_ERROR("[gantry_control][joint_states_callback] msg->position.size() == 0!");
    }
    current_joint_states_ = *joint_state_msg;
}

////////////////////////////
void GantryControl::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gantry controller states (throttled to 0.1 Hz):\n" << *msg);
    current_gantry_controller_state_ = *msg;
}

////////////////////////////
void GantryControl::left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Left arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_left_arm_controller_state_ = *msg;
}

////////////////////////////
void GantryControl::right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Right arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_right_arm_controller_state_ = *msg;
}

////////////////////////////
bool GantryControl::sendJointPosition(trajectory_msgs::JointTrajectory command_msg)
{
    // ROS_INFO_STREAM("[gantry_control][sendJointPosition] called.");

    if (command_msg.points.size() == 0)
    {
        ROS_WARN("[gantry_control][sendJointPosition] Trajectory is empty or NAN, returning.");
        return false;
    }
    else if ((command_msg.joint_names[0] == "small_long_joint") // command is for gantry
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0]))
    {

        gantry_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][sendJointPosition] gantry command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0, 4) == "left") // command is for left_arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0]))
    {

        left_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][sendJointPosition] left_arm command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0, 5) == "right") // command is for right arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0]))
    {

        right_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][sendJointPosition] right_arm command published!");
        return true;
    }
    else
    {
        return false;
    }
}
/**
 * If part needs to be flipped, the moves required to flip the part
 * @param for_gantry
 */
void GantryControl::flipPart(anytype for_gantry) {
    goToPresetLocation(start_);
    flippart_.gantry = start_.gantry;
    flippart_.left_arm = {1.84, -2.73, -1.88, -0.2, 1.63, 0};
    flippart_.right_arm = {1.75, -3.35, -1.4, 0.13, 1.51, 0};
    goToPresetLocation(flippart_);
    auto state1 = getGripperState("right_arm");
    while(!state1.enabled){
        activateGripper("right_arm");
        state1 = getGripperState("right_arm");
    }
    auto state2 = getGripperState("left_arm");
    while(!state1.attached){
        state1 = getGripperState("right_arm");
    }
    while(state2.enabled){
        deactivateGripper("left_arm");
        state2 = getGripperState("left_arm");
    }
    goToPresetLocation(start_);

    ROS_INFO_STREAM("Flipped the part");
    flip_trig = true;

}
/**
 * Finding the position of the required shelf in world frame
 * @param shelf_id
 */
double GantryControl::shelfPosition(std::string shelf_id) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration timeout(5.0);
    geometry_msgs::TransformStamped transformStamped;

    try {
        transformStamped = tfBuffer.lookupTransform("world", shelf_id, ros::Time(0), timeout);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return double(transformStamped.transform.translation.x);
}
/**
 * Determining the gaps required for obstacle avoidance
 */
std::vector<std::string> GantryControl::determineGaps() {
    ROS_INFO_STREAM("Determing the gaps");
    std::string temp;
    int row_no = 0;
    std::string temp_shelf1;
    std::string temp_shelf2;
    double s1, s2, distance_bw_shelves;
    double x_pos;

    for(int row = 3; row <= 11; row = row + 3) {
        for(int shelf = row; shelf < (row + 3)-1; shelf++){
            temp_shelf1 = "shelf"+std::to_string(shelf)+"_frame";
            temp_shelf2 = "shelf"+std::to_string(shelf+1)+"_frame";
            s1 = shelfPosition(temp_shelf1);
            s2 = shelfPosition(temp_shelf2);
            distance_bw_shelves = double(abs(abs(s1)-abs(s2)));

            if(distance_bw_shelves >= gapThreshold[0] 
               && distance_bw_shelves <= gapThreshold[1]) {
                   gap.push_back(row_name[row_no] + std::to_string((shelf+1) % 3));
                   preset_x_gap.push_back(double(s2 + 1.1));
            }
        }
        row_no++;
    }
    
    for(auto val:gap)
        ROS_INFO_STREAM(val);
    
    for(auto val:preset_x_gap)
        ROS_INFO_STREAM(val);
    
    return gap;
}