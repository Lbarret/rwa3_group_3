#include "sensor_read.h"
#include "utils.h"
#include <array>

#include <std_srvs/Trigger.h>

sensor_read::sensor_read():
camera1_part_list{},
camera2_part_list{},
camera3_part_list{}{
    ROS_INFO_STREAM(">>>>> Subscribing to logical sensors");
    camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10,
                                                &sensor_read:Logical_Camera_1Callback, this);
    camera_2_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_2", 10,
                                                &sensor_read::Logical_Camera_2Callback, this);
    camera_3_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_3", 10,
                                                &sensor_read::Logical_Camera_3Callback, this);
    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;

    init_ = false;
    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;

}

sensor_read::~sensor_read() {}


void sensor_read::logical_camera_1callback(
      const nist_gear::LogicalCameraImage::ConstPtr &image_msg)
{
if (init_) return;
ROS_INFO_STREAM_THROTTLE(10,
"Logical_camera_1: '" << image_msg->models.size() << "' objects.");

if (image_msg->models.size() == 0) {
ROS_ERROR_STREAM("Logical_Camera_1 sees nothing");
}

current_parts_1_ = *image_msg;
this->BuildProductFrames(1);
}


void sensor_read :: Logical_Camera_2Callback(const nist_gear::LogicalCameraImage::ConstPtr &image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 2: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical_Camera_2 sees nothing");

    current_parts_2_ = *image_msg;
    this->BuildProductFrames(2);
}

void sensor_read::Logical_Camera_3Callback(const nist_gear::LogicalCameraImage::ConstPtr &image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 3: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical_Camera_3 sees nothing");

    current_parts_3_ = *image_msg;
    this->BuildProductFrames(3);
}

void sensor_read::ProductFrames(int camera_id){
    if (camera_id == 1) {
        for (auto& msg : current_parts_1_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_1_" + msg.type + "_" +
                                        std::to_string(camera1_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera1_frame_counter_++;
        }
        cam_1_ = true;
    }
    else if (camera_id == 2) {
        for (auto& msg : current_parts_2_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_2_" + msg.type + "_" +
                                        std::to_string(camera2_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera2_frame_counter_++;
        }
        cam_2_ = true;
    }
    else if (camera_id == 3) {
        for (auto& msg : current_parts_3_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_3_" + msg.type + "_" +
                                        std::to_string(camera3_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera3_frame_counter_++;
        }
        cam_3_ = true;
    }
    if (cam_1_ && cam_2_ && cam_3_) {
        init_ = true;
    }
}


geometry_msgs::Pose sensor_read::Part_Poses(const std::string& src_frame,
                                        const std::string& target_frame) {
    geometry_msgs::Pose part_pose;

    ROS_INFO_STREAM("Getting part pose...");

    if (init_) {
        camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                             ros::Duration(3));
        camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                            camera_tf_transform_);

        part_pose.position.x = camera_tf_transform_.getOrigin().x();
        part_pose.position.y = camera_tf_transform_.getOrigin().y();
        part_pose.position.z = camera_tf_transform_.getOrigin().z();

    } else {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(1);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(2);

        part_pose = this->Part_Poses(src_frame, target_frame);
    }

    return part_pose;
}
