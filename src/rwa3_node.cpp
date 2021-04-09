// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


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

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"
#include "agv_control.h"
#include "sensor_read.h"

#include <tf2/LinearMath/Quaternion.h>


int main(int argc, char ** argv) {
	part part_in_tray;

    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    AGVControl agv_control(node);
    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    //--1-Read order

    std::vector<order> list_of_orders = comp.get_order_list();
    std::string current_agv = "";
    bool check_faulty;
    part faulty_p;


    //--2-Look for parts in this order
    sensor_read sensors(node);
    sensors.init();
    ros::Duration(2.0).sleep();
    part found_part;
    bool new_order_triggered = false;
    bool agv_cleared = false;
    int order_left_at;

    std::string part_loc = "";
    for (int i=0; i < list_of_orders.size(); i++)
    {
        for (int j=0; j < list_of_orders[i].shipments.size(); j++)
        {
            for (int k=0; k < list_of_orders[i].shipments[j].products.size(); k++)
            {
                if(!new_order_triggered){
                    list_of_orders = comp.get_order_list();
                }
                if (list_of_orders.size() > 1 && !new_order_triggered){
                    new_order_triggered = true;
                    // Check to see if current AGV is needed for the next shipments
                    for (int m = 0; m<list_of_orders[1].shipments.size(); m++){
                        if (list_of_orders[1].shipments[m].agv_id == current_agv){
                            ROS_INFO_STREAM("Clear AGV");
                            agv_cleared = true;
                        }
                    }
                    auto temp = list_of_orders[0];
                    list_of_orders[0] = list_of_orders[1];
                    list_of_orders[1] = temp;
                    order_left_at = k;
                    k = -1;
                    ROS_INFO_STREAM("Order swapped");
                    continue;
                }
                if(new_order_triggered && i == 1 && !agv_cleared && k==0){
                    k = order_left_at -1;
                    ROS_INFO_STREAM("pick back up");
                    continue;
                }
                sensors.reset_logicam_update();
                ros::Duration(1.0).sleep();
                part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,0);
                    // //--We go to this bin because a camera above
                   //--this bin found one of the parts in the order
                ROS_INFO_STREAM_THROTTLE(10,"part location: " << part_loc);
                // if part isn't found, move onto next part but come back to it
                if (part_loc == "part not found"){
                	if(k != list_of_orders[i].shipments[j].products.size()-1){
                		// auto temp = list_of_orders[i].shipments[j].products[k];
                		// list_of_orders[i].shipments[j].products[k] = list_of_orders[i].shipments[j].products[k+1];
                		// list_of_orders[i].shipments[j].products[k+1]=temp;
                        ROS_INFO_STREAM_THROTTLE(10,"waiting for first part");
                		k-=1;
                	}
                	else{
                		k-=1;
                	}
                	continue;
                }
                	
                found_part = sensors.found_part;
    			part_in_tray.type = list_of_orders[i].shipments[j].products[k].type;
    			part_in_tray.pose = list_of_orders[i].shipments[j].products[k].pose;
    			current_agv = list_of_orders[i].shipments[j].agv_id;
    			//ROS_INFO_STREAM(current_agv);

    			if (part_loc == "bin1_"){
			    	gantry.goToPresetLocation(gantry.bin1_);

                    //--Go pick the part
                    ROS_INFO_STREAM("In bin 1");
                    gantry.pickPart(found_part);

                    // Flipping Part(if condition satisfies)
                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin1_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    // PLacing the part in the agv
                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

			    if (part_loc == "bin2_"){
                    // Go to bin
			    	gantry.goToPresetLocation(gantry.bin2_);

                    //--Go pick the part
                    ROS_INFO_STREAM("In bin 2");
                    gantry.pickPart(found_part);
                    
                    // Flipping Part(if condition satisfies)
                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin2_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    // Placing the part in the agv
                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

			    if (part_loc == "bin3_"){

			    	gantry.goToPresetLocation(gantry.bin3_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin3_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin4_"){

                    gantry.goToPresetLocation(gantry.bin4_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin4_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin5_"){

                    gantry.goToPresetLocation(gantry.bin5_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin5_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin6_"){

                    gantry.goToPresetLocation(gantry.bin6_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin6_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin7_"){

                    gantry.goToPresetLocation(gantry.bin7_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin7_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin8_"){

                    gantry.goToPresetLocation(gantry.bin8_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin8_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin9_"){

                    gantry.goToPresetLocation(gantry.bin9_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin9_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin10_"){

                    gantry.goToPresetLocation(gantry.bin10_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin10_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin11_"){

                    gantry.goToPresetLocation(gantry.bin11_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin11_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin12_"){

                    gantry.goToPresetLocation(gantry.bin12_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin12_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin13_"){

                    gantry.goToPresetLocation(gantry.bin13_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin13_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);

                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin14_"){

                    gantry.goToPresetLocation(gantry.bin14_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin14_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin15_"){

                    gantry.goToPresetLocation(gantry.bin15_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin15_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if (part_loc == "bin16_"){

                    gantry.goToPresetLocation(gantry.bin16_);

                    gantry.pickPart(found_part);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.bin16_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.start_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if(part_loc == "shelf5a_" || part_loc == "shelf5b_"){
                    gantry.goToPresetLocation(gantry.shelf5a_);
                    gantry.goToPresetLocation(gantry.shelf5b_);
                    gantry.goToPresetLocation(gantry.shelf5c_);
                    if (part_loc == "shelf5a_"){
                    	gantry.goToPresetLocation(gantry.shelf5d_);
                    }
                    else{
                    	gantry.goToPresetLocation(gantry.shelf5e_);
                    }
                    gantry.pickPart(found_part);
                    gantry.goToPresetLocation(gantry.shelf5c_);
                    gantry.goToPresetLocation(gantry.shelf5b_);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.shelf5b_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(gantry.shelf5a_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                 if(part_loc == "shelf8a_" || part_loc == "shelf8b_"){
                    gantry.goToPresetLocation(gantry.shelf58a_);
                    gantry.goToPresetLocation(gantry.shelf58b_);
                    gantry.goToPresetLocation(gantry.shelf58c_);
                    if (part_loc == "shelf8a_"){
                    	gantry.goToPresetLocation(gantry.shelf58d_);
                    }
                    else{
                    	gantry.goToPresetLocation(gantry.shelf58e_);
                    }
                    gantry.pickPart(found_part);
                    gantry.goToPresetLocation(gantry.shelf58c_);
                    gantry.goToPresetLocation(gantry.shelf58b_);
                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.shelf58b_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }
                    gantry.goToPresetLocation(gantry.shelf58a_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                if(part_loc == "shelf11a_" || part_loc == "shelf11b_"){
                    gantry.goToPresetLocation(gantry.shelf811a_);
                    gantry.goToPresetLocation(gantry.shelf811b_);
                    gantry.goToPresetLocation(gantry.shelf811c_);
                    if (part_loc == "shelf11a_"){
                    	gantry.goToPresetLocation(gantry.shelf811d_);
                    }
                    else{
                    	gantry.goToPresetLocation(gantry.shelf811e_);
                    }
                    gantry.pickPart(found_part);
                    gantry.goToPresetLocation(gantry.shelf811c_);
                    gantry.goToPresetLocation(gantry.shelf811b_);
                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(gantry.shelf811b_);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }
                    gantry.goToPresetLocation(gantry.shelf811a_);
                    gantry.placePart(part_in_tray, current_agv);
                    while(gantry.part_dropped){
                        sensors.reset_logicam_update();
                        ros::Duration(1.0).sleep();
                        if(current_agv == "agv1"){
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                        }
                        else{
                            part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                        }
                        
                        found_part = sensors.found_part;
                        gantry.pickPart(found_part);
                        if(part_loc == "part not found"){
                            k--;
                            continue;
                        }
                        gantry.placePart(part_in_tray, current_agv);
                    }
                }

                    if(part_loc == "beltm_" || part_loc == "beltf_"){
                    gantry.goToPresetLocation(gantry.conveyor_);
                    gantry.pickPartConveyor(found_part);
                    gantry.goToPresetLocation(gantry.conveyor_bin1_);
                    gantry.deactivateGripper("left_arm");
                    ros::Duration(2.0).sleep();
                    k--;
                    continue;
                    }

                    
                
			    ros::Duration(2.0).sleep();
			    check_faulty = sensors.get_is_faulty(current_agv);
                if(check_faulty) {
                    ROS_INFO_STREAM("Found faulty part");
                    sensors.reset_faulty();
                    faulty_p = sensors.get_faulty_pose(current_agv);
                    faulty_p.type = part_in_tray.type;
                    faulty_p.pose.position.z +=.015;
                    ROS_INFO_STREAM(faulty_p.pose.position.x);
                    ROS_INFO_STREAM(faulty_p.pose.position.y);
                    ROS_INFO_STREAM(faulty_p.pose.position.z);
                    if(current_agv=="agv2") {
                        gantry.goToPresetLocation(gantry.agv2_);
                        gantry.pickPart(faulty_p);
                        gantry.goToPresetLocation(gantry.agv2_);
                        gantry.goToPresetLocation(gantry.agv2_faulty);
                        gantry.deactivateGripper("left_arm");
                    }
                    else {
                        gantry.goToPresetLocation(gantry.agv1_);
                        gantry.pickPart(faulty_p);
                        gantry.goToPresetLocation(gantry.agv1_);
                        gantry.goToPresetLocation(gantry.agv1_faulty);
                        gantry.deactivateGripper("left_arm");
                    }
                    k--;
                    continue;
                }

            }

            if(current_agv == "agv1"){
            	agv_control.sendAGV(list_of_orders[i].shipments[j].shipment_type, "kit_tray_1");

            }
            else{
            	agv_control.sendAGV(list_of_orders[i].shipments[j].shipment_type, "kit_tray_2");
            }
            gantry.goToPresetLocation(gantry.start_);
        }
		    
    }

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}