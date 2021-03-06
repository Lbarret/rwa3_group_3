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

/** \file rwa3_node.cpp
 * Code which controls the robot throughout the ARIAC competition, determining which actions the robot will take in order to complete the order
 */
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

/**
 * Main function for running the ARIAC simulation. It will do the picking and placing of the parts, checking for faulty
 * parts as well as other agility challenges
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char ** argv) {
	part part_in_tray; // Create a variable which holds all the data for the given part in the kit tray

    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    AGVControl agv_control(node);
    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    sensor_read sensors(node);
    sensors.init();

    GantryControl gantry(node);
    gantry.determineGaps();
    ros::Duration(3.0).sleep();
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    std::unordered_map<std::string, bin> bins = {
        {"bin1_", gantry.bin1_},
        {"bin2_", gantry.bin2_},
        {"bin3_", gantry.bin3_},
        {"bin4_", gantry.bin4_},
        {"bin5_", gantry.bin5_},
        {"bin6_", gantry.bin6_},
        {"bin7_", gantry.bin7_},
        {"bin8_", gantry.bin8_},
        {"bin9_", gantry.bin9_},
        {"bin10_", gantry.bin10_},
        {"bin11_", gantry.bin11_},
        {"bin12_", gantry.bin12_},
        {"bin13_", gantry.bin13_},
        {"bin14_", gantry.bin14_},
        {"bin15_", gantry.bin15_},
        {"bin16_", gantry.bin16_},
    };

    std::unordered_map<std::string, std::vector<shelf>> shelves = {
        {"shelf1a_", {gantry.shelf1a_, gantry.shelf1b_, gantry.shelf1c_, gantry.shelf1d_, gantry.shelf1f_, gantry.shelf1g_}},
        {"shelf1b_", {gantry.shelf1a_, gantry.shelf1b_, gantry.shelf1c_, gantry.shelf1e_, gantry.shelf1f_, gantry.shelf1g_}},
        {"shelf2a_", {gantry.shelf2a_, gantry.shelf2b_, gantry.shelf2c_, gantry.shelf2d_, gantry.shelf2f_, gantry.shelf2g_}},
        {"shelf2b_", {gantry.shelf2a_, gantry.shelf2b_, gantry.shelf2c_, gantry.shelf2e_, gantry.shelf2f_, gantry.shelf2g_}},
        {"shelf5a_", {gantry.shelf5a_, gantry.shelf5b_, gantry.shelf5c_, gantry.shelf5d_, gantry.shelf5f_, gantry.shelf5g_}},
        {"shelf5b_", {gantry.shelf5a_, gantry.shelf5b_, gantry.shelf5c_, gantry.shelf5e_, gantry.shelf5f_, gantry.shelf5g_}},
        {"shelf8a_", {gantry.shelf58a_, gantry.shelf58b_, gantry.shelf58c_, gantry.shelf58d_, gantry.shelf58f_, gantry.shelf58g_}},
        {"shelf8b_", {gantry.shelf58a_, gantry.shelf58b_, gantry.shelf58c_, gantry.shelf58e_, gantry.shelf58f_, gantry.shelf58g_}},
        {"shelf11a_", {gantry.shelf811a_, gantry.shelf811b_, gantry.shelf811c_, gantry.shelf811d_, gantry.shelf811f_, gantry.shelf811g_}},
        {"shelf11b_", {gantry.shelf811a_, gantry.shelf811b_, gantry.shelf811c_, gantry.shelf811e_, gantry.shelf811f_, gantry.shelf811g_}},
    };

    //--1-Read order

    std::vector<order> list_of_orders = comp.get_order_list();
    std::string current_agv = "";
    bool check_faulty;
    part faulty_p;
    bool blackout;
    std::vector<product> parts_in_blackout;


    //--2-Look for parts in this order
    
    ros::Duration(2.0).sleep();
    part found_part;
    bool new_order_triggered = false;
    bool agv_cleared = false;
    int order_left_at;

    bool in_isle = false;
    bool need_flip = false;
    int position1 = 0;
    int position2 = 0;
    int gap_location = 0;
    int swapped = -1;
    std::vector<std::string> gaps;

    std::string part_loc = "";
    // Continue to loop through all of the different products in the order until the order has been completed*/
    for (int i=0; i < list_of_orders.size(); i++)
    {
        for (int j=0; j < list_of_orders[i].shipments.size(); j++)
        {
            for (int k=0; k < list_of_orders[i].shipments[j].products.size(); k++)
            {
                gantry.goToPresetLocation(gantry.start_);
                sensors.reset_logicam_update();
                ros::Duration(2.0).sleep();
                // If there is a new order, then get the products in that order */
                if(!new_order_triggered && !sensors.blackout && swapped<k){
                    list_of_orders = comp.get_order_list();

                }
                if (list_of_orders.size() > 1 && !new_order_triggered){
                    new_order_triggered = true;
                    // Check to see if current AGV is needed for the next shipments
                    for (int m = 0; m<list_of_orders[1].shipments.size(); m++){
                        if (list_of_orders[1].shipments[m].agv_id == current_agv){
                            
                            if(current_agv=="agv2") {
                                for(int part = 0; part< k; part++){
                                    part_loc = sensors.find_part(sensors.part_info[13][part].type,2);
                                    found_part = sensors.found_part;                                    
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    if(found_part.pose.position.z>.75+model_height[found_part.type]/2){
                                        found_part.pose.position.z -= model_height[found_part.type];
                                        need_flip = true;
                                    }
                                    else{
                                        need_flip = false;
                                    }
                                    ros::Duration(1.0).sleep();
                                    gantry.pickPart(found_part);
                                    ros::Duration(1.0).sleep();
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    ros::Duration(1.0).sleep();
                                    gantry.goToPresetLocation(gantry.start_);
                                    ros::Duration(1.0).sleep();
                                    if(need_flip) {
                                        ROS_INFO_STREAM("Part needs to be flipped");
                                        gantry.flipPart(bins[part_loc]);
                                        ros::Duration(1.0).sleep();
                                        gantry.goToPresetLocation(gantry.clear_bins_right[part]);
                                        ros::Duration(1.0).sleep();
                                        gantry.deactivateGripper("right_arm");
                                    }
                                    else{
                                        gantry.goToPresetLocation(gantry.clear_bins[part]);
                                        ros::Duration(1.0).sleep();
                                        gantry.deactivateGripper("left_arm");
                                    }
                                    ros::Duration(1.0).sleep();
                                    gantry.goToPresetLocation(gantry.start_);
                                }
                                
                            }
                            else {
                                for(int part = 0; part< k; i++){
                                    part_loc = sensors.find_part(sensors.part_info[12][part].type,2);
                                    found_part = sensors.found_part;                                    
                                    gantry.goToPresetLocation(gantry.agv1_);
                                    if(found_part.pose.position.z>.75+model_height[found_part.type]/2){
                                        found_part.pose.position.z -= model_height[found_part.type];
                                        need_flip = true;
                                    }
                                    else{
                                        need_flip = false;
                                    }
                                    ros::Duration(1.0).sleep();
                                    gantry.pickPart(found_part);
                                    ros::Duration(1.0).sleep();
                                    gantry.goToPresetLocation(gantry.agv1_);
                                    ros::Duration(1.0).sleep();
                                    gantry.goToPresetLocation(gantry.start_);
                                    ros::Duration(1.0).sleep();
                                    if(need_flip) {
                                        ROS_INFO_STREAM("Part needs to be flipped");
                                        gantry.flipPart(bins[part_loc]);
                                        ros::Duration(1.0).sleep();
                                        gantry.goToPresetLocation(gantry.clear_bins_right[part]);
                                        ros::Duration(1.0).sleep();
                                        gantry.deactivateGripper("right_arm");
                                    }
                                    else{
                                        gantry.goToPresetLocation(gantry.clear_bins[part]);
                                        ros::Duration(1.0).sleep();
                                        gantry.deactivateGripper("left_arm");
                                    }
                                    ros::Duration(1.0).sleep();
                                    gantry.goToPresetLocation(gantry.start_);
                                }
                            }
                            agv_cleared = true;
                        }
                    }
                    // Depending on the situation, might be necessary to swap which order is being filled
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
                

                part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,0);
                //--We go to this bin because a camera above
                //--this bin found one of the parts in the order
                ROS_INFO_STREAM_THROTTLE(10,"part location: " << part_loc);
                
                // Check for sensor blackout
                if (part_loc.find("shelf") != std::string::npos && sensors.blackout) {
                    ROS_INFO_STREAM("BLACKOUT");
                    if(k != list_of_orders[i].shipments[j].products.size()-1){
                        auto temp = list_of_orders[i].shipments[j].products[k];
                        list_of_orders[i].shipments[j].products[k] = list_of_orders[i].shipments[j].products[k+1];
                        list_of_orders[i].shipments[j].products[k+1]=temp;
                        swapped = k+1;
                        k-=1;
                    }
                    else{
                        k-=1;
                    }
                    continue;
                }

                // if part isn't found, move onto next part but come back to it
                if (part_loc == "part not found"){
                	if(k != list_of_orders[i].shipments[j].products.size()-1){
                		auto temp = list_of_orders[i].shipments[j].products[k];
                		list_of_orders[i].shipments[j].products[k] = list_of_orders[i].shipments[j].products[k+1];
                		list_of_orders[i].shipments[j].products[k+1]=temp;
                		k-=1;
                	}
                	else{
                	}
                	continue;
                }
                // If part has been found, find exactly where that part is and go to find that part */

                found_part = sensors.found_part;
    			part_in_tray.type = list_of_orders[i].shipments[j].products[k].type;
    			part_in_tray.pose = list_of_orders[i].shipments[j].products[k].pose;
    			current_agv = list_of_orders[i].shipments[j].agv_id;
    			//ROS_INFO_STREAM(current_agv);

                if (part_loc.find("bin") != std::string::npos) {
                    std::cout << "bin was found in string!!" << '\n';
                    gantry.goToPresetLocation(bins[part_loc]);

                    //--Go pick the part
                    ROS_INFO_STREAM("In bin " << part_loc);
                    gantry.pickPart(found_part);
                    ros::Duration(.5).sleep();
                    gantry.goToPresetLocation(bins[part_loc]);
                    ros::Duration(.5).sleep();
                    gantry.goToPresetLocation(gantry.start_);
                    ros::Duration(.5).sleep();


                    // Flipping Part(if condition satisfies)
                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(bins[part_loc]);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }
                    gantry.goToPresetLocation(gantry.start_);
                    // PLacing the part in the agv
                    
                    gantry.placePart(part_in_tray, current_agv);
                    // Check to see if the robot dropped the part at the wrong location
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
                    gantry.goToPresetLocation(gantry.start_);
                }

                if (part_loc.find("shelf") != std::string::npos) {

                    gantry.goToPresetLocation(shelves[part_loc][0]);
                    if(part_loc.find("shelf5") != std::string::npos && sensors.human_in_isle[0]){
                        in_isle = sensors.human_in_isle[0];
                        position1 = 0;
                        position2 = 1;
                        gap_location = 10;
                    }
                    if(part_loc.find("shelf8") != std::string::npos && sensors.human_in_isle[1]){
                        in_isle =sensors.human_in_isle[1];
                        position1 = 2;
                        position2 = 3;
                        gap_location = 8;
                    }
                    if(part_loc.find("shelf11") != std::string::npos && sensors.human_in_isle[2]){
                        in_isle =sensors.human_in_isle[3];
                        position1 = 4;
                        position2 = 5;
                        gap_location = 9;
                    }
                    if(in_isle){
                        while(!sensors.human_check[position1]){
                            ROS_INFO_STREAM_THROTTLE(5,"waiting for human");
                        }
                        while(sensors.human_check[position1]){
                            ROS_INFO_STREAM_THROTTLE(5,"waiting for human to move");
                        }
                        while(!sensors.human_check[gap_location]){
                            ROS_INFO_STREAM_THROTTLE(5,"waiting for human to get to gap");
                        }
                        gantry.goToPresetLocation(shelves[part_loc][4]);
                        ros::Duration(1.0).sleep(); //go to gap
                        gantry.goToPresetLocation(shelves[part_loc][5]);
                        ros::Duration(1.0).sleep(); //go into gap
                        while(!sensors.human_check[position2]){
                            ROS_INFO_STREAM_THROTTLE(5,"waiting for human");
                        }
                        while(sensors.human_check[position2]){
                            ROS_INFO_STREAM_THROTTLE(5,"waiting for human to move");
                        }
                        while(!sensors.human_check[gap_location]){
                            ROS_INFO_STREAM_THROTTLE(5,"waiting for human to clear gap");
                        }
                        gantry.goToPresetLocation(shelves[part_loc][4]);
                        gantry.goToPresetLocation(shelves[part_loc][3]);
                        gantry.pickPart(found_part);
                        gantry.goToPresetLocation(shelves[part_loc][2]);
                        gantry.goToPresetLocation(shelves[part_loc][4]);
                        gantry.goToPresetLocation(shelves[part_loc][5]);
                        while(!sensors.human_check[position2]){
                            ROS_INFO_STREAM_THROTTLE(5,"waiting for human");
                        }
                        gantry.goToPresetLocation(shelves[part_loc][4]);
                        
                    }
                    else{
                        
                        gantry.goToPresetLocation(shelves[part_loc][4]);
                        ros::Duration(1).sleep();
                        gantry.goToPresetLocation(shelves[part_loc][3]);
                        ros::Duration(1).sleep();
                        gantry.pickPart(found_part);
                        ros::Duration(1).sleep();
                        gantry.goToPresetLocation(shelves[part_loc][3]);
                        ros::Duration(1).sleep();
                        gantry.goToPresetLocation(shelves[part_loc][4]);
                        ros::Duration(1).sleep();
                    }
                    
                    gantry.goToPresetLocation(shelves[part_loc][0]);
                    gantry.goToPresetLocation(gantry.start_);
                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(shelves[part_loc][1]);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

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
                    
                    sensors.reset_logicam_update();
                    
                    ros::Duration(2.0).sleep();
                    k--;
                    continue;
                }


                // Check to see if the part is faulty */
			    ros::Duration(2.0).sleep();
                blackout = sensors.blackout;
                if((k == list_of_orders[i].shipments[j].products.size()) && blackout) {
                    ROS_INFO_STREAM("Checking for blackout");
                    ros::Duration(10.0).sleep();
                    blackout = sensors.blackout; 
                }
                if(blackout) {
                    ROS_INFO_STREAM("In blackout");
                    parts_in_blackout.emplace_back(list_of_orders[i].shipments[j].products[k]);
                }
                else {
                    ROS_INFO_STREAM("Not Blackout");
                }
                ros::Duration(2.0).sleep();
			    check_faulty = sensors.get_is_faulty(current_agv);
                if(check_faulty) {
                    if(parts_in_blackout.size() == 0) {
                        ROS_INFO_STREAM("Found faulty part");
                        sensors.reset_faulty();
                        faulty_p = sensors.get_faulty_pose(current_agv);
                        faulty_p.type = part_in_tray.type;
                                                
                        ROS_INFO_STREAM(faulty_p.pose.position.x);
                        ROS_INFO_STREAM(faulty_p.pose.position.y);
                        ROS_INFO_STREAM(faulty_p.pose.position.z);
                        if(current_agv=="agv2") {
                            gantry.goToPresetLocation(gantry.agv2_);
                            if(faulty_p.pose.position.z>.75+model_height[faulty_p.type]/2){
                                        faulty_p.pose.position.z -= model_height[faulty_p.type];
                                    }
                            gantry.pickPart(faulty_p);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.agv2_);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.agv2_faulty);
                            ros::Duration(1).sleep();
                            gantry.deactivateGripper("left_arm");
                        }
                        else {
                            gantry.goToPresetLocation(gantry.agv1_);
                            if(faulty_p.pose.position.z>.75+model_height[faulty_p.type]/2){
                                        faulty_p.pose.position.z -= model_height[faulty_p.type];
                                    }
                            gantry.pickPart(faulty_p);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.agv1_);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.agv1_faulty);
                            ros::Duration(1).sleep();
                            gantry.deactivateGripper("left_arm");
                        }
                        k--;
                        continue;
                    }
                    else {
                        ROS_INFO_STREAM("Found faulty part");
                        sensors.reset_faulty();
                        faulty_p = sensors.get_faulty_pose(current_agv);
                        for (auto check_part:parts_in_blackout) {
                            if (faulty_p.pose.position.x == check_part.pose.position.x &&
                            faulty_p.pose.position.y == check_part.pose.position.y &&
                            faulty_p.pose.position.z == check_part.pose.position.z) {
                                faulty_p.type = check_part.type;
                                list_of_orders[i].shipments[j].products[k] = check_part;
                                parts_in_blackout.clear();
                                break;
                            }
                        }
                        
                        ROS_INFO_STREAM(faulty_p.pose.position.x);
                        ROS_INFO_STREAM(faulty_p.pose.position.y);
                        ROS_INFO_STREAM(faulty_p.pose.position.z);
                        if(current_agv=="agv2") {
                            gantry.goToPresetLocation(gantry.agv2_);
                            if(faulty_p.pose.position.z>.75+model_height[faulty_p.type]/2){
                                        faulty_p.pose.position.z -= model_height[faulty_p.type];
                                    }
                            ros::Duration(1).sleep();        
                            gantry.pickPart(faulty_p);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.agv2_);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.agv2_faulty);
                            ros::Duration(1).sleep();
                            gantry.deactivateGripper("left_arm");
                        }
                        else {
                            gantry.goToPresetLocation(gantry.agv1_);
                            if(faulty_p.pose.position.z>.75+model_height[faulty_p.type]/2){
                                        faulty_p.pose.position.z -= model_height[faulty_p.type];
                                    }
                            ros::Duration(1).sleep();
                            gantry.pickPart(faulty_p);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.agv1_);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.agv1_faulty);
                            ros::Duration(1).sleep();
                            gantry.deactivateGripper("left_arm");
                        }
                        k--;
                        continue;
                    }
                }
                gantry.flip_trig = false;

            }
            // If agv is done, then send the agv */
            if(current_agv == "agv1"){
            	agv_control.sendAGV(list_of_orders[i].shipments[j].shipment_type, "kit_tray_1");

            }
            else{
            	agv_control.sendAGV(list_of_orders[i].shipments[j].shipment_type, "kit_tray_2");
            }
            gantry.goToPresetLocation(gantry.start_);
            ros::Duration(5.0).sleep();
        }
		    
    }

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}
