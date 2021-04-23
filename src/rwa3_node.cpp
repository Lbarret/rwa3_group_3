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
 * Code for rwa4 which helps the robot navigate the ARIAC world and complete the agility challenges
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

    GantryControl gantry(node);
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
        {"shelf5a_", {gantry.shelf5a_, gantry.shelf5b_, gantry.shelf5c_, gantry.shelf5d_, gantry.shelf5f_}},
        {"shelf5b_", {gantry.shelf5a_, gantry.shelf5b_, gantry.shelf5c_, gantry.shelf5e_, gantry.shelf5f_}},
        {"shelf8a_", {gantry.shelf58a_, gantry.shelf58b_, gantry.shelf58c_, gantry.shelf58d_, gantry.shelf5f_}},
        {"shelf8b_", {gantry.shelf58a_, gantry.shelf58b_, gantry.shelf58c_, gantry.shelf58e_, gantry.shelf5f_}},
        {"shelf11a_", {gantry.shelf811a_, gantry.shelf811b_, gantry.shelf811c_, gantry.shelf811d_, gantry.shelf5f_}},
        {"shelf11b_", {gantry.shelf811a_, gantry.shelf811b_, gantry.shelf811c_, gantry.shelf811e_, gantry.shelf5f_}},
    };

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

    //--3-Check where humans are
    int human_aisle_one = 0;
    int human_aisle_two = 0;
    int human_aisle_three = 0;
    int human_aisle_four = 0;
    int human_hole_one = 0;
    // human_aisle_one = sensors.check_human_aisle_one();
    // human_aisle_two = sensors.check_human_aisle_two();
    // human_aisle_three = sensors.check_human_aisle_three();
    // human_aisle_four = sensors.check_human_aisle_four();

    // ROS_INFO_STREAM(human_aisle_one);
    // ROS_INFO_STREAM(human_aisle_two);
    // ROS_INFO_STREAM(human_aisle_three);
    // ROS_INFO_STREAM(human_aisle_four);
    // if (human_aisle_one == 1){
    //     ROS_INFO_STREAM("Human obstacle has been found in aisle one");
    // } 
    // if (human_aisle_two == 1){
    //     ROS_INFO_STREAM("Human obstacle has been found in aisle two");
    // } 
    // if (human_aisle_three == 1){
    //     ROS_INFO_STREAM("Human obstacle has been found in aisle three");
    // } 
    // if (human_aisle_four == 1){
    //     ROS_INFO_STREAM("Human obstacle has been found in aisle four");
    // } 
    // if (human_aisle_one == 0 && human_aisle_two == 0 && human_aisle_three == 0 && human_aisle_four == 0){
    //     ROS_INFO_STREAM("No human obstacles in the facitily");
    // }

    std::string part_loc = "";
    /*! Continue to loop through all of the different products in the order until the order has been completed*/
    for (int i=0; i < list_of_orders.size(); i++)
    {
        for (int j=0; j < list_of_orders[i].shipments.size(); j++)
        {
            for (int k=0; k < list_of_orders[i].shipments[j].products.size(); k++)
            {
                gantry.goToPresetLocation(gantry.start_);
                /*! If there is a new order, then get the products in that order */
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
                /*! If part has been found, find exactly where that part is and go to find that part */

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

                    // Flipping Part(if condition satisfies)
                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(bins[part_loc]);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    // PLacing the part in the agv
                    gantry.goToPresetLocation(gantry.start_);
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
                }

                if (part_loc.find("shelf") != std::string::npos) {

                    human_aisle_four = sensors.check_human_aisle_one();
                    if (human_aisle_four == 1){
                        ROS_INFO_STREAM("Human in aisle four");
                        ros::Duration(4.0).sleep();
                    }
                    gantry.goToPresetLocation(shelves[part_loc][0]);
                    gantry.goToPresetLocation(shelves[part_loc][1]);
                    gantry.goToPresetLocation(shelves[part_loc][2]);
                    gantry.goToPresetLocation(shelves[part_loc][3]);
                    gantry.pickPart(found_part);
                    gantry.goToPresetLocation(shelves[part_loc][2]);
                    gantry.goToPresetLocation(shelves[part_loc][1]);

                    if(part_in_tray.pose.orientation.x==1) {
                        ROS_INFO_STREAM("Part needs to be flipped");
                        gantry.flipPart(shelves[part_loc][1]);
                        part_in_tray.pose.orientation.x = 0;
                        part_in_tray.pose.orientation.w = 1;
                    }

                    gantry.goToPresetLocation(shelves[part_loc][0]);
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

                //  if(part_loc == "shelf8a_" || part_loc == "shelf8b_"){
                //     gantry.goToPresetLocation(gantry.shelf58a_);
                //     human_aisle_three = sensors.check_human_aisle_three();
                //     if (human_aisle_three == 1){
                //         ROS_INFO_STREAM("Human in aisle three");
                //         gantry.goToPresetLocation(gantry.shelf5a_);
                //         gantry.goToPresetLocation(gantry.shelf5f_); // go around to the outer aisle
                //         human_aisle_three = sensors.check_human_aisle_three();
                //         human_hole_one = sensors.check_human_hole_one();
                //         while (human_aisle_three == 0 && human_hole_one == 0){ // wait until the area by part is clear
                //             ros::Duration(2.0).sleep();
                //             human_aisle_three = sensors.check_human_aisle_three();
                //             human_hole_one = sensors.check_human_hole_one();
                //         }
                //         if (human_aisle_three == 1 && human_hole_one == 0){ // once area by aisle is clear, go pick up part
                //             gantry.goToPresetLocation(gantry.shelf58f_);
                //         }
                //         gantry.goToPresetLocation(gantry.shelf58b_);
                //         // gantry.goToPresetLocation(gantry.shelf58c_);
                //         if (part_loc == "shelf8a_"){
                //             gantry.goToPresetLocation(gantry.shelf58d_);
                //         }
                //         else{
                //             gantry.goToPresetLocation(gantry.shelf58e_);
                //         }
                //         gantry.pickPart(found_part);
                //         // gantry.goToPresetLocation(gantry.shelf58c_);
                //         gantry.goToPresetLocation(gantry.shelf58b_);
                //         gantry.goToPresetLocation(gantry.shelf58f_);
                //         gantry.goToPresetLocation(gantry.shelf5f_);
                //         if(part_in_tray.pose.orientation.x==1) {  // wait to flip part, until robot is safe from human
                //             ROS_INFO_STREAM("Part needs to be flipped");
                //             gantry.flipPart(gantry.shelf5f_);
                //             part_in_tray.pose.orientation.x = 0;
                //             part_in_tray.pose.orientation.w = 1;
                //         }
                //         gantry.goToPresetLocation(gantry.shelf5a_);
                //         gantry.placePart(part_in_tray, current_agv);
                //         while(gantry.part_dropped){
                //             sensors.reset_logicam_update();
                //             ros::Duration(1.0).sleep();
                //             if(current_agv == "agv1"){
                //                 part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                //             }
                //             else{
                //                 part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                //             }
                            
                //             found_part = sensors.found_part;
                //             gantry.pickPart(found_part);
                //             if(part_loc == "part not found"){
                //                 k--;
                //                 continue;
                //             }
                //             gantry.placePart(part_in_tray, current_agv);
                //         }
                //     } else {
                //         gantry.goToPresetLocation(gantry.shelf58b_);
                //         gantry.goToPresetLocation(gantry.shelf58c_);
                //         if (part_loc == "shelf8a_"){
                //             gantry.goToPresetLocation(gantry.shelf58d_);
                //         }
                //         else{
                //             gantry.goToPresetLocation(gantry.shelf58e_);
                //         }
                //         gantry.pickPart(found_part);
                //         gantry.goToPresetLocation(gantry.shelf58c_);
                //         gantry.goToPresetLocation(gantry.shelf58b_);
                //         if(part_in_tray.pose.orientation.x==1) {
                //             ROS_INFO_STREAM("Part needs to be flipped");
                //             gantry.flipPart(gantry.shelf58b_);
                //             part_in_tray.pose.orientation.x = 0;
                //             part_in_tray.pose.orientation.w = 1;
                //         }
                //         gantry.goToPresetLocation(gantry.shelf58a_);
                //         gantry.placePart(part_in_tray, current_agv);
                //         while(gantry.part_dropped){
                //             sensors.reset_logicam_update();
                //             ros::Duration(1.0).sleep();
                //             if(current_agv == "agv1"){
                //                 part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                //             }
                //             else{
                //                 part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                //             }
                            
                //             found_part = sensors.found_part;
                //             gantry.pickPart(found_part);
                //             if(part_loc == "part not found"){
                //                 k--;
                //                 continue;
                //             }
                //             gantry.placePart(part_in_tray, current_agv);
                //         }
                //     }
                //  }

                // if(part_loc == "shelf11a_" || part_loc == "shelf11b_"){
                //     human_aisle_two = sensors.check_human_aisle_two();
                //     if (human_aisle_two == 1){
                //         ROS_INFO_STREAM("Human in aisle two");
                //         ros::Duration(4.0).sleep();
                //     }
                //     gantry.goToPresetLocation(gantry.shelf811a_);
                //     gantry.goToPresetLocation(gantry.shelf811b_);
                //     gantry.goToPresetLocation(gantry.shelf811c_);
                //     if (part_loc == "shelf11a_"){
                //     	gantry.goToPresetLocation(gantry.shelf811d_);
                //     }
                //     else{
                //     	gantry.goToPresetLocation(gantry.shelf811e_);
                //     }
                //     gantry.pickPart(found_part);
                //     gantry.goToPresetLocation(gantry.shelf811c_);
                //     gantry.goToPresetLocation(gantry.shelf811b_);
                //     if(part_in_tray.pose.orientation.x==1) {
                //         ROS_INFO_STREAM("Part needs to be flipped");
                //         gantry.flipPart(gantry.shelf811b_);
                //         part_in_tray.pose.orientation.x = 0;
                //         part_in_tray.pose.orientation.w = 1;
                //     }
                //     gantry.goToPresetLocation(gantry.shelf811a_);
                //     gantry.placePart(part_in_tray, current_agv);
                //     while(gantry.part_dropped){
                //         sensors.reset_logicam_update();
                //         ros::Duration(1.0).sleep();
                //         if(current_agv == "agv1"){
                //             part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,1);
                //         }
                //         else{
                //             part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type,2);
                //         }
                        
                //         found_part = sensors.found_part;
                //         gantry.pickPart(found_part);
                //         if(part_loc == "part not found"){
                //             k--;
                //             continue;
                //         }
                //         gantry.placePart(part_in_tray, current_agv);
                //     }
                // }

                if(part_loc == "beltm_" || part_loc == "beltf_"){
                    gantry.goToPresetLocation(gantry.conveyor_);
                    gantry.pickPartConveyor(found_part);
                    gantry.goToPresetLocation(gantry.conveyor_bin1_);
                    gantry.deactivateGripper("left_arm");
                    ros::Duration(2.0).sleep();
                    k--;
                    continue;
                }


                /*! Check to see if the part is faulty */
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
            /*! If agv is done, then send the agv */
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