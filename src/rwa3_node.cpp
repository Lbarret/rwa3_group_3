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

// void pickandplace(int logi_cam_id, auto product, std::string part_loc, auto sensors_info, GantryControl& gantry) {
//     part part_to_pick;
//     for(int i=0;i<sensors_info[logi_cam_id].size();i++){
//         if(product.type==sensors_info[logi_cam_id][i].type){
//             part_to_pick.pose = sensors_info[logi_cam_id][i].pose;
//         }
//     }
//     if(part_loc=="bin3_") {
//         gantry.goToPresetLocation(gantry.bin3_);
//     }
//     gantry.pickPart(part_to_pick);
//     gantry.goToPresetLocation(gantry.agv2_);
//     gantry.goToPresetLocation(gantry.start_);

// }

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

    // comp.order_callback();
    std::vector<order> list_of_orders = comp.get_order_list();
    //ROS_INFO_STREAM(list_of_products[0].type);
    std::string current_agv = "";
    //ROS_INFO_STREAM(current_agv);
    //ROS_INFO_STREAM(current_agv);
    //ROS_INFO_STREAM(list_of_orders[0].shipments[0].products[0].type);
    bool check_faulty;
    part faulty_part;


    //--2-Look for parts in this order

    sensor_read sensors(node);
    sensors.init();
    while(sensors.part_info[0][0].type == ""){
    	ROS_INFO_STREAM_THROTTLE(10,"waiting for cameras");
    }
    
    //ROS_INFO_STREAM(sensors.part_info_type[0][0]);
    auto sensors_parts_info = sensors.get_part_info();
    int logi_cam_id = sensors.get_logi_cam();
    // ROS_INFO_STREAM("Part type: " << sensors_parts_info[5][5].pose.position.x);
    std::string part_loc = "";
    for (int i=0; i < list_of_orders.size(); i++)
    {
        for (int j=0; j < list_of_orders[i].shipments.size(); j++)
        {
            for (int k=0; k < list_of_orders[i].shipments[j].products.size(); k++)
            {
                part_loc = sensors.find_part(list_of_orders[i].shipments[j].products[k].type);
                    // //--We go to this bin because a camera above
                   //--this bin found one of the parts in the order
                ROS_INFO_STREAM("part location: " << part_loc);
                
    			part_in_tray.type = list_of_orders[i].shipments[j].products[k].type;
    			part_in_tray.pose = list_of_orders[i].shipments[j].products[k].pose;
    			current_agv = list_of_orders[i].shipments[j].agv_id;
    			ROS_INFO_STREAM(current_agv);

    			if (part_loc == "bin1_"){
			    	gantry.goToPresetLocation(gantry.bin1_);
			    }
			    if (part_loc == "bin2_"){
			    	gantry.goToPresetLocation(gantry.bin2_);
			    }
			    if (part_loc == "bin3_"){
			    	gantry.goToPresetLocation(gantry.bin3_);
			    }
			    
                //--Go pick the part
			    if (!gantry.pickPart(sensors.found_part)){
			        gantry.goToPresetLocation(gantry.start_);
			        spinner.stop();
			        ros::shutdown();
			    }
			    
			    //--Go place the part
			    //--TODO: agv2 should be retrieved from /ariac/orders (list_of_products in this case)
			    gantry.placePart(part_in_tray, current_agv);
                //pickandplace(logi_cam_id,list_of_orders[i].shipments[j].products[k], part_loc, sensors_parts_info, gantry);

                check_faulty = sensors.get_is_faulty(current_agv);
                if(check_faulty) {
                    ROS_INFO_STREAM("Found faulty part");
                    sensors.reset_faulty();
                    faulty_part.pose = sensors.get_faulty_pose(current_agv);
                    if(current_agv=="agv2") {
                        gantry.goToPresetLocation(gantry.agv2_);
                        gantry.pickPart(faulty_part);
                        gantry.goToPresetLocation(gantry.agv2_);
                        gantry.goToPresetLocation(gantry.agv2_faulty);
                        gantry.deactivateGripper("left_arm");
                    }
                    else {
                        gantry.goToPresetLocation(gantry.agv1_);
                        gantry.pickPart(faulty_part);
                        gantry.goToPresetLocation(gantry.agv1_);
                        gantry.goToPresetLocation(gantry.agv1_faulty);
                        gantry.deactivateGripper("left_arm");
                    }
                    continue;

                }
            }
            //--TODO: get the following arguments from the order
            if(current_agv == "agv1"){
            	agv_control.sendAGV(list_of_orders[i].shipments[j].shipment_type, "kit_tray_1");

            }
            else{
            	agv_control.sendAGV(list_of_orders[i].shipments[j].shipment_type, "kit_tray_2");

            }
        }
		    
    }

    

   

    
    //--TODO: Parse each product in list_of_products
    //--TODO: For each product in list_of_product find the part in the environment using cameras
    //--TODO: Choose one part and pick it up
    //--Assume the following part was found by a camera

    //--Where to place the part in the tray?
    //--TODO: Get this information from /ariac/orders (list_of_products in this case)
    




    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}