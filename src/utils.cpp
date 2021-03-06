#include "utils.h"
/** \file utils.cpp
 * Code which provides dimension information for the parts as well as location information for the shelves and bins
 */


std::unordered_map<std::string, double> model_height = {
        {"piston_rod_part_red", 0.015}, // modified because it sinks into the surface a bit
        {"piston_rod_part_green", 0.015},
        {"piston_rod_part_blue", 0.015},
        {"pulley_part_red", 0.069262},
        {"pulley_part_green", 0.069262},
        {"pulley_part_blue", 0.069262},
        {"gear_part_red", 0.020346},
        {"gear_part_green", 0.020346},
        {"gear_part_blue", 0.020346},
        {"gasket_part_red", 0.02},
        {"gasket_part_green", 0.02},
        {"gasket_part_blue", 0.02},
        {"disk_part_red", 0.023},
        {"disk_part_green", 0.023},
        {"disk_part_blue", 0.023}
};
/**
 * Create a list for locations of each of the bins and shelves in the environment
 */
std::unordered_map<std::string, std::vector<float>> bin_locations = {
        {"bin1_", {2,2,3,1}},
        {"bin2_", {3,2,4,1}},
        {"bin3_", {4,1.62,5,1.02}},
        {"bin4_", {5.15,1.62,5.75,1.02}},
        {"bin5_", {2.33,2.46,2.93,1.86}},
        {"bin6_", {3.27,2.46,3.87,1.86}},
        {"bin7_", {4.21,2.46,4.81,1.86}},
        {"bin8_", {5.15,2.46,5.75,1.86}},
        {"bin9_", {2.33,-1.02,2.93,-1.62}},
        {"bin10_", {3.27,-1.02,3.87,-1.62}},
        {"bin11_", {4.21,-1.02,4.81,-1.62}},
        {"bin12_", {5.15,-1.02,5.75,-1.62}},
        {"bin13_", {2.33,-1.86,2.93,-2.46}},
        {"bin14_", {3.27,-1.86,3.87,-2.46}},
        {"bin15_", {4.21,-1.86,4.81,-2.46}},
        {"bin16_", {5.15,-1.86,5.75,-2.46}},
        {"shelf1a_", {6.0,4.0,4.0,1.0}},
        {"shelf1b_", {8.0,4.0,6.0,1.0}},
        {"shelf2a_", {6.0,-2.0,4.0,-3.76}},
        {"shelf2b_", {8.0,-2.0,6.0,-3.76}},
        {"shelf5a_", {-14.6,4.0,-12.5,-5.0}},
        {"shelf5b_", {-16.7,4.0,-14.6,-5.0}},
        {"shelf8a_", {-14.6,1.0,-12.5,-2.0}},
        {"shelf8b_", {-16.7,1.0,-14.6,-2.0}},
        {"shelf11a_", {-14.6,-2.0,-12.5,-5.0}},
        {"shelf11b_", {-16.7,-2.0,-14.6,-5.0}},
        {"beltm_", {0,1.5,2.0,-1.5}},
        {"beltf_", {0,4.0,2.0,2.0}},
        {"agv1_", {0,9.0,2.0,5.0}},
        {"agv2_", {0,-5.0,2.0,-9.0}},
};

