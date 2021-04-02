#include "utils.h"

std::unordered_map<std::string, double> model_height = {
        {"piston_rod_part_red", 0.0065}, // modified because it sinks into the surface a bit
        {"piston_rod_part_green", 0.0065},
        {"piston_rod_part_blue", 0.0065},
        {"pulley_part_red", 0.07},
        {"pulley_part_green", 0.07},
        {"pulley_part_blue", 0.07},
        {"gear_part_red", 0.012},
        {"gear_part_green", 0.012},
        {"gear_part_blue", 0.012},
        {"gasket_part_red", 0.02},
        {"gasket_part_green", 0.02},
        {"gasket_part_blue", 0.02},
        {"disk_part_red", 0.023},
        {"disk_part_green", 0.023},
        {"disk_part_blue", 0.023}
};

std::unordered_map<std::string, std::vector<float>> bin_locations = {
        {"bin1_", {2.33,1.62,2.98,1.02}},
        {"bin2_", {3.27,1.62,3.87,1.02}},
        {"bin3_", {4.21,1.62,4.81,1.02}},
        {"bin4_", {5.15,1.62,5.75,1.02}},
        {"bin5_", {2.88,2.41,2.885,1.9}},
        {"bin6_", {3.32,2.41,3.83,1.9}},
        {"bin7_", {4.24,2.41,4.29,1.9}},
        {"bin8_", {5.24,2.41,5.29,1.9}},
        {"bin9_", {2.33,1.62,2.98,1.02}},//Change these in next assignment
        {"bin10_", {3.27,1.62,3.87,1.02}},
        {"bin11_", {4.21,1.62,4.81,1.02}},
        {"bin12_", {5.15,1.62,5.75,1.02}},
        {"bin13_", {2.88,2.41,2.885,1.9}},
        {"bin14_", {3.32,2.41,3.83,1.9}},
        {"bin15_", {4.24,2.41,4.29,1.9}},
        {"bin16_", {5.24,2.41,5.29,1.9}},
        {"shelf5a_", {-12.6,4.43,-10.5,2.93}},
        {"shelf5b_", {-14.6,4.43,-12.4,2.93}},
        {"shelf8a_", {-12.6,1.33,-10.5,-0.171}},
        {"shelf8b_", {-14.6,1.33,-12.4,-0.171}},
        {"shelf11a_", {-12.6,-1.644,-10.5,-3.144}},
        {"shelf11b_", {-14.6,-1.644,-12.4,-3.144}},
};
