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
        {"bin1_", {2.38,1.5,2.88,1.08}},
        {"bin2_", {3.31,1.88,3.82,1.06}},
        {"bin3_", {4.24,1.6,4.29,1.05}},
        {"bin4_", {5.24,1.6,5.29,1.05}},
        {"bin5_", {2.88,2.41,2.885,1.9}},
        {"bin6_", {3.32,2.41,3.83,1.9}},
        {"bin7_", {4.24,2.41,4.29,1.9}},
        {"bin8_", {5.24,2.41,5.29,1.9}}
};