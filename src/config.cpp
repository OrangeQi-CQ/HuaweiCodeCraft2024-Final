#include "config.hpp"

std::mutex cerr_mtx;

std::atomic_int g_frame_id = -1;
int g_money = -1;
int g_ship_capacity = -1;
bool stop = false;

// DEBUG
int value_got_berth = 0;
int g_value_token = 0;

int g_num_berth;
int g_ship_cnt;
int g_robot_cnt = 0;
int g_cargo_num = 0;

int g_other_robot_num;
int g_other_ship_num;

int g_caplim = 5;