/**
 * @file 这个文件记录所有的宏、常数，基本的全局变量声明
 */

#pragma once
#include <atomic>
#include <mutex>
#include <utility>

#define USE_MFMC
#define AVOID_SWING
// #define DONT_USE_ROBOT_AVOID_SWING

#define USE_DFS

const int MAP_SIZE = 800;  // 地图大小
const int INF = 1e9;
const float EPS = 1e-6;  // 浮点数精度，费用流用到
const int MAX_FRAME_NUM = 20000;

const int ship_tar_hash[20] = {5, 2, 3, 0, 4, 1};

const int MAX_ROBOT_NUM = 54;           // 最大的机器人数量
const int MAX_SHIP_NUM = 6;             // 最大的船的数量
const int ROBOT_DANGEROUS_RANGE = 0;    // 其他队伍的危险范围
const int BERTH_SELECT_STEP = 1;        // 泊位选择的步长
const int MAX_REAL_CARGO_NUM = 500;     // CargoManager 存储的货物上限（不严格）
const int MAX_VIRTUAL_CARGO_NUM = 300;  // 虚拟点的数量上限
const int MAX_TOT_CARGO_NUM = MAX_REAL_CARGO_NUM + MAX_VIRTUAL_CARGO_NUM;
const int MIN_ADVANCED_CARGO_VALUE = 1350;

const int MAX_CARGO_NUM_FOR_ROBOT_MATCH = 10;  // 费用流中每个机器人考虑的货物数量上限

const int VIRTUAL_CARGO_BENEFIT_RANGE = 5;  // 虚拟货物计算机器人密度时的范围


const int MAX_ROBOT_CARGO_MATCH_DISTANCE = 200;  // 机器人与货物匹配时，考虑的最远的距离

const int ROBOT_TYPE = 0;             // 机器人的种类
const int BERTH_BFS_MAX_DEPTH = INF;  // 泊位 bfs 的最大深度
const int CARGO_BFS_MAX_DEPTH = INF;  // 货物 bfs 的最大深度
const int ship_crash_limit = 8;
const int predict_step = 6;
const int LLM_THREAD_NUM_0 = 1;   // 向大模型提问的线程数量（CPU 0）
const int LLM_THREAD_NUM_1 = 10;  // 向大模型提问的线程数量（CPU 1）
const int TOT_LLM_THREAD_NUM = LLM_THREAD_NUM_0 + LLM_THREAD_NUM_1;

const int CARGO_INIT_THREAD_NUM = 0;

extern std::mutex cerr_mtx;  // std::cerr 和 fprintf 的线程安全锁


// 用于换方向，可以看 Ship::Rot
const int ROT_RANK[4]{0, 3, 1, 2};
const int ROT_HASH[4]{0, 2, 3, 1};

using Instruction = std::pair<std::string, int>;  // 输出的指令

// 重要全局变量
extern std::atomic_int g_frame_id;  // 当前帧号
extern int g_money;                 // 当前拥有的金钱数
extern int g_ship_capacity;         // 船的容积
extern int g_cargo_num;             // 货物的数量
extern int g_caplim;
extern int g_num_berth;
extern int g_ship_cnt;
extern int g_robot_cnt;
extern int g_value_token;

// 判断一个点是越界
#define IN_MAP(x, y) ((x) >= 0 && (x) < MAP_SIZE && (y) >= 0 && (y) < MAP_SIZE)


const int DEBUG_PRINT_FREQUENCY = 100;
#ifdef LOCAL
// #define DONT_USE_MFMC_LIMITATIONO
#define LOCAL_TIMER
// #define LOCAL_DEBUG_LLM
// #define LOCAL_TIMER
#endif
