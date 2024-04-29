#pragma once
#include "robot.hpp"
#include "ship.hpp"
#include "utils.hpp"

class Astar {
public:
    /**
     * @brief 计算船到达目标地预估时间开销
     * @param pos 位置
     * @param dir 方向
     * @return 开销
     */
    int CalcHcost(Position& pos, int& dir);

    /**
     * @brief 计算船为去往目标地的下一步应该做的动作
     * @return 0、1：旋转方向；2：直行
     */
    int FindNextStep();

    /**
     * @brief 计算船是否应该止步
     * @param pos 位置
     * @param dir 方向
     */
    bool CheckStop(Position& pos, int& dir);

    /**
     * @brief 计算船是否会碰撞
     * @param pos 位置
     * @param dir 方向
     * @return 1：碰撞；0：不碰撞
     */
    bool CheckCrash(Position& pos, int& dir);

    /**
     * @brief 查看控制系统上有没有记录其他船存在
     * @return 1：有；0：无
     */
    bool CheckEasy();

    /**
     * @brief 将一艘船载入此控制器
     * @param ship_index 船序号
     */
    void LoadShip(int ship_index);

    // void SetEndPoint(Position);

private:
    std::vector<Position> pos_stack_;  // 使用A*算法进行搜索的位置栈
    std::vector<int> dir_stack_;       // 使用A*算法进行搜索的方向栈
    std::vector<int> dfs_stack_;       // 使用A*算法进行搜索的记录栈
    std::vector<int> res_stack_;       // 使用A*算法进行搜索的结果栈

    std::array<Position, 3> next_pos_;
    std::array<int, 3> next_dir_;
    std::array<int, 3> next_cost_;

    int ship_index_;                         // 此调度器所负责船id
    std::vector<Position> other_ships_pos_;  // 其他船坐标集合
};

class OceanController {
public:
    OceanController() = default;

    OceanController(const OceanController&) = delete;
    OceanController& operator=(const OceanController&) = delete;

    OceanController(OceanController&&) = delete;
    OceanController& operator=(OceanController&&) = delete;

    /**
     * @brief 确定每条船的运动策略
     */
    void ControlShips();
    void ControlShips2();

private:
    /**
     * @brief 简单方法估计两船碰撞
     */
    bool CheckShipMayCrash(int ship_a, int ship_b);

    /**
     * @brief 确定两船是否碰撞
     */
    bool CheckShipCrash(int ship_a, int ship_b);

    /**
     * @brief 避免两条船的碰撞
     */
    void SolveShipCrash(std::vector<int> crash_set);

    /**
     * @brief 给两船尝试发出接触碰撞指令
     */
    void SolveTwoShipCrash(int ship_a, int ship_b);

    /**
     * @brief 判断海上的一个方位会不会和其他组的船只碰撞
     * @return 如果会发生碰撞就返回 true
     */
    bool CheckShipCollision(Position nxt, int dir);

    /**
     * @brief 使用A*算法计算一艘船应执行的下一个指令
     * @param ship_index 船序号
     * @return 指令
     */
    Instruction AstarFindNextInstruction(int ship_index);

    Astar astar;
};

extern OceanController g_ocean_controller;