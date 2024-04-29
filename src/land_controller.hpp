#pragma once
#include "robot.hpp"
#include "ship.hpp"

/**
 * @struct 描述一个机器人的运动状态
 */
struct RobotMotionState {
    RobotMotionState(Position position_, std::map<int, int> dire_val_);
    Position position_;            // 此刻的位置
    std::map<int, int> dire_val_;  // 距离收益表。包含原地不动，不包含非法运动方向。
};


class LandController {
public:
    LandController() = default;

    LandController(const LandController &) = delete;
    LandController &operator=(const LandController &) = delete;

    LandController(LandController &&) = delete;
    LandController &operator=(LandController &&) = delete;

    ~LandController() = default;

    /**
     * @brief 确定每个机器人的运动策略
     */
    void ControlRobots();

private:
    /**
     * @brief 对于机器人 robot，确定它沿不同方向运动的距离增益
     */
    std::map<int, int> GetDirectionMap(const Robot &robot);

    /**
     * @brief 获取所有机器人的 RobotMotionState
     */
    std::vector<RobotMotionState> GetRobotMotionState();

    /**
     * @brief 将所有机器人分组，每组内部有碰撞风险，组之间没有碰撞风险。
     * @return 外层 vector 表示每一个组。内层 vector 表示组内的机器人编号
     */
    std::vector<std::vector<int>> GetRobotGroups();

    /**
     * @brief 协调机器人的碰撞
     * @param motion_state_vec 每个机器人的运动状态
     * @return 每个机器人的运动方向决策
     */
    std::vector<int> AvoidCrash(const std::vector<RobotMotionState> &motion_state_vec);

    /**
     * @brief 较差的机器人防碰撞
     */
    void WorseAvoidCrash(int i, int c);

    // 有可能会与其他队伍的机器人碰撞的位置
    std::shared_ptr<Grid> dangerous_position_ = std::make_shared<Grid>();
};

extern LandController g_land_controller;