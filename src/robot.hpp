#pragma once

#include "cargo.hpp"
#include "config.hpp"
#include "map.hpp"
#include "question.hpp"
#include "utils.hpp"

class Cargo;
using CargoPtr = std::shared_ptr<Cargo>;

/**
 * @class 主要目的是维护以下信息：
 *  位置，是否携带物品，目标货物，目标泊位，下一步的防线
 */
class Robot {
public:
    Robot(int id, Position pos, int vol);

    // 禁用复制
    Robot(const Robot &) = delete;
    Robot &operator=(const Robot &) = delete;

    // 允许移动构造函数，禁用移动构造运算符
    Robot(Robot &&) noexcept;
    Robot &operator=(Robot &&) = delete;

    // 默认析构
    ~Robot() = default;

    /**
     * @brief 读入这一帧该机器人参数，同时 dist_grid。
     */
    void InputFrame();

    /**
     * @brief 输出这一帧该机器人指令，并清空指令队列
     */
    void OutputFrame();

    /**
     * @brief 是否携带货物
     */
    bool FullCargo() const noexcept;

    /**
     * @brief 查询位置
     */
    Position GetPosition() const noexcept;

    /**
     * @brief 设置准备去取得 cargo
     */
    void SetCargoPtr(const CargoPtr &cargo) noexcept;

    /**
     * @brief 返回携带的货物，或者正在去取的货物
     */
    CargoPtr GetCargoPtr() const noexcept;

    /**
     * @brief 设置目的泊位的 ID
     */
    void SetTargetBerthID(int id) noexcept;

    /**
     * @brief 查询目的泊位的 ID
     */
    int GetTargetBerthID() const noexcept;

    /**
     * @brief 设定下一步机器人的移动方向 (0 ~ 5)
     */
    void SetMoveDirection(int direction) noexcept;

    /**
     * @brief 查询下一步机器人的移动方向
     */
    int GetMoveDirection() noexcept;

    /**
     * @brief 设置问题内容
     */
    void AskQuestion(std::string question) noexcept;

    /**
     * @brief 回答问题
     */
    void AnswerQuestion() noexcept;

    /**
     * @brief 给机器人新增额外的货物（虚拟货物）
     */
    void SetAddtionalTarget(CargoPtr car) noexcept;

    /**
     * @brief 获取额外货物的目的地（虚拟货物）
     */
    CargoPtr GetAddtionalTarget() const noexcept;

    /**
     * @brief 判断这个机器人是否正在等待 LLM
     */
    bool IsWaitingForLLM() const noexcept;

    const int id_;           // 机器人的编号
    static int value_sent_;  // 机器人总共送的钱

private:
    int cargo_num_{0};                              // 携带物品数量
    int volume_{1};                                 // 机器人容量
    int target_berth_id_{-1};                       // 目标泊位 id
    Position position_{-1, -1};                     // 此刻坐标
    std::vector<Instruction> instructions_;         // 这一帧需要输出的指令
    std::stack<std::shared_ptr<Cargo>> cargo_ptr_;  // 携带的货物，或者目标货物
    int move_dir_{4};                               // 前进方向
    std::string lst_instruction_;                   // 上一帧的指令，DEBUG 用
    std::map<Position, int> pass_;  // 此机器人取货后到达泊位前途经点，到泊位后清空
    QuestionPtr question_ptr_;      // 问题
    CargoPtr addtional_target_;     // 给有货机器人的额外任务(占领高价值货物)
};

/**
 * @brief 记录别的队伍的机器人
 */
struct OtherRobot {
    int id_;
    int cargo_num_;
    int volume_;
    bool cargo_change_{false};
    std::stack<int> cargo_value_;
    Position position_;
};

extern std::vector<Robot> g_robots;
extern std::map<int, OtherRobot> g_other_robots;
extern int g_other_robot_num;
extern std::map<int, int> g_robot_volume;
extern std::map<int, int> g_self_robot_volume;
extern std::map<int, bool> g_robot_owned;