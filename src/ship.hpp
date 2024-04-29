#pragma once

#include "config.hpp"
#include "map.hpp"
#include "utils.hpp"

using ShipPositions = std::array<Position, 6>;

class Ship {
public:
    Ship(int id, Position pos, int dir, int status);

    Ship(const Ship &) = delete;
    Ship &operator=(const Ship &) = delete;

    Ship(Ship &&) noexcept;
    Ship &operator=(Ship &&) = delete;

    ~Ship() = default;

    /**
     * @brief 在每一帧开始时读入该船的参数
     */
    void InputFrame();

    /**
     * @brief 在每一帧结束时输出该船的参数
     */
    void OutputFrame();

    /**
     * @brief 确定这艘船应该去那里
     */
    void Drive();

    /**
     * @brief 更新船的状态
     */
    void Arrive();

    /**
     * @brief 进行这一帧的决策
     */
    void PlanningFrame();

    void Take();

    // 判断船是不是装够了
    bool IsFull();

    int GetID();

    // 船的货物数
    int GetCaNum() const;

    int GetStatus() const;

    // 船的剩余容量
    int GetCap() const;

    // 设置目标泊位
    void SetTar(int val);

    // 得到目标泊位
    int GetTar() const;

    Position GetPostion() const;

    void AddInstruction(Instruction);    // 增加指令
    void ClearInstruction();             // 清空指令
    Instruction GetOutputInstruction();  // 得到要输出的指令
    void OutputInstructions();           // 输出所有命令
    bool EmptyInstructioins();           // 命令集是否为空

    static ShipPositions GetPostions(Position pos, int dir);
    ShipPositions GetPostions();
    ShipPositions GetPostionsAfterInstruction(Instruction);

    int GetDir() const;

    void SetLoadBerth(int berth_id);  // 设置靠泊装货泊位
    int GetLoadBerth();               // 得到靠泊装货泊位

    Instruction ResToIns(int res);  // 对接借口，将 dij 传的数字变命令

    Instruction CalculateInstruction();  // 计算正常决策下的命令
    // void Move(Instruction);
    int Rot(int dir, int rot);  // 计算旋转后的方向

    void AddValue(int num);  // 增加船携带货物价值
    void ClearValue();       // 清空船携带货物价值
    int GetValue() const;    // 得到船携带货物价值

    std::pair<Position, int> GetNextPD();  // 获得船下一步的方位

    const int id_;      // 船编号
    const int volume_;  // 这一条船的容积

private:
    Position position_;                      // 船当前的位置
    int direction_;                          // 船的朝向
    int capacity_;                           // 这一条船还能装载的容积
    int status_;                             // 0正在移动，1装货或运输完成，2泊位外等待
    int next_berth_{-1};                     // 目标泊位，虚拟点则为-1
    int location_;                           // 船所停靠的泊位 id
    int value_token_{0};                     // 船此时携带的价值
    std::vector<Instruction> instructions_;  // 这一帧需要输出的指令集
};

struct OtherShip {
    int id_;
    int cargo_num_;
    int cargo_change_{0};
    Position position_;
    int direction_;
    int status_;
};

extern int g_other_ship_num;
extern std::vector<Ship> g_ships;
extern std::map<int, OtherShip> g_other_ships;
extern std::map<int, bool> g_ship_owned;
using ShipPositions = std::array<Position, 6>;