#pragma once

#include "cargo.hpp"
#include "config.hpp"
#include "map.hpp"
#include "robot.hpp"
#include "ship.hpp"
#include "utils.hpp"


class ShipPlanMaker {
public:
    ShipPlanMaker();                                                // 默认加入全部泊位
    ShipPlanMaker(int ship_id);                                     // 加入全部泊位
    ShipPlanMaker(int ship_id, std::vector<int> berths_id_vector);  // 加入部分泊位

    /**
     * @brief 随机生成一个泊位序号排列，作为一种船调度路线
     */
    void RandomGlobal();

    /**
     * @brief 根据泊位数量等信息对一个排列作随机化调整
     */
    void RandomLocal();

    /**
     * @brief 对一个排列计算收益
     */
    double CalcValue();

    /**
     * @brief 计算100个随机排列中的最优情况
     */
    void GetLocalBest();

    /**
     * @brief 对一个排列作随机化调整
     */
    void GetGlobalBest();

    /**
     * @brief 生成随机序列并判断价值最大
     */
    void GetGlobalBestTSP();

    /**
     * @brief 枚举全排列，拣选权值最大的
     */
    void GetGlobalBestForce();

    /**
     * @brief 根据拣选出的最佳排列得到下一个泊位的序号
     * @return 最佳排列中的第一个序号
     */
    int GetNextBerth();

    /**
     * @brief 清除现存排列内容
     */
    void Clear();

    /**
     * @brief 重载泊位id
     */
    void ResetBerthIDs(std::vector<int> berths_id_vector);

    /**
     * @brief 打印调试信息
     */
    void Print();

private:
    std::vector<int> permutation_;  // 一个排列，表示船对泊位的一组访问顺序
    std::vector<int> berths_id_;    // 泊位id序列
    int ship_id_;                   // 此调度器负责的船的id
};

class ShipsPlanMaker {
public:
    /**
     * @brief 为整个系统添加一艘船
     */
    void AddShip();

    /**
     * @brief 为一艘船安排调度
     * @param ship_id 请求调度的船的id
     */
    int GetNextBerth(int ship_id);

    /**
     * @brief 为一艘船安排负责的泊位组
     * @param ship_id 需设置泊位组的船的id
     */
    void SetBerthSet(int ship_id);

    /**
     * @brief 获得已加入调度系统的船数量
     */
    int GetShipNum();

    /**
     * @brief 为一艘船安排目标泊位（试行版）
     */
    int GetNextBerthTemp(int ship_id);

    /**
     * @brief 得到一个泊位组的货物数量
     * @param set_id 泊位组id
     */
    int GetBerthSetCargoNum(int set_id);

    /**
     * @brief 输出调试信息
     */
    void Print();

private:
    /**
     * @brief 为一艘船安排调度
     * @param step 当前调度轮进行到的步数
     * @param set_id 泊位组id
     */
    int StepToBerthID(int step, int set_id);

    std::vector<int> ship_berth_set_;  // 船-泊位组匹配关系
    std::vector<int> step_;            // 调度轮步数
    std::vector<int> used_set_;        // 已用集合
};

class OceanScheduler {
public:
    OceanScheduler() = default;

    OceanScheduler(const OceanScheduler &) = delete;
    OceanScheduler &operator=(const OceanScheduler &) = delete;

    OceanScheduler(OceanScheduler &&) = delete;
    OceanScheduler operator=(OceanScheduler &&) = delete;

    /**
     * @brief 开启调度
     */
    void ScheduleShips();

    /**
     * @brief 开启调度（试行版）
     */
    void ScheduleShipsTemp();

private:
    /**
     * @brief 为全部搜索一套最佳调度
     * @param now 目前价值
     * @param j 目前搜索深度
     * @param sum 最佳价值
     * @param tmp 目前匹配方案
     * @param res 最佳匹配方案
     */
    void ShipDFS(double now, int j, double &sum, std::vector<int> &tmp, std::vector<int> &res);
};

extern OceanScheduler g_ocean_scheduler;
extern ShipsPlanMaker sspm;