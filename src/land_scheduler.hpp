#pragma once

#include "cargo.hpp"
#include "config.hpp"
#include "map.hpp"
#include "robot.hpp"


/**
 * @class 封装起来的分层图最大费用最大流。
 */
class ScheduleGraph {
public:
    /**
     * @brief 每一帧都需要重新生成流网络图。
     * @param g_robot_cnt 机器人数量
     * @param cargo_cnt 货物数量
     * @param berth_cnt 泊位数量
     */
    ScheduleGraph(int robot_cnt, int cargo_cnt, int berth_cnt);

    ScheduleGraph(const ScheduleGraph &) = delete;
    ScheduleGraph &operator=(const ScheduleGraph &) = delete;

    ScheduleGraph(ScheduleGraph &&) = delete;
    ScheduleGraph operator=(ScheduleGraph &&) = delete;

    ~ScheduleGraph() = default;

    /**
     * @brief 重新初始化
     */
    void Init(int robot_cnt, int cargo_cnt, int berth_cnt);

    /**
     * @brief 给一组 （机器人，货物，泊位）设定收益
     * @param robot_id 机器人编号
     * @param cargo_id 货物编号
     * @param berth_id 货物需要被送往的泊位编号
     * @param benefit 这组任务的收益
     */
    void SetMisionBenifit(int robot_id, int cargo_id, double benefit) noexcept;

    /**
     * @brief 设置一个货物的泊位。
     * @attention 每个货物只能被设置一次! 否则会检查报错
     * @param cargo_id 货物 id
     * @param berth_id 泊位 id
     */
    void SetCargoBerth(int cargo_id, int berth_id) noexcept;

    /**
     * @brief 查询一组任务的收益
     * @param robot_id 机器人编号
     * @param cargo_id 货物编号
     */
    double GetMissionBenifit(int robot_id, int cargo_id) noexcept;

    /**
     * @brief 给一组泊位增添流量控制（最多有多少个机器人往这里送货）
     * @attention 每个泊位之多只能属于一个组，否则会检查报错。没有设置的组不参与匹配。
     * @param group 表示这个组里有哪些泊位
     * @param limitation 这个组的流量限制
     */
    void SetBerthGroupLimitation(const std::set<int> &berth_group, int limitation) noexcept;

    /**
     * @brief 进行最优匹配。即最大费用最大流算法的主体。
     * @return 返回一个 std::map，里面包含若干组 （机器人，货物）匹配。
     */
    std::map<int, int> MatchRobotCargo() noexcept;

private:
    /**
     * @brief 费用流网络加边。
     * @param 方向为从 u->v，流量为 cap，费用为 cost
     */
    void AddEdge(int u, int v, int cap, double cost) noexcept;

    /**
     * @brief 求解费用流的过程中使用 spfa 算法寻找增广路径
     */
    bool SPFA() noexcept;

    /**
     * @struct 费用流网络里的边
     */
    struct Edge {
        int to_ = 0;
        int cap_ = 0;
        double cost_ = 0;
    };

    /**
     * @brief 图论建模：节点编号从小到大分别为
     *    机器人
     *    货物
     *    泊位
     *    泊位组
     *    源点
     *    汇点
     */
    int GetRobotID(int robot_id) const noexcept;
    int GetCargoID(int cargo_id) const noexcept;
    int GetBerthID(int berth_id) const noexcept;
    int GetBerthGroupID(int berth_group_id) const noexcept;

    int source_{-1};                           // 源点编号
    int sink_{-1};                             // 汇点编号
    int ver_cnt_{0};                           // 点总数
    int robot_cnt_{0};                         // 机器人数量
    int cargo_cnt_{0};                         // 货物数量
    int berth_cnt_{0};                         // 泊位数量
    int berth_group_cnt_{0};                   // 泊位组数量
    std::set<int> already_in_group_;           // 已经在泊位组的泊位编号，debug 用
    std::map<int, int> tmp_group_limitation_;  // 建图时暂存泊位组的流量
    std::set<int> cargo_berth_;                // 记录已经设置目标泊位的货物，debug 用

    std::vector<std::vector<int>> adj_;  // adj[u] 存的是与 u 相连的边编号
    std::vector<Edge> edge_;             // 将边编号映射到 struct Edge，边的编号从 0 开始
    std::vector<double> dis_;            // 从 s 出发，到每个点的最大单位费用
    std::vector<int> incf_;              // 从 s 出发，到每个点的最大流量
    std::vector<int> pre_;               // 可行流当中，每个点的前驱
    // std::map<std::pair<int, int>, double> benefit_matrix_;  // 模拟邻接矩阵
    std::array<std::array<double, MAX_TOT_CARGO_NUM + 200>, MAX_ROBOT_NUM + 10> benefit_matrix_;
};


/**
 * @brief 管理陆地上的虚拟点
 */
class LandVirtualPointManager {
public:
    /**
     * @brief 添加一个陆地上的虚拟点
     */
    void AddLandVirtuaPoint(const Position &pos);
};


class LandScheduler {
public:
    LandScheduler() = default;

    // 禁用所有拷贝和移动
    LandScheduler(const LandScheduler &) = delete;
    LandScheduler &operator=(const LandScheduler &) = delete;
    LandScheduler(LandScheduler &&) = delete;
    LandScheduler operator=(LandScheduler &&) = delete;

    // 默认析构
    ~LandScheduler() = default;

    /**
     * @brief 进行这一帧的路上调度。
     * - 调用 ScheculeBusyRobots() 给所有有货的机器人设置目标泊位
     * - 调用 ScheduleFreeRobots() 给所有无货的机器人设置目标货物
     */
    void ScheduleRobots();

private:
    /**
     * @brief 枚举有货物的机器人，调用 robot.SetTargetBertiID() 设置送货泊位的 ID。
     */
    void ScheduleBusyRobots();

    /**
     * @brief 枚举没有货物的机器人，调用 robot.SetCargoPosition() 设置取货的 Position。
     */
    void ScheduleFreeRobots();

    /**
     * @brief 将货物与泊位匹配
     */
    void MatchCargoBerth() noexcept;

    /**
     * @brief 设置机器人与货物的对应收益
     */
    void CalcRobotCargoBenefit() noexcept;

    /**
     * @brief 利用 RobotCargoGraph 进行匹配，并是适量避免摆头。
     */
    void UpdateMatch();

    /**
     * @brief 给泊位分组
     */
    void DivideBerthGroups();

    /**
     * @brief 判断高价货物是否需要机器人抢占、安排合适的机器人
     */
    void Occupy();

    /**
     * @brief 输入货物指针和一个距离，返回范围内的有空闲容量的机器人数量
     */
    int RoNumAroundCargo(CargoPtr cp, int dis);

    std::array<int, MAX_TOT_CARGO_NUM + 100> cargo_berth_dist_;  // 每个货物到泊位的距离
    std::vector<CargoPtr> cargo_ptr_vec_;                        // 暂存货物指针
    std::shared_ptr<ScheduleGraph> graph_ptr_{
        std::make_shared<ScheduleGraph>(0, 0, 0)};  // 机器人-泊位 匹配图的指针
};

extern LandScheduler g_land_scheduler;