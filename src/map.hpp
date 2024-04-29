#pragma once

#include "berth.hpp"
#include "config.hpp"
#include "utils.hpp"

/**
 * @struct 描述图中的一个连通块。
 */
struct ConnectedComponent {
    ConnectedComponent() = default;
    ConnectedComponent(const std::set<int> &berth_set,
                       const std::vector<Position> &robot_buy_pos,
                       const std::vector<Position> &ship_buy_pos,
                       int land_area,
                       double berth_ocean_dist);

    // 禁用拷贝构造
    ConnectedComponent(const ConnectedComponent &) = delete;
    ConnectedComponent &operator=(const ConnectedComponent &) = delete;

    // 允许移动构造（需要排序）
    ConnectedComponent(ConnectedComponent &&) = default;
    ConnectedComponent &operator=(ConnectedComponent &&) = default;

    // 排序用的比较函数
    bool operator>(const ConnectedComponent &other) const;

    std::set<int> berth_set_;              // 泊位集合
    std::vector<Position> robot_buy_pos_;  // 机器人购买点
    std::vector<Position> ship_buy_pos_;   // 轮船购买点

    int land_area_{0};            // 陆地面积
    double berth_ocean_dist_{0};  // 泊位距离送货点的平均距离
};


class Map {
public:
    /**
     * @brief 构造函数，主要是初始化 berth_id_grid_ 矩阵为 -1
     */
    Map();

    // 禁用复制
    Map(const Map &) = delete;
    Map &operator=(const Map &) = delete;

    // 禁用移动
    Map(Map &&) = delete;
    Map &operator=(Map &&) = delete;

    // 默认析构
    ~Map() = default;

    /**
     * @brief 在程序开始时，读入地图。
     */
    void Init();

    /**
     * @brief 多源 BFS 最短路
     * @param[in] src 所有的出发点，它们的距离视为 0
     * @param[out] dist_grid 距离矩阵，无法到达的点距离为 INF。
     */
    void MultisourceBFS(const std::vector<Position> &src, Grid &dist_grid, int max_depth = INF) noexcept;

    /**
     * @brief 查找距离位置 pos 最近的泊位
     * @return std::pair 的两个元素分别是；泊位编号和距离
     */
    std::pair<int, int> FindNearestBerth(const Position &pos) const;

    /**
     * @brief 判断位置 pos 是否可以存在机器人
     */
    bool IsField(const Position &pos) const noexcept;

    /**
     * @brief 双核双线程初始化所有的泊位
     */
    void InitAllBerths(std::queue<BerthMetaData> berth_metadata_que);

    /**
     * @brief 用来初始化泊位
     * @param[in] berth_id 需要被初始化的泊位 id
     * @param[in] core_pos 该泊位的核心位置
     * @param[in] velocity 泊位的装货速度
     * @param[out] dist_grid 距离矩阵
     */
    void GenerateBerth(
        int berth_id, const Position &core_pos, int velocity, Grid &dist_grid, std::set<Position> &area);

    /**
     * @brief 寻找图中的连通块，并选择其中之一启用。
     * 连通块：虚拟点——泊位——轮船购买点——机器人购买点。
     */
    void FindConnectedComponent();

    /**
     * @brief 判断点位 pos,dir 是否可以存在船只
     */
    bool ShipInOcean(const Position &pos, int dir) const noexcept;

    /**
     * @brief 判断点 pos 是否在海上
     */
    bool PointInOcean(const Position &pos) const noexcept;

    /**
     * @brief 判断点位 pos,dir 是否为海上主干道
     */
    bool ShipInMainChannel(const Position &pos, int dir) const noexcept;

    /**
     * @brief 判断点 pos 是否在主干道上
     */
    bool IsLandMainLine(const Position &pos) const noexcept;

    /**
     * @brief 判断点 pos 是否在主航道上
     */
    bool PointInMainChannel(const Position &pos) const noexcept;

    /**
     * @brief 海上单源最短路
     */
    void FindOceanShortestPath(const std::vector<Position> &src, std::array<Grid, 4> &ocean_dist_grid) const;

    /**
     * @brief 寻找交货点的海上最短路
     */
    void FindDeleveryShortestPath();

    /**
     * @brief 买一个机器人，直接输出指令
     */
    void BuyRobot(int race, int berth_id);

    /**
     * @brief 买一条船，直接输出指令
     */
    void BuyShip(int berth_id);

    /**
     * @brief 给出位置、方向，返回下一步走法。
     */
    int GetDeliveryPointsMarch(const Position &pos, int dir) const noexcept;

    /**
     * @brief 给出位置、方向，返回距离此泊位步数。
     */
    int GetDeliveryPointsDist(const Position &pos, int dir) const noexcept;

    /**
     * @brief 判断陆地上的一组移动是否会发生碰撞
     * @return 如果会发生碰撞就返回 true
     */
    bool CheckLandCollision(const Position &cur_x,
                            const Position &nxt_x,
                            const Position &cur_y,
                            const Position &nxt_y) const noexcept;

    /**
     * @brief 计算两个坐标的曼哈顿距离
     * @return 曼哈顿距离
     */
    int Manhattan(const Position &x, const Position &y) const noexcept;

    unsigned long long hash_value_;

private:
    std::array<std::array<char, MAP_SIZE>, MAP_SIZE> map_;  // 地图的字符矩阵
    Grid berth_id_grid_;  // 表示每个点属于哪个泊位，如果不是泊位就为 -1；
    std::array<Grid, 4> delivery_point_dist_;  // 表示每个点位离虚拟点的距离
    std::vector<Position> delivery_points_;    // 记录虚拟点坐标
    std::vector<Position> robot_buy_pos_;      // 可以购买机器人的地方
    std::vector<Position> ship_buy_pos_;       // 可以购买船的地方
    ConnectedComponent connnected_component_;  // 选中的连通块
    Grid nearest_berth_id_;                    // 每个位置最近的泊位 id
    Grid nearest_berth_dist_;                  // 每个位置最近的泊位距离
};

extern Map g_map;  // 全局的地图