#pragma once
#include "config.hpp"
#include "utils.hpp"

/**
 * @brief 泊位的元信息。用于双核异步初始化泊位。
 */
struct BerthMetaData {
    BerthMetaData(int id, Position core_position, int velocity)
        : id_(id), core_position_(core_position), velocity_(velocity) {}
    int id_;
    Position core_position_;
    int velocity_;
};

class Berth {
public:
    friend class Map;

    /**
     * @brief 会自动更新到这个泊位的海上最短路
     * @param id 编号
     * @param pos 核心点的位置
     * @param velocity 装载速度
     * @param area 这个泊位所拥有的点集
     */
    explicit Berth(int id, Position core_pos, int velocity, Grid &&dist_grid, std::set<Position> &&area);

    Berth(const Berth &) = delete;
    Berth &operator=(const Berth &) = delete;

    Berth(Berth &&) noexcept;
    Berth &operator=(Berth &&) = delete;

    ~Berth() = default;

    /**
     * @brief 计算泊位前　num 个货物的价值之和
     * @attention 如果泊位失活，并且还有货物，就让价值为无穷大，吸引船来清空该泊位。
     */
    int CalcValue(int num) const noexcept;

    /**
     * @brief 计算泊位滞留有几个货物
     */
    int GetCargoNum() const noexcept;

    /**
     * @brief 计算泊位上停靠的船的数量
     */
    int GetShipNum() const noexcept;

    /**
     * @brief 给这个泊位增加 x 条船（x 可能为负）
     */
    void AddShipNum(int x) noexcept;

    /**
     * @brief 像泊位中增加 1 个价值为 value 的货物。由 Robot 发出 pull 命令时调用
     */
    void AddCargos(int value) noexcept;

    /**
     * @brief 从泊位中取走 num 个货物，由 ship 取货时调用。
     * @return num and value
     */
    std::pair<int, int> TakeCargo(int num) noexcept;

    const Grid &GetLandDistanceGrid() const noexcept;

    /**
     * @brief 求陆地上点 pos 到泊位的距离
     * @attention 如果泊位失活，直接返回无穷大，让=使得机器人拒绝到这里送货
     */
    int GetLandDistance(Position pos) const noexcept;

    /**
     * @brief 求陆地上点 pos 到泊位所需要前进的方向。
     * @attention 理论上不会用到，已经由 Controller 中的距离增益表替代。
     */
    int GetLandDirection(Position pos) const noexcept;

    /**
     * @brief 判断点 pos 是否在泊位范围内
     */
    bool InBerth(Position pos) const noexcept;

    /**
     * @brief 给出位置、方向，返回下一步走法。
     */
    int GetOceanMarch(Position pos, int dir) const noexcept;

    /**
     * @brief 给出位置、方向，返回距离此泊位步数。
     */
    int GetOceanDistance(Position pos, int dir) const noexcept;

    /**
     * @brief 修改泊位距离
     */
    void ChangeDistance(std::map<Position, int> &pass) noexcept;

    /**
     * @brief 激活泊位，使之可以被货物和船识别到
     */
    void Activate() noexcept;

    /**
     * @brief 失活泊位，使之不可以被货物和船识别到
     */
    void Deactivate() noexcept;

    /**
     * @brief 判断泊位是否处于激活状态
     */
    bool IsActive() const noexcept;

    /**
     * @brief 判断泊位是否有我们的船
     */
    bool HereIsOurShip() const noexcept;

    /**
     * @brief 设定泊位是否有我们的船
     */
    void SetOurShip(bool f) noexcept;

    const int id_;                   // ID.
    const Position core_pos_;        // 核心点的位置
    const int velocity_;             // 装货速度
    const std::set<Position> area_;  // 泊位占据的点集
    const int time_;
    int tot_value = 0;
    int tot_cnt = 0;

private:
    int ship_num_;                         // Number of g_ships.
    std::vector<int> cargo_;               // 所存货物的价值
    Grid dist_grid_;                       // 陆地距离矩阵
    std::array<Grid, 4> ocean_dist_grid_;  // 海上距离矩阵（带方向）
    bool active_{false};                   // 泊位是否被激活
    bool our_ship_{false};                 // 是不是我们的船占据
};

extern std::vector<Berth> g_berths;  // 全局的所有泊位

#define VIRTUAL_BERTH_ID (static_cast<int>(g_berths.size()))

/**
 * @struct 描述一个泊位组
 */
struct BerthGroup {
    BerthGroup() = default;
    BerthGroup(const std::vector<int> &group);
    bool active_{false};  // 该泊位组是否被启用
    bool hasship_;
    std::set<int> group_member_;  // 泊位组内的成员
};

/**
 * @class 管理所有的泊位组
 */
class BerthGroupManager {
public:
    BerthGroupManager() = default;

    /**
     * @brief 是用聚类算法将场上的泊位组分类
     */
    void InitGroups();

    /**
     * @brief 开启一个泊位组
     */
    void ActivateOneGroup(int id);

    /**
     * @brief 返回所有的泊位组
     * @attention 包括已经激活了的和没激活的
     */
    const std::vector<BerthGroup> &GetAllBerthGroup() const;

    /**
     * @brief 返回已经被激活了的泊位组
     */
    std::vector<BerthGroup> GetActivatedBerthGroup() const;

    /**
     * @brief 返回已经激活的泊位组的数量
     */
    int ActivatedSize() const;

private:
    std::vector<BerthGroup> groups_;
};

extern BerthGroupManager g_berth_group_manager;