#pragma once

#include <optional>

#include "config.hpp"
#include "map.hpp"
#include "robot.hpp"
#include "utils.hpp"

struct CargoMetadata {
    CargoMetadata(Position pos, int value, int appear_frame, int dist_to_berth)
        : position_(pos), value_(value), appear_frame_(appear_frame), dist_to_berth_(dist_to_berth) {}
    Position position_;
    int value_;
    int appear_frame_;
    int dist_to_berth_;
};

class Cargo {
public:
    friend class CargoManager;

    explicit Cargo(Position pos, int value, int appear_frame, std::shared_ptr<Grid> dist_gird_ptr);

    Cargo(const Cargo &) = delete;
    Cargo &operator=(const Cargo &) = delete;

    Cargo(Cargo &&) noexcept;
    Cargo &operator=(Cargo &&) = delete;

    ~Cargo();

    /**
     * @brief 判断这个货物是否为虚拟的
     */
    bool IsVirtual() const noexcept;

    /**
     * @brief 查询货物编号
     */
    int GetID() const noexcept;

    /**
     * @brief 设置货物编号
     */
    void SetID(int id) noexcept;

    /**
     * @brief 获取距离矩阵
     */
    const Grid &GetDistanceGrid() const noexcept;

    /**
     * @brief 查询 pos 距离该货物的距离
     */
    int GetDistance(Position pos) const noexcept;

    /**
     * @brief 查询 pos 前往该货物的方向
     */
    int GetDirection(Position pos) const noexcept;

    /**
     * @brief 查询这个货物的价值
     */
    int GetValue() const noexcept;

    /**
     * @brief 机器人将货物拿走
    */
    void Delete() noexcept;

    /**
     * @brief 判断这个货物是否已经被
    */
    bool IsDeleted() const noexcept;

    /**
     * @brief 判断这个货物还能继续存在的时间
     */
    int GetRemainingLife() const noexcept;

    /**
     * @brief 判断这个货物上面是否有其他队机器人
     */
    bool IsOccupied() const noexcept;

    /**
     * @brief 设定这个货物上面是否有其他队机器人
     */
    void SetIsOccupied(bool be) noexcept;

    const int value_;          // 货物的价值
    const int appear_frame_;   // 货物出现的时间
    const Position position_;  // 货物的位置

private:
    int id_;                               // 货物的编号
    bool delete_{false};                   // 货物是否已经被机器人取走
    std::shared_ptr<Grid> dist_grid_ptr_;  // 货物的距离矩阵
    bool is_occupied_;                     // 是否被其他队伍占领
    bool is_virtual_{false};               // 是否为虚拟货物
};

using CargoPtr = std::shared_ptr<Cargo>;

class CargoManager {
public:
    friend void Control();
    friend void *CargoManagerThreadFunc(void *arg);

    CargoManager();

    CargoManager(const CargoManager &) = delete;
    CargoManager &operator=(const CargoManager &) = delete;

    CargoManager(CargoManager &&) = delete;
    CargoManager &operator=(CargoManager &&) = delete;

    ~CargoManager();

    /**
     * @brief 新增一个虚拟货物
     */
    void AddVirtualCargo(const Position &position, std::shared_ptr<Grid> distance_grid_ptr);
    /**
     * @brief 启动异步的搜索
     */
    void StartAsynTask();

    /**
     * @brief 在一帧当中，读入新增的货物。
     */
    void InputFrame();

    /**
     * @brief 在一帧当中，尝试更细腻 task_list 和 ready_queue
     */
    void UpdateAsynTaskList();

    /**
     * @brief 在一帧当中，更新货物，检查现有的货物是否合法。
     */
    void RefreshCargos();

    /**
     * @brief 将所有货物的指针拷贝一份。
     * @attention 包括真实货物和虚拟货物
     */
    std::vector<CargoPtr> GetAllCargoPtr() const;

    static int tot_bfs_cnt_;
    static int tot_erase_cnt_;

private:
    /**
     * @brief 在另一个 CPU 上进行货物的 bfs
     */
    void InitCargoDistGrid();

    std::vector<CargoPtr> cargo_ptr_vec_;              // 所有的货物。包括真实货物和虚拟货物
    std::queue<std::array<int, 4>> cargo_join_queue_;  // 货物缓存

    std::list<CargoMetadata> task_list_;  // 任务链表
    std::mutex task_list_mtx_;            // 任务队列的锁
    std::queue<CargoPtr> ready_queue_;    // 就绪队列
    std::mutex ready_queue_mtx_;          // 就绪队列的锁
    std::atomic_bool cargo_vec_full_;     // cargo_ptr_vec_ 的容量是否过大

    ConcurrencyGrid cargo_exist_grid_;             // 表示某个地方是否存在货物
    std::vector<std::thread> cargo_init_threads_;  // 用来给货物搜索路径的线程池
};

extern CargoManager g_cargo_manager;