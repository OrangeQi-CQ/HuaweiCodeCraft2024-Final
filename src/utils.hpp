#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <stack>
#include <thread>
#include <unordered_set>
#include <vector>

#include "config.hpp"

/**
 * @class 并查集
 */
class DisjointSetUnion {
public:
    /**
     * @brief 初始化并查集
     * @param n 元素个数，下标从 [0, n-1]
     */
    explicit DisjointSetUnion(int n);

    ~DisjointSetUnion() = default;

    /**
     * @brief 合并两个元素所在的集合
     * @return 原本在一个集合返回 false，否则返回 true
     */
    bool Merge(int x, int y);

    /**
     * @brief 寻找所在集合的代表
     */
    int FindLeader(int x);

    /**
     * @brief 输出并查集
     * @return 外层的 vector 表示一个集合，内层的 vector 表示这个集合里面有哪些元素。
     */
    std::vector<std::vector<int>> GetSets();

private:
    int n_;
    std::vector<int> leader_;
};

/**
 * @class Position 用来表示地图上的一个点
 */
using Position = std::array<int, 2>;

using Direction = Position;

std::ostream &operator<<(std::ostream &os, const Position &pos);

Position operator+(const Position &lhs, const Position &rhs);

Position operator-(const Position &lhs, const Position &rhs);

/**
 * @brief 获取两个 Position 之间的曼哈顿距离
 */
int GetL1(const Position &lhs, const Position &rhs);

/**
 * @brief 五个方向
 */
const int DX[5]{0, 0, -1, 1, 0};
const int DY[5]{1, -1, 0, 0, 0};
const Direction DIR[5]{Direction{0, 1}, Direction{0, -1}, Direction{-1, 0}, Direction{1, 0}, Direction{0, 0}};

/**
 * @class TwoDimArray 是一个简单的二维矩阵
 * @attention 禁止了复制，但是允许移动：利用 std::vector 的移动构造，复杂度 O(1)。
 */
template <typename T, int N, int M>
class TwoDimArray {
public:
    TwoDimArray(T w = 0) {
        // std::cerr << "Grid Construct!" << std::endl;
        Set(w);
    }

    // 禁止拷贝构造
    TwoDimArray(const TwoDimArray &) = delete;
    TwoDimArray &operator=(const TwoDimArray &) = delete;

    TwoDimArray(TwoDimArray &&other) noexcept {
        grid_ = std::move(other.grid_);
    }

    TwoDimArray &operator=(TwoDimArray &&other) noexcept {
        grid_ = std::move(other.grid_);
        return *this;
    }

    // 默认析构
    ~TwoDimArray() = default;

    /**
     * @brief 单点赋值
     */
    void Set(const Position &pos, T w) {
        const auto &[x, y] = pos;
        grid_[x][y] = w;
    }

    /**
     * @brief 全局赋值
     */
    void Set(T w) {
        grid_.assign(N, std::vector<T>(M, w));
    }

    /**
     * @brief 单点取值
     */
    T Get(const Position &pos) const {
        const auto &[x, y] = pos;
        return grid_.at(x).at(y);
    }

    /**
     * @brief 获得梯度最大的方向
     * @attention 自带随机性
     */
    int GetDescentDirection(const Position &pos) const {
        int dir = 0;
        int minv = INF;
        std::vector<int> cho;
        for (int i = 0; i < 5; i++) {
            Position nxt = pos + Direction{DX[i], DY[i]};
            if (!IN_MAP(nxt[0], nxt[1])) {
                continue;
            }

            if (Get(nxt) < minv) {
                minv = Get(nxt);
                cho.clear();
                cho.push_back(i);
            } else if (Get(nxt) == minv) {
                cho.push_back(i);
            }
        }

        dir = cho[rand() % cho.size()];
        return dir;
    }

    auto &operator[](int index) {
        return grid_[index];
    }

    auto begin() {  // NOLINT
        return grid_.begin();
    }

    auto end() {  // NOLINT
        return grid_.end();
    }

private:
    std::vector<std::vector<T>> grid_;
};

/**
 * @class Grid 针对并发的偏特化类型
 */
template <int N, int M>
class TwoDimArray<std::atomic_int, N, M> {
public:
    TwoDimArray(std::atomic_int w = 0) {
        Set(w.load());
    }

    // 禁止拷贝构造
    TwoDimArray(const TwoDimArray &) = delete;
    TwoDimArray &operator=(const TwoDimArray &) = delete;

    // 禁止移动构造
    TwoDimArray(TwoDimArray &&other) = delete;
    TwoDimArray &operator=(TwoDimArray &&other) = delete;

    /**
     * @brief 单点赋值
     */
    void Set(const Position &pos, std::atomic_int w) {
        const auto &[x, y] = pos;
        std::atomic_store(&grid_[x][y], w.load());
    }

    /**
     * @brief 全局赋值
     */
    void Set(std::atomic_int w) {
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < M; j++) {
                std::atomic_store(&grid_[i][j], w.load());
            }
        }
    }

    /**
     * @brief 单点取值
     */
    std::atomic_int Get(const Position &pos) const {
        const auto &[x, y] = pos;
        return grid_[x][y].load();
    }

    auto &operator[](int index) {
        return grid_[index];
    }

    auto begin() {  // NOLINT
        return grid_.begin();
    }

    auto end() {  // NOLINT
        return grid_.end();
    }

private:
    std::array<std::array<std::atomic_int, M>, N> grid_;
};

using Grid = TwoDimArray<int, MAP_SIZE, MAP_SIZE>;
using ConcurrencyGrid = TwoDimArray<std::atomic_int, MAP_SIZE, MAP_SIZE>;

/**
 * @brief 专门用来管理 Grid 内存池
 */
class GridPool {
public:
    using GridPtr = std::shared_ptr<Grid>;

    /**
     * @brief 单例模式
     */
    static GridPool &GetInstance() {
        static GridPool grid_pool;
        return grid_pool;
    }

    /**
     * @brief 申请一个 GridPtr
     */
    GridPtr Alloc() {
        std::lock_guard lg(mtx_);
        assert(!pool_.empty());
        auto it = *(pool_.begin());
        pool_.erase(it);
        return it;
    }

    /**
     * @brief 释放一个 GridPtr
     */
    void Free(GridPtr ptr) {
        std::lock_guard lg(mtx_);
        pool_.insert(ptr);
    }

private:
    GridPool() {
        for (int i = 0; i < MAX_TOT_CARGO_NUM + 50; i++) {
            pool_.insert(std::make_shared<Grid>());
        }
    }

    std::mutex mtx_;
    std::set<GridPtr> pool_;
};

/**
 * @class 用来打印日志到文件
 */
class Logger {
public:
    Logger(const std::string &entity_name) : entity_name_(entity_name) {
        log_file_.open("./log/" + GetCurrentTime() + entity_name_ + ".log", std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            std::cerr << "Error: Unable to open log file for entity " << entity_name_ << std::endl;
        }
    }

    template <typename T>
    void log(const T &arg) {  // NOLINT
        log_file_ << " " << arg << std::endl;
    }

    template <typename T, typename... Args>
    void log(const T &firstArg, Args... args) {  // NOLINT
        log_file_ << " " << firstArg;
        log(args...);
    }

    ~Logger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    std::string entity_name_;
    std::ofstream log_file_;

    std::string GetCurrentTime() {
        time_t now = time(0);
        struct tm tstruct;
        char buf[80];
        tstruct = *localtime(&now);
        strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
        return buf;
    }
};

/**
 * @brief 计时器，用来记录一个作用域的执行耗时
 */
class Timer {
public:
    Timer(std::string name) : name_(name), start_time_(std::chrono::steady_clock::now()) {}
    Timer(std::string name, int worst)
        : name_(name), worst_(worst), start_time_(std::chrono::steady_clock::now()) {}

    void Check(std::string name, int worst) {
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_);

        std::chrono::milliseconds tmp_worst(worst);
        if (worst_ == std::chrono::milliseconds(INF) || duration > tmp_worst) {
            std::cerr << "\033[31m" << name << " took \t" << duration.count()
                      << " milliseconds to run.\033[0m" << std::endl;
        }
    }

    ~Timer() {
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_);
        if (worst_ == std::chrono::milliseconds(INF) || duration > worst_) {
            std::cerr << "\033[31m" << name_ << " took \t" << duration.count()
                      << " milliseconds to run.\033[0m" << std::endl;
        }
    }

private:
    std::string name_;
    std::chrono::milliseconds worst_{INF};
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

extern Logger crashLog;