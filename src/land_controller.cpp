#include "land_controller.hpp"


RobotMotionState::RobotMotionState(Position pos, std::map<int, int> dire_val)
    : position_(pos), dire_val_(dire_val) {}

void LandController::ControlRobots() {
#ifdef LOCAL_TIMER
    Timer("LandController::ControlRobots", 2);
#endif
    std::vector<RobotMotionState> motion_states = GetRobotMotionState();
    std::vector<std::vector<int>> groups = GetRobotGroups();

    for (const auto &group : groups) {
#ifdef USE_DFS
        // 如果该组只有一个机器人，直接选择最好的方向
        if (group.size() == 1) {
            int x = group[0];
            std::map<int, int> dire_val_ = motion_states[x].dire_val_;
            assert(!dire_val_.empty());
            auto best = dire_val_.begin();
            for (auto it = dire_val_.begin(); it != dire_val_.end(); it++) {
                if (it->second > best->second) {
                    best = it;
                } else if (it->second == best->second && rand() % 2) {
                    // 加入随机化
                    best = it;
                }
            }
            g_robots[x].SetMoveDirection(best->first);
        }

        // 如果该组有多个机器人，那么调用 AvoidCrash，递归搜索
        if (group.size() >= 2) {
            std::vector<RobotMotionState> group_states;
            for (int id : group) {
                group_states.push_back(motion_states[id]);
            }
            std::vector<int> directions = AvoidCrash(group_states);
            for (int i = 0; i < group.size(); i++) {
                int id = group[i];
                g_robots[id].SetMoveDirection(directions[i]);
            }
        }

#endif

#ifdef DONT_USE_DFS
        for (int i = 0; i < group.size(); i++) {
            int x = group[i];
            std::map<int, int> dire_val_ = motion_states[x].dire_val_;
            assert(dire_val_.size() > 1);
            auto best = dire_val_.begin();
            for (auto it = dire_val_.begin(); it != dire_val_.end(); it++) {
                if (it->second > best->second) {
                    best = it;
                } else if (it->second == best->second && rand() % 2) {
                    // 加入随机化
                    best = it;
                }
            }
            g_robots[x].SetMoveDirection(best->first);
        }
#endif
    }

#ifdef DONT_USE_DFS
    for (int i = 0; i < g_robot_cnt; i++) {
        WorseAvoidCrash(i, 0);
    }
#endif
}

std::map<int, int> LandController::GetDirectionMap(const Robot &robot) {
    std::map<int, int> direction_map;

    // 如果正在等待大模型，那么必须原地不动！
    if (robot.IsWaitingForLLM()) {
        direction_map[4] = 1e7;
        return direction_map;
    }

    // 这个机器人漫无目的，去哪里都行
    if (robot.FullCargo() && robot.GetTargetBerthID() == -1 ||
        !robot.FullCargo() && robot.GetCargoPtr() == nullptr) {
        for (int i = 0; i < 5; i++) {
            Position cur_pos = robot.GetPosition();
            Position nxt_pos = cur_pos + DIR[i];
            if (g_map.IsField(nxt_pos) && !dangerous_position_->Get(nxt_pos)) {
                direction_map[i] = 0;
            }
        }

        return direction_map;
    }

    // 获取这个机器人的目的地的距离矩阵
    auto get_dist_grid = [&]() -> const Grid & {
        if (robot.GetAddtionalTarget() != nullptr) {
            return robot.GetAddtionalTarget()->GetDistanceGrid();
        }
        if (robot.FullCargo()) {
            int berth_id = robot.GetTargetBerthID();
            return g_berths[berth_id].GetLandDistanceGrid();
        } else {
            CargoPtr cargo_ptr = robot.GetCargoPtr();
            return cargo_ptr->GetDistanceGrid();
        }
    };

    // 目标的距离矩阵
    const Grid &distance_grid = get_dist_grid();

    // 临时 BFS 的最大距离，单个机器人搜的面积不会超过 4 * MAX_BFS_DEPTH * MAX_BFS_DEPTH
    const int MAX_BFS_DEPTH = 7;

    auto bfs = [&](Position start_pos, int max_depth) -> int {
        // 记录每个点与机器人之间的距离
        std::map<Position, int> dist_robot;
        dist_robot[start_pos] = 0;

        // 维护 BFS 时的一个状态
        struct BFSState {
            Position position_;
            int dist_;
        };

        std::queue<BFSState> que;
        que.push(BFSState{start_pos, 0});
        int res = INF;

#ifdef LOCAL_TEST
        std::set<Position> st;
#endif

        while (!que.empty()) {
            auto [cur_pos, cur_dist] = que.front();
#ifdef LOCAL_TEST
            if (res == distance_grid.Get(cur_pos)) {
                st.insert(cur_pos);
            } else if (res > distance_grid.Get(cur_pos)) {
                st.clear();
                st.insert(cur_pos);
            }
#endif
            res = std::min(res, distance_grid.Get(cur_pos));
            if (res == 0) {
                return res;
            }
            que.pop();
            if (dist_robot[cur_pos] < cur_dist) {
                continue;
            }

            for (int i = 0; i < 4; i++) {
                Position nxt_pos = cur_pos + DIR[i];
                int nxt_dist = cur_dist + 1;
                if (nxt_dist > max_depth) {
                    continue;
                }
                if (!g_map.IsField(nxt_pos) || dangerous_position_->Get(nxt_pos)) {
                    continue;
                }
                if (!dist_robot.count(nxt_pos) || nxt_dist < dist_robot[nxt_pos]) {
                    dist_robot[nxt_pos] = nxt_dist;
                    que.push({nxt_pos, nxt_dist});
                }
            }
        }

#ifdef LOCAL_TEST
        /**
         * @test 测试 st 中的所有点，到目标的距离都是 res
         */
        for (auto pos : st) {
            assert(distance_grid.Get(pos) == res);
        }
#endif

        return res;
    };

    Position cur_pos = robot.GetPosition();
#ifdef LOCAL
    assert(g_map.IsField(cur_pos));
    assert(direction_map.empty());
#endif
    direction_map[4] = 0;
    int cur_dist = distance_grid.Get(cur_pos);

    for (int i = 0; i < 5; i++) {
        Position nxt_pos = cur_pos + DIR[i];
        if (!g_map.IsField(nxt_pos) || dangerous_position_->Get(nxt_pos)) {
            continue;
        }

        int bfs_depth = (cur_dist <= MAX_BFS_DEPTH + 3 ? 0 : MAX_BFS_DEPTH - 1);
        int benefit = bfs(nxt_pos, bfs_depth);
        if (benefit == 0) {
            benefit = MAX_BFS_DEPTH;
        } else {
            benefit = cur_dist - benefit;
        }
        // direction_map[i] = benefit;
        // 这里使用了有货让无货机制
        direction_map[i] = benefit * (robot.FullCargo() ? 1 : 100);
    }

    for (auto [d, v] : direction_map) {
        if (d != 4) {
            direction_map[d] -= direction_map[4];
        }
    }
    direction_map[4] = 0;

    return direction_map;
}

std::vector<RobotMotionState> LandController::GetRobotMotionState() {
    // 更新 dangerous_position
    dangerous_position_->Set(0);

    for (const auto &[id, other_robot] : g_other_robots) {
#ifdef LOCAL
        for (const auto &robot : g_robots) {
            assert(other_robot.position_ != robot.GetPosition());
        }
#endif
        Position cur_pos = other_robot.position_;

        for (int i = -ROBOT_DANGEROUS_RANGE; i <= ROBOT_DANGEROUS_RANGE; i++) {
            for (int j = -ROBOT_DANGEROUS_RANGE; j <= ROBOT_DANGEROUS_RANGE; j++) {
                Position nxt_pos = cur_pos + Position{i, j};

                if (!g_map.IsLandMainLine(nxt_pos)) {
                    dangerous_position_->Set(nxt_pos, 1);
                }
            }
        }
    }

    std::vector<RobotMotionState> motion_states;
    for (auto &robot : g_robots) {
        std::map<int, int> dire_map = GetDirectionMap(robot);
        motion_states.emplace_back(robot.GetPosition(), dire_map);
    }

    return motion_states;
}

std::vector<std::vector<int>> LandController::GetRobotGroups() {
    DisjointSetUnion dsu(g_robots.size());

    // 判断一个位置是否绝对安全，不需要防碰撞（避免泊位附近主干道的拥堵）
    auto check_safe = [&](const Position &pos) -> bool {
        for (int i = 0; i < 5; i++) {
            const Position nxt_pos = pos + DIR[i];
            if (g_map.IsField(nxt_pos) && !g_map.IsLandMainLine(nxt_pos)) {
                return false;
            }
        }
        return true;
    };

    // 将曼哈顿距离不超过 3 的机器人用并查集两两合并
    for (int i = 0; i < g_robots.size(); i++) {
        const auto &[xi, yi] = g_robots[i].GetPosition();
        if (check_safe(Position{xi, yi})) {
            continue;
        }

        for (int j = i + 1; j < g_robots.size(); j++) {
            const auto &[xj, yj] = g_robots[j].GetPosition();
            if (check_safe(Position{xj, yj})) {
                continue;
            }

            if (std::abs(xi - xj) + std::abs(yi - yj) <= 3) {
                dsu.Merge(i, j);
            }
        }
    }
    return dsu.GetSets();
}

std::vector<int> LandController::AvoidCrash(const std::vector<RobotMotionState> &motion_state_vec) {
    int n = motion_state_vec.size();
    std::vector<int> res_dir(n, 4);
    std::vector<int> cur_dir(n);
    std::vector<Position> cur_pos(n);
    std::vector<Position> nxt_pos(n);
    std::vector<Position> res_nxt_pos(n);

    for (int i = 0; i < n; i++) {
        cur_pos[i] = motion_state_vec[i].position_;
    }
    double best_val = -INF;

    auto dfs = [&](auto self, int id, double cur_val) -> void {
        if (id == n) {
            if (std::count(cur_dir.begin(), cur_dir.end(), 4) != n) {
                if (cur_val > best_val || (cur_val == best_val && rand() % 2)) {
                    best_val = cur_val;
                    res_dir = cur_dir;
                    res_nxt_pos = nxt_pos;
                }
            }
            return;
        }

        for (auto [dir, val] : motion_state_vec[id].dire_val_) {
            nxt_pos[id] = cur_pos[id] + DIR[dir];
            cur_dir[id] = dir;
            bool success = true;
            for (int i = 0; i < id; i++) {
                if (g_map.CheckLandCollision(cur_pos[i], nxt_pos[i], cur_pos[id], nxt_pos[id])) {
                    success = false;
                    break;
                }
            }
            if (success) {
                self(self, id + 1, cur_val + (0.01 * id + 1) * val);
            }
        }
    };

    dfs(dfs, 0, 0);

#ifdef LOCAL
    /**
     * @test 验证运动控制的合法性：不会发生碰撞
     */
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (g_map.CheckLandCollision(cur_pos[i], res_nxt_pos[i], cur_pos[j], res_nxt_pos[j])) {
                fprintf(stderr,
                        "i = %d, j = %d, cur_pos[i]: (%d, %d), nxt_pos[i]: (%d, %d), "
                        "cur_pos[j]: (%d, %d), nxt_pos[j]: (%d, %d)\n",
                        i, j, cur_pos[i][0], cur_pos[i][1], res_nxt_pos[i][0], res_nxt_pos[i][1],
                        cur_pos[j][0], cur_pos[j][1], res_nxt_pos[j][0], res_nxt_pos[j][1]);
                std::cerr << "failed" << std::endl;
            }
            assert(!g_map.CheckLandCollision(cur_pos[i], res_nxt_pos[i], cur_pos[j], res_nxt_pos[j]));
        }
    }
    // assert(std::count(res_dir.begin(), res_dir.end(), 4) != n);
#endif

    return res_dir;
}


void LandController::WorseAvoidCrash(int i, int c) {
    if (c > 30) {
        return;
    }
    const int maxrc = 30;
    std::array<Position, maxrc> cur_pos;
    std::array<Position, maxrc> nxt_pos;
    std::array<Position, maxrc> direction;

    for (int k = 0; k < g_robot_cnt; k++) {
        cur_pos[k] = g_robots[k].GetPosition();
    }
    for (int j = 0; j < g_robot_cnt; j++) {
        if (j == i) {
            continue;
        }

        for (int k = 0; k < g_robot_cnt; k++) {
            direction[k] = DIR[g_robots[k].GetMoveDirection()];
            nxt_pos[k] = cur_pos[k] + direction[k];
        }

        // j 追尾 i
        if (nxt_pos[j] == cur_pos[i] && nxt_pos[i] == cur_pos[i]) {
            bool success = false;
            for (int d = 0; d < 4; d++) {
                if (d == g_robots[i].GetMoveDirection()) {
                    continue;
                }
                auto tmp_pos = cur_pos[j] + DIR[d];
                if (g_map.IsField(tmp_pos) && tmp_pos != cur_pos[i] && tmp_pos != nxt_pos[i]) {
                    g_robots[j].SetMoveDirection(d);
                    success = true;
                    break;
                }
            }

            // 否则stop
            if (!success) {
                g_robots[j].SetMoveDirection(4);
                // g_robots[j].SetMoveDirection(g_robots[j].GetMoveDirection()^1);
            }
            WorseAvoidCrash(j, c + 1);
            continue;
        }

        // 挨在一起迎面撞，j 让出身位，i 等待
        if (nxt_pos[i] == cur_pos[j] && nxt_pos[j] == cur_pos[i]) {
            // 先尝试向侧边避让
            bool success = false;
            for (int d = 0; d < 4; d++) {
                if (d == g_robots[i].GetMoveDirection()) {
                    continue;
                }
                auto tmp_pos = cur_pos[j] + DIR[d];
                if (g_map.IsField(tmp_pos) && tmp_pos != cur_pos[i] && tmp_pos != nxt_pos[i]) {
                    g_robots[j].SetMoveDirection(d);
                    success = true;
                    break;
                }
            }
            // 否则向后退让
            if (!success) {
                g_robots[j].SetMoveDirection(g_robots[i].GetMoveDirection());
            }
            WorseAvoidCrash(j, c + 1);
            continue;
        }

        // i 追 j
        if (nxt_pos[i] == cur_pos[j] && nxt_pos[j] != cur_pos[j]) {
            WorseAvoidCrash(j, c + 1);
            continue;
        }
    }

    for (int j = 0; j < g_robot_cnt; j++) {
        if (j == i) {
            continue;
        }

        for (int k = 0; k < g_robot_cnt; k++) {
            direction[k] = DIR[g_robots[k].GetMoveDirection()];
            nxt_pos[k] = cur_pos[k] + direction[k];
        }

        // 直角撞，或迎面间隔撞，j 等 i 先走
        if (nxt_pos[i] == nxt_pos[j]) {
            g_robots[j].SetMoveDirection(4);
            WorseAvoidCrash(j, c + 1);
            continue;
        }
    }
}

LandController g_land_controller;