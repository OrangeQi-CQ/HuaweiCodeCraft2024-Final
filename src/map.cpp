#include "map.hpp"

#include <optional>

ConnectedComponent::ConnectedComponent(const std::set<int> &berth_set,
                                       const std::vector<Position> &robot_buy_pos,
                                       const std::vector<Position> &ship_buy_pos,
                                       int land_area,
                                       double berth_ocean_dist)
    : berth_set_(berth_set),
      robot_buy_pos_(robot_buy_pos),
      ship_buy_pos_(ship_buy_pos),
      land_area_(land_area),
      berth_ocean_dist_(berth_ocean_dist) {}

bool ConnectedComponent::operator>(const ConnectedComponent &other) const {
    if (this->land_area_ != other.land_area_) {
        return this->land_area_ > other.land_area_;
    }
    return this->berth_ocean_dist_ < other.berth_ocean_dist_;
}

Map::Map() {
    berth_id_grid_.Set(-1);
}

void Map::Init() {
    for (int i = 0; i < MAP_SIZE; i++) {
        std::string line;
        std::cin >> line;

        for (int j = 0; j < MAP_SIZE; j++) {
            map_[i][j] = line[j];

            if (map_[i][j] == 'T') {
                delivery_points_.push_back(Position{i, j});
            }
            if (map_[i][j] == 'R') {
                robot_buy_pos_.push_back(Position{i, j});
            }
            if (map_[i][j] == 'S') {
                ship_buy_pos_.push_back(Position{i, j});
            }
        }

        std::string ok;
        std::cin >> ok;
        assert(ok == "OK");
        std::cout << ok << std::endl;
    }

    FindDeleveryShortestPath();

    unsigned long long hash = 0;
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            hash = (hash * 377 + map_[i][j]);
        }
    }

    hash_value_ = hash % 37;
}

void Map::InitAllBerths(std::queue<BerthMetaData> berth_metadata_que) {
    auto start = std::chrono::steady_clock::now();

    std::mutex metadata_mtx;
    std::vector<std::optional<Berth>> tmp_berths(berth_metadata_que.size());

    // 双核并行初始化所有泊位
    auto init_function = [&]() -> void {
        while (true) {
            std::optional<BerthMetaData> berth_metadata;
            {
                std::lock_guard lg(metadata_mtx);
                if (berth_metadata_que.empty()) {
                    return;
                }
                berth_metadata = berth_metadata_que.front();
                berth_metadata_que.pop();
            }

            Grid dist_grid;
            std::set<Position> area;
            auto [id, pos, vel] = berth_metadata.value();
            GenerateBerth(id, pos, vel, dist_grid, area);
            tmp_berths[id].emplace(id, pos, vel, std::move(dist_grid), std::move(area));
            {
                std::lock_guard lg(cerr_mtx);
                fprintf(stderr, "finish init %d", id);
                std::cerr << std::endl;
            }
        }
    };

    // 两个线程分别绑定到两个 CPU 上
    std::thread init_thread0(init_function);
    std::thread init_thread1(init_function);

    pthread_t handle0 = init_thread0.native_handle();
    pthread_t handle1 = init_thread1.native_handle();

#ifdef LOCAL
    fprintf(stderr, "handle0 = %ld\n", handle0);
    fprintf(stderr, "handle1 = %ld\n", handle1);
#endif

#if !defined(_WIN32) && !defined(_WIN64)
    cpu_set_t cpu_set0;
    CPU_ZERO(&cpu_set0);
    CPU_SET(0, &cpu_set0);
    pthread_setaffinity_np(handle0, sizeof(cpu_set0), &cpu_set0);

    cpu_set_t cpu_set1;
    CPU_ZERO(&cpu_set1);
    CPU_SET(1, &cpu_set1);
    pthread_setaffinity_np(handle1, sizeof(cpu_set1), &cpu_set1);
#endif

    init_thread0.join();
    init_thread1.join();

    for (auto &it : tmp_berths) {
        g_berths.emplace_back(std::move(it.value()));
    }

    nearest_berth_dist_.Set(INF);

    // 初始化每个点到泊位的距离
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            for (int k = 0; k < g_berths.size(); k++) {
                int dist = g_berths[k].dist_grid_.Get(Position{i, j});
                if (dist < nearest_berth_dist_[i][j]) {
                    nearest_berth_id_[i][j] = k;
                    nearest_berth_dist_[i][j] = dist;
                }
            }
        }
    }

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cerr << "init berth using " << duration.count() << " ms\n";
}

void Map::GenerateBerth(
    int berth_id, const Position &core_pos, int velocity, Grid &dist_grid, std::set<Position> &area) {
    std::queue<Position> que;  // 广义泊位
    que.push(core_pos);
    berth_id_grid_.Set(core_pos, berth_id);  // 广义泊位
    std::vector<Position> src{core_pos};     // 狭义泊位

    while (!que.empty()) {
        Position cur_pos = que.front();
        que.pop();
        if (area.count(cur_pos)) continue;
        area.insert(cur_pos);

        for (int i = 0; i < 4; i++) {
            Position nxt_pos = cur_pos + DIR[i];

            if (IN_MAP(nxt_pos[0], nxt_pos[1]) && berth_id_grid_.Get(nxt_pos) == -1 && !area.count(nxt_pos)) {
                char type = map_[nxt_pos[0]][nxt_pos[1]];
                if (type == 'B' || type == 'K') {
                    que.push(nxt_pos);
                    berth_id_grid_.Set(nxt_pos, berth_id);
                }
                if (type == 'B') {
                    src.push_back(nxt_pos);
                }
            }
        }
    }

    MultisourceBFS(std::move(src), dist_grid, BERTH_BFS_MAX_DEPTH);
}

void Map::MultisourceBFS(const std::vector<Position> &src, Grid &dist_grid, int max_depth) noexcept {
    dist_grid.Set(INF);
    std::queue<std::array<int, 3>> que;

    for (const auto &[x, y] : src) {
        if (!IsField({x, y})) {
            continue;
        }

        dist_grid[x][y] = 0;
        que.push(std::array<int, 3>{0, x, y});
    }

    while (!que.empty()) {
        const auto [d, x, y] = que.front();
        que.pop();
        if (d > dist_grid[x][y]) {
            continue;
        }

        for (int i = 0; i < 4; i++) {
            int xx = x + DX[i];
            int yy = y + DY[i];
            int dist = dist_grid[x][y] + 1;

            if (IsField({xx, yy})) {
                if (dist_grid[xx][yy] > dist && dist <= max_depth) {
                    dist_grid[xx][yy] = dist;
                    que.push(std::array<int, 3>{dist, xx, yy});
                }
            }
        }
    }
}

std::pair<int, int> Map::FindNearestBerth(const Position &pos) const {
    return std::pair<int, int>{nearest_berth_id_.Get(pos), nearest_berth_dist_.Get(pos)};
}

void Map::BuyRobot(int race, int berth_id) {
    static int idx = 0;
    int sz = connnected_component_.robot_buy_pos_.size();
    idx = rand() % sz;
    auto [x, y] = connnected_component_.robot_buy_pos_[idx];
    std::cout << "lbot " << x << " " << y << ' ' << race << "\n";
}

void Map::BuyShip(int berth_id) {
    // static int idx = 0;
    // int sz = connnected_component_.ship_buy_pos_.size();
    // idx = (idx + 1) % sz;
    g_berth_group_manager.ActivateOneGroup(ship_tar_hash[g_ship_cnt]);
    Position p = {-1, -1};
    for (auto pos : ship_buy_pos_) {
        if (p == Position{-1, -1} ||
            g_berths[berth_id].GetOceanDistance(pos, -1) < g_berths[berth_id].GetOceanDistance(p, -1)) {
            p = pos;
        }
    }
    auto [x, y] = p;
    // auto [x, y] = connnected_component_.ship_buy_pos_[idx];
    std::cout << "lboat " << x << " " << y << "\n";
}

void Map::FindConnectedComponent() {
    // 先把泊位都激活，方便计算距离
    for (auto &berth : g_berths) {
        berth.Activate();
    }

    // 枚举所有的买船点和泊位，用并查集维护其连通性
    DisjointSetUnion dsu(ship_buy_pos_.size() + g_berths.size());
    for (int i = 0; i < ship_buy_pos_.size(); i++) {
        for (int j = 0; j < g_berths.size(); j++) {
            if (g_berths[j].GetOceanDistance(ship_buy_pos_[i], -1) < 1e5) {
                dsu.Merge(i, ship_buy_pos_.size() + j);
            }
        }
    }

    auto connected_block = dsu.GetSets();
    std::vector<ConnectedComponent> connected_components;

    // 选择一个最佳的连通块
    for (auto &block : connected_block) {
        // 记录；这个连通块里面有哪些泊位
        std::set<int> berth_set;
        std::vector<Position> ship_buy_pos;
        for (int x : block) {
            if (x >= ship_buy_pos_.size()) {
                berth_set.insert(x - ship_buy_pos_.size());
            } else {
                ship_buy_pos.push_back(ship_buy_pos_[x]);
            }
        }

        // 如果这个连通块没有买船点或者泊位，就跳过
        if (ship_buy_pos.empty() || berth_set.empty()) {
            continue;
        }

        // 如果与送货点不连通，就跳过
        if (GetDeliveryPointsDist(ship_buy_pos[0], 0) > 1e5) {
            continue;
        }

        // BFS 计算这个连通块的陆地面积
        Grid visit;
        visit.Set(0);
        std::queue<Position> que;
        que.push(robot_buy_pos_[0]);
        int land_area = 0;

        while (!que.empty()) {
            auto cur_pos = que.front();
            visit.Set(cur_pos, 1);
            land_area += 1;
            que.pop();
            for (int i = 0; i < 5; i++) {
                Position nxt_pos = cur_pos + DIR[i];
                if (!IsField(nxt_pos) || visit.Get(nxt_pos) == 1) {
                    continue;
                }
                que.push(nxt_pos);
                visit.Set(nxt_pos, 1);
            }
        }

        // 计算泊位到送货点的平均距离
        double berth_ocean_dist = 0;
        for (int berth_id : berth_set) {
            Position core_pos = g_berths[berth_id].core_pos_;
            berth_ocean_dist += GetDeliveryPointsDist(core_pos, -1);
        }
        berth_ocean_dist /= berth_set.size();
        connected_components.emplace_back(berth_set, robot_buy_pos_, ship_buy_pos, land_area,
                                          berth_ocean_dist);
    }

    // 选择最优连通块，并启动相应泊位
    std::sort(connected_components.begin(), connected_components.end(), std::greater<ConnectedComponent>());
    this->connnected_component_ = std::move(connected_components[0]);

    // 将连通块以外的泊位全都失活
    for (int i = 0; i < g_berths.size(); i++) {
        if (connnected_component_.berth_set_.count(i) == 0) {
            g_berths[i].Deactivate();
        }
    }
}

bool Map::IsField(const Position &pos) const noexcept {
    const auto &[x, y] = pos;
    const char &c = map_[x][y];
    return IN_MAP(x, y) && c != '*' && c != '#' && c != '~' && c != 'S' && c != 'K' && c != 'T';
}

bool Map::IsLandMainLine(const Position &pos) const noexcept {
    const auto &[x, y] = pos;
    const char &c = map_[x][y];
    return IN_MAP(x, y) && (c == '>' || c == 'R' || c == 'c' || c == 'B');
}

bool Map::PointInOcean(const Position &pos) const noexcept {
    const auto &[x, y] = pos;
    const char &c = map_[x][y];
    return IN_MAP(x, y) && c != '#' && c != '.' && c != '>' && c != 'R';
}

bool Map::PointInMainChannel(const Position &pos) const noexcept {
    const auto &[x, y] = pos;
    const char &c = map_[x][y];
    return IN_MAP(x, y) && (c == '~' || c == 'S' || c == 'c' || c == 'B' || c == 'K');
}

bool Map::ShipInOcean(const Position &pos, int dir) const noexcept {
    bool in = true;
    switch (dir) {
        case 0:
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 3; j++) {
                    in &= PointInOcean(pos + Position{i, j});
                }
            }
            break;

        case 1:
            for (int i = -1; i < 1; i++) {
                for (int j = -2; j < 1; j++) {
                    in &= PointInOcean(pos + Position{i, j});
                }
            }
            break;

        case 2:
            for (int i = -2; i < 1; i++) {
                for (int j = 0; j < 2; j++) {
                    in &= PointInOcean(pos + Position{i, j});
                }
            }
            break;

        case 3:
            for (int i = 0; i < 3; i++) {
                for (int j = -1; j < 1; j++) {
                    in &= PointInOcean(pos + Position{i, j});
                }
            }
            break;
    }
    return in;
}

bool Map::ShipInMainChannel(const Position &pos, int dir) const noexcept {
    bool in = false;
    switch (dir) {
        case 0:
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 3; j++) {
                    in |= PointInMainChannel(pos + Position{i, j});
                }
            }
            break;

        case 1:
            for (int i = -1; i < 1; i++) {
                for (int j = -2; j < 1; j++) {
                    in |= PointInMainChannel(pos + Position{i, j});
                }
            }
            break;

        case 2:
            for (int i = -2; i < 1; i++) {
                for (int j = 0; j < 2; j++) {
                    in |= PointInMainChannel(pos + Position{i, j});
                }
            }
            break;

        case 3:
            for (int i = 0; i < 3; i++) {
                for (int j = -1; j < 1; j++) {
                    in |= PointInMainChannel(pos + Position{i, j});
                }
            }
            break;
    }
    return in;
}

void Map::FindOceanShortestPath(const std::vector<Position> &src,
                                std::array<Grid, 4> &ocean_dist_grid) const {
    // 直接跑 dijkstra 最短路
    for (int i = 0; i < 4; i++) {
        ocean_dist_grid[i].Set(INF);
    }
    std::priority_queue<std::pair<int, std::pair<Position, int>>> q;

    for (const auto &i : src) {
        for (int j = 0; j < 4; j++) {
            if (ShipInOcean(i, j)) {
                ocean_dist_grid[j].Set(i, 0);
                q.push({0, {i, j}});
            }
        }
    }

    while (!q.empty()) {
        auto [dis, pad] = q.top();
        dis = -dis;
        const auto [p, d] = pad;
        q.pop();

        if (ocean_dist_grid[d].Get(p) < dis) {
            continue;
        }

        std::pair<Position, int> nxt[3];
        nxt[0] = {p + DIR[d ^ 1], d};
        nxt[1] = {p + DIR[d] + DIR[d ^ ((d >> 1) | 2)], d ^ 3 ^ (d >> 1)};
        nxt[2] = {p + DIR[d ^ 3 ^ (d >> 1)] + DIR[d ^ 3 ^ (d >> 1)], d ^ ((d >> 1) | 2)};

        for (int i = 0; i < 3; i++) {
            int nxtdis = dis + 1 + ShipInMainChannel(p, d);

            if (ShipInOcean(nxt[i].first, nxt[i].second) &&
                nxtdis < ocean_dist_grid[nxt[i].second].Get(nxt[i].first)) {
                ocean_dist_grid[nxt[i].second].Set(nxt[i].first, nxtdis);
                q.push({-nxtdis, nxt[i]});
            }
        }
    }
}

void Map::FindDeleveryShortestPath() {
    FindOceanShortestPath(delivery_points_, delivery_point_dist_);
}

int Map::GetDeliveryPointsDist(const Position &pos, int dir) const noexcept {
    if (dir == -1) {
        int mi = INF;
        for (int i = 0; i < 4; i++) {
            mi = std::min(mi, delivery_point_dist_[i].Get(pos));
        }
        return mi;
    }

    return delivery_point_dist_[dir].Get(pos);
}

int Map::GetDeliveryPointsMarch(const Position &pos, int dir) const noexcept {
    int dis = delivery_point_dist_[dir].Get(pos);
    std::vector<int> res{3};

    if (delivery_point_dist_[dir].Get(pos + DIR[dir]) < dis) {
        res.clear();
        res.push_back(2);
        dis = delivery_point_dist_[dir].Get(pos + DIR[dir]);
    } else if (delivery_point_dist_[dir].Get(pos + DIR[dir]) == dis) {
        res.push_back(2);
    }

    int tmp_dir = dir ^ (2 | (dir >> 1));
    Position tmp_pos = pos + DIR[dir] + DIR[dir ^ 3 ^ (dir >> 1)];
    if (delivery_point_dist_[tmp_dir].Get(tmp_pos) < dis) {
        res.clear();
        res.push_back(1);
        dis = delivery_point_dist_[tmp_dir].Get(tmp_pos);
    } else if (delivery_point_dist_[tmp_dir].Get(tmp_pos) == dis) {
        res.push_back(1);
    }

    tmp_dir = dir ^ 3 ^ (dir >> 1);
    tmp_pos = pos + DIR[dir] + DIR[dir];
    if (delivery_point_dist_[tmp_dir].Get(tmp_pos) < dis) {
        res.clear();
        res.push_back(0);
        dis = delivery_point_dist_[tmp_dir].Get(tmp_pos);
    } else if (delivery_point_dist_[tmp_dir].Get(tmp_pos) == dis) {
        res.push_back(0);
    }

    return res[rand() % res.size()];
}

bool Map::CheckLandCollision(const Position &cur_x,
                             const Position &nxt_x,
                             const Position &cur_y,
                             const Position &nxt_y) const noexcept {
    const static std::set<char> land_main{'>', 'R', 'c', 'B'};
    for (const auto &pos : {cur_x, nxt_x, cur_y, nxt_y}) {
        if (!IsField(pos)) {
            return false;
        }
    }

    auto check = [&](const Position &pos) -> bool { return land_main.count(map_[pos[0]][pos[1]]) == 0; };

    // 下一个位置重合
    if (nxt_x == nxt_y && check(nxt_x)) {
        return true;
    }
    // 面对面迎面撞
    if (nxt_y == cur_x && nxt_x == cur_y && (check(cur_x) || check(cur_y))) {
        return true;
    }

    return false;
}

int Map::Manhattan(const Position &x, const Position &y) const noexcept {
    return abs(x[0] - y[0]) + abs(x[1] - y[1]);
}

Map g_map;