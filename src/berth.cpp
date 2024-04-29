#include "berth.hpp"

#include "map.hpp"

Berth::Berth(int id, Position core_pos, int velocity, Grid &&dist_grid, std::set<Position> &&area)
    : id_(id),
      core_pos_(core_pos),
      velocity_(velocity),
      area_(std::move(area)),
      time_(g_map.GetDeliveryPointsDist(core_pos, -1)),
      dist_grid_(std::move(dist_grid)) {
    // 更新到这个泊位的海上最短路
    std::vector<Position> src;
    for (auto pos : area_) {
        src.push_back(pos);
    }
    g_map.FindOceanShortestPath(src, ocean_dist_grid_);
}

Berth::Berth(Berth &&other) noexcept
    : id_(other.id_),
      core_pos_(other.core_pos_),
      velocity_(other.velocity_),
      area_(other.area_),
      time_(other.time_),
      tot_value(other.tot_value),
      tot_cnt(other.tot_cnt),
      ship_num_(other.ship_num_),
      cargo_(std::move(other.cargo_)),
      dist_grid_(std::move(other.dist_grid_)),
      ocean_dist_grid_(std::move(other.ocean_dist_grid_)),
      active_(other.active_) {
    other.ship_num_ = 0;
    other.tot_value = 0;
    other.tot_cnt = 0;
    other.active_ = false;
}

int Berth::CalcValue(int num) const noexcept {
    if (cargo_.empty()) return 0;
    num = std::min({num, static_cast<int>(cargo_.size())});
    int res = std::accumulate(cargo_.begin(), cargo_.begin() + num, 0);

    if (!active_) {
        return res *= 5;
    }
    return res;
}

int Berth::GetCargoNum() const noexcept {
    return cargo_.size();
}

int Berth::GetShipNum() const noexcept {
    return ship_num_;
}

void Berth::AddShipNum(int x) noexcept {
    ship_num_ += x;
}

const Grid &Berth::GetLandDistanceGrid() const noexcept {
    return dist_grid_;
}

int Berth::GetLandDistance(Position pos) const noexcept {
    if (!active_) {
        return INF;
    }
    assert(g_map.IsField(pos));
    int res = dist_grid_.Get(pos);
    if (!HereIsOurShip()) {
        res += 200;
    }
    return res;
}

int Berth::GetLandDirection(Position pos) const noexcept {
    assert(g_map.IsField(pos));
    return dist_grid_.GetDescentDirection(pos);
}

bool Berth::InBerth(Position pos) const noexcept {
    return area_.count(pos) > 0;
}

void Berth::AddCargos(int value) noexcept {
    cargo_.push_back(value);
    this->tot_value += value;
    this->tot_cnt += 1;
}

std::pair<int, int> Berth::TakeCargo(int num) noexcept {
    std::vector<int> cargo_new;
    num = std::min(std::min(static_cast<int>(cargo_.size()), velocity_), num);
    if (g_frame_id < MAX_FRAME_NUM - time_) {
        for (int i = 0; i < num; i++) {
            g_value_token += cargo_[i];
        }
    }
    int value = 0;
    for (int i = 0; i < num; i++) {
        value += cargo_[i];
    }

    for (int i = num; i < cargo_.size(); i++) {
        cargo_new.push_back(cargo_[i]);
    }
    cargo_ = cargo_new;
    return {num, value};
}

int Berth::GetOceanMarch(Position pos, int dir) const noexcept {
    int res = 3;
    int dis = ocean_dist_grid_[dir].Get(pos);

    int tmp_dir = dir;
    Position tmp_pos = pos + DIR[dir];
    if (ocean_dist_grid_[tmp_dir].Get(tmp_pos) < dis) {
        res = 2;
        dis = ocean_dist_grid_[tmp_dir].Get(tmp_pos);
    }

    tmp_dir = dir ^ (2 | (dir >> 1));
    tmp_pos = pos + DIR[dir] + DIR[dir ^ 3 ^ (dir >> 1)];
    if (ocean_dist_grid_[tmp_dir].Get(tmp_pos) < dis) {
        res = 1;
        dis = ocean_dist_grid_[tmp_dir].Get(tmp_pos);
    }

    tmp_dir = dir ^ 3 ^ (dir >> 1);
    tmp_pos = pos + DIR[dir] + DIR[dir];
    if (ocean_dist_grid_[tmp_dir].Get(tmp_pos) < dis) {
        res = 0;
        dis = ocean_dist_grid_[tmp_dir].Get(tmp_pos);
    }

    return res;
}

int Berth::GetOceanDistance(Position pos, int dir) const noexcept {
    if (dir == -1) {
        int res = INF;
        for (int i = 0; i < 4; i++) {
            res = std::min(res, ocean_dist_grid_[i].Get(pos));
        }
        return res;
    }
    return ocean_dist_grid_[dir].Get(pos);
}

void Berth::Activate() noexcept {
    active_ = true;
}

void Berth::Deactivate() noexcept {
    this->active_ = false;
}

bool Berth::IsActive() const noexcept {
    return active_;
}

void Berth::ChangeDistance(std::map<Position, int> &pass) noexcept {
    for (auto [p, t] : pass) {
        int newdist = dist_grid_.Get(p) * 0.8 + (g_frame_id - t) * 0.2;
        dist_grid_.Set(p, newdist);
    }
    pass.clear();
}

bool Berth::HereIsOurShip() const noexcept {
    return our_ship_;
}

/**
 * @brief 设定泊位是否有我们的船
 */
void Berth::SetOurShip(bool f) noexcept {
    our_ship_ = f;
}

std::vector<Berth> g_berths;

BerthGroup::BerthGroup(const std::vector<int> &group) {
    for (int x : group) {
        group_member_.insert(x);
    }
}

void BerthGroupManager::InitGroups() {
    // 用并查集将泊位分组
    int berth_num = g_berths.size();
    DisjointSetUnion dsu(berth_num);

    for (int u = 0; u < berth_num; u++) {
        for (int v = 0; v < u; v++) {
            if (GetL1(g_berths[u].core_pos_, g_berths[v].core_pos_) < 20) {
                dsu.Merge(u, v);
            }
        }
    }

    auto berth_groups = dsu.GetSets();

    for (const auto &bset : berth_groups) {
        groups_.emplace_back(bset);
        std::cerr << "set: ";
        for (int bid : bset) {
            std::cerr << bid << " ";
        }
        std::cerr << std::endl;
    }
}

const std::vector<BerthGroup> &BerthGroupManager::GetAllBerthGroup() const {
    return groups_;
}

std::vector<BerthGroup> BerthGroupManager::GetActivatedBerthGroup() const {
    std::vector<BerthGroup> activated_groups;
    for (const auto &group : groups_) {
        if (group.active_) {
            activated_groups.push_back(group);
        }
    }
    return activated_groups;
}

int BerthGroupManager::ActivatedSize() const {
    int res = 0;
    for (const auto &berths : groups_) {
        res += berths.active_;
    }
    return res;
}

void BerthGroupManager::ActivateOneGroup(int id) {
    groups_[id % groups_.size()].active_ = true;
}

BerthGroupManager g_berth_group_manager;