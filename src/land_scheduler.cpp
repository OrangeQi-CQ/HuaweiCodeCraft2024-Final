#include "land_scheduler.hpp"


ScheduleGraph::ScheduleGraph(int robot_cnt, int cargo_cnt, int berth_cnt) {
#ifdef LOCAL_TIMER
    Timer timer("ScheduleGraph Constructor", 1);
#endif  // LOCAL_TIMER

    this->robot_cnt_ = robot_cnt;
    this->cargo_cnt_ = cargo_cnt;
    this->berth_cnt_ = berth_cnt;
    this->ver_cnt_ = robot_cnt + cargo_cnt + berth_cnt;

    adj_.resize(ver_cnt_);
    for (auto &line : benefit_matrix_) {
        std::fill(line.begin(), line.end(), 0);
    }
}

void ScheduleGraph::Init(int robot_cnt, int cargo_cnt, int berth_cnt) {
    source_ = -1;
    sink_ = -1;
    ver_cnt_ = robot_cnt + cargo_cnt + berth_cnt;
    robot_cnt_ = robot_cnt;
    cargo_cnt_ = cargo_cnt;
    berth_cnt_ = berth_cnt;
    berth_group_cnt_ = 0;

    already_in_group_.clear();
    tmp_group_limitation_.clear();
    cargo_berth_.clear();

    adj_.resize(ver_cnt_);
    for (auto &line : adj_) {
        line.clear();
    }

    edge_.clear();
    dis_.clear();
    incf_.clear();
    pre_.clear();

    for (auto &line : benefit_matrix_) {
        std::fill(line.begin(), line.end(), 0);
    }
}

void ScheduleGraph::SetMisionBenifit(int robot_id, int cargo_id, double benefit) noexcept {
#ifdef LOCAL
    assert(robot_id < robot_cnt_);
    assert(cargo_id < cargo_cnt_);
#endif
    benefit_matrix_[robot_id][cargo_id] = benefit;
    AddEdge(GetRobotID(robot_id), GetCargoID(cargo_id), 1, benefit);
}

void ScheduleGraph::SetCargoBerth(int cargo_id, int berth_id) noexcept {
#ifdef LOCAL
    assert(cargo_id < cargo_cnt_);
    assert(berth_id < berth_cnt_);
#endif
    assert(cargo_berth_.count(cargo_id) == 0);
    cargo_berth_.insert(cargo_id);
    AddEdge(GetCargoID(cargo_id), GetBerthID(berth_id), 1, 0);
}

double ScheduleGraph::GetMissionBenifit(int robot_id, int cargo_id) noexcept {
#ifdef LOCAL
    assert(robot_id < robot_cnt_);
    assert(cargo_id < cargo_cnt_);
#endif
    return benefit_matrix_[robot_id][cargo_id];
}

void ScheduleGraph::SetBerthGroupLimitation(const std::set<int> &berth_group, int limitation) noexcept {
    int berth_group_id = berth_group_cnt_;
    adj_.emplace_back();
    for (int berth_id : berth_group) {
        // 首先减去已经取到货的机器人数量
        for (const auto &robot : g_robots) {
            if (robot.FullCargo()) {
                int target_berth_id = robot.GetTargetBerthID();
                if (target_berth_id == berth_id) {
                    limitation -= 1;
                }
            }
        }
        assert(already_in_group_.count(berth_id) == 0);
        already_in_group_.insert(berth_id);
        AddEdge(GetBerthID(berth_id), GetBerthGroupID(berth_group_id), INF, 0);
    }

    tmp_group_limitation_[berth_group_id] = limitation;
    berth_group_cnt_ += 1;
    ver_cnt_ += 1;
}

std::map<int, int> ScheduleGraph::MatchRobotCargo() noexcept {
    source_ = ver_cnt_;
    sink_ = ver_cnt_ + 1;
    ver_cnt_ += 2;
    adj_.emplace_back();
    adj_.emplace_back();
    dis_.resize(ver_cnt_);
    incf_.resize(ver_cnt_);
    pre_.resize(ver_cnt_);

    for (int i = 0; i < robot_cnt_; i++) {
        AddEdge(source_, GetRobotID(i), 1, 0);
    }
    for (auto [berth_group_id, limit] : tmp_group_limitation_) {
        AddEdge(GetBerthGroupID(berth_group_id), sink_, limit, 0);
    }

    int flow = 0;
    double cost = 0;
    std::map<std::pair<int, int>, int> match;

    while (SPFA()) {
        int tmp = incf_[sink_];
        assert(tmp == 1);  // 在本问题中，每次的增广流量一定是 1。
        flow += tmp;
        cost += tmp * dis_[sink_];
        for (int i = sink_, j; i != source_; i = j) {
            // i 从汇点 t 往回找，边的方向是 j->i，流量为 tmp
            edge_[pre_[i]].cap_ -= tmp;
            edge_[pre_[i] ^ 1].cap_ += tmp;
            j = edge_[pre_[i] ^ 1].to_;

            // 寻找机器人——货物匹配，注意辨别反向边
            if (j < robot_cnt_ && i >= robot_cnt_ && i < robot_cnt_ + cargo_cnt_) {
                match[std::pair{j, i}] += tmp;
            }
            if (i < robot_cnt_ && j >= robot_cnt_ && j < robot_cnt_ + cargo_cnt_) {
                match[std::pair{i, j}] -= tmp;
            }
        }
    }

    std::map<int, int> ans;
#ifdef LOCAL_TEST
    /**
     * @test 检查一下返回的匹配是否不会重合
     */
    std::set<int> st_u, st_v;
#endif
    for (auto [p, v] : match) {
        if (v != 0) {
            assert(v == 1);
            auto [u, v] = p;
            ans[u] = v - robot_cnt_;
#ifdef LOCAL_TEST
            assert(st_u.count(u) == 0);
            assert(st_v.count(v) == 0);
            st_u.insert(u);
            st_v.insert(v);
#endif
        }
    }

    return ans;
}

void ScheduleGraph::AddEdge(int u, int v, int cap, double cost) noexcept {
    adj_[u].push_back(edge_.size());
    edge_.push_back(Edge{v, cap, cost});
    adj_[v].push_back(edge_.size());
    edge_.push_back(Edge{u, 0, -cost});
}

bool ScheduleGraph::SPFA() noexcept {
    int s = source_;
    int t = sink_;
    std::fill(dis_.begin(), dis_.end(), -INF);
    std::fill(incf_.begin(), incf_.end(), 0);

    std::vector<bool> inq(ver_cnt_ + 1, false);
    std::queue<int> que;
    que.push(s);
    dis_[s] = 0;
    inq[s] = true;
    incf_[s] = INF;

    while (!que.empty()) {
        int u = que.front();
        que.pop();
        inq[u] = false;

        for (int i : adj_[u]) {
            auto [v, cap, co] = edge_[i];
            if (cap > 0 && dis_[u] + co > dis_[v] + EPS) {
                dis_[v] = dis_[u] + co;
                pre_[v] = i;
                incf_[v] = std::min(cap, incf_[u]);

                if (!inq[v]) {
                    que.push(v);
                    inq[v] = true;
                }
            }
        }
    }

    return incf_[t] > 0;
}

int ScheduleGraph::GetRobotID(int robot_id) const noexcept {
    return robot_id;
}

int ScheduleGraph::GetCargoID(int cargo_id) const noexcept {
    return robot_cnt_ + cargo_id;
}

int ScheduleGraph::GetBerthID(int berth_id) const noexcept {
    return robot_cnt_ + cargo_cnt_ + berth_id;
}

int ScheduleGraph::GetBerthGroupID(int berth_group_id) const noexcept {
    return robot_cnt_ + cargo_cnt_ + berth_cnt_ + berth_group_id;
}


void LandScheduler::ScheduleRobots() {
#ifdef LOCAL_TIMER
    Timer timer("LandScheduler::ScheduleRobots", 12);
#endif
    ScheduleBusyRobots();
    ScheduleFreeRobots();
}

void LandScheduler::ScheduleBusyRobots() {
    for (auto &robot : g_robots) {
        if (!robot.FullCargo() || robot.IsWaitingForLLM()) {
            continue;
        }

        int target_berth_id = -1;
        int dist_to_berth = INF;

        for (const auto &berth_group : g_berth_group_manager.GetActivatedBerthGroup()) {
            for (int id : berth_group.group_member_) {
                const auto &berth = g_berths[id];
                int cur_dist = berth.GetLandDistance(robot.GetPosition());

                if (dist_to_berth > cur_dist) {
                    dist_to_berth = cur_dist;
                    target_berth_id = berth.id_;
                }
            }
        }

        if (target_berth_id == -1) {
            for (const auto &berth : g_berths) {
                int cur_dist = berth.GetLandDistance(robot.GetPosition());

                if (dist_to_berth > cur_dist) {
                    dist_to_berth = cur_dist;
                    target_berth_id = berth.id_;
                }
            }
        }

        // fprintf(stderr, "robotid %d, dist %d\n", robot.id_, dist_to_berth);

        assert(target_berth_id >= 0 && target_berth_id < g_berths.size());
        robot.SetTargetBerthID(target_berth_id);
    }

#ifdef LOCAL_TEST
    for (const auto &robot : g_robots) {
        if (robot.FullCargo()) {
            assert(robot.GetTargetBerthID() != -1);
            assert(robot.GetCargoPtr() != nullptr);
            assert(robot.GetCargoPtr()->value_ > 0);
            assert(robot.GetCargoPtr()->IsDeleted());
        }
    }
#endif
}

void LandScheduler::ScheduleFreeRobots() {
#ifdef LOCAL_TIMER
    Timer timer("LandScheduler::ScheduleFreeRobots", 13);
#endif  // LOCAL_TIMER

    for (auto &robot : g_robots) {
        // 如果它上一帧设定的目标，已经被别的机器人拿走了，
        if (!robot.FullCargo() && robot.GetCargoPtr() != nullptr && robot.GetCargoPtr()->IsDeleted() &&
            !robot.IsWaitingForLLM()) {
            robot.SetCargoPtr(nullptr);
        }
    }

    cargo_ptr_vec_ = g_cargo_manager.GetAllCargoPtr();
    graph_ptr_->Init(static_cast<int>(g_robots.size()), static_cast<int>(cargo_ptr_vec_.size()),
                     static_cast<int>(g_berths.size() + 1));

    // std::cerr << "MatchCargoBerth" << std::endl;
    MatchCargoBerth();

    // std::cerr << "CalcRobotCargoBenefit" << std::endl;
    CalcRobotCargoBenefit();

    // std::cerr << "DivideBerthGroups" << std::endl;
    DivideBerthGroups();

    // std::cerr << "UpdateMatch" << std::endl;
    UpdateMatch();

    // std::cerr << "Finish" << std::endl;
#ifdef LOCAL_TEST
    /**
     * @test 验证匹配不会重复
     */
    std::set<CargoPtr> st;
    for (const auto &robot : g_robots) {
        if (!robot.FullCargo() && !robot.IsWaitingForLLM() && robot.GetCargoPtr() != nullptr) {
            auto ptr = robot.GetCargoPtr();
            assert(!ptr->IsDeleted());
            assert(st.count(ptr) == 0);
            st.insert(ptr);
        }
    }
#endif
}

int LandScheduler::RoNumAroundCargo(CargoPtr cp, int dis) {
    int res = 0;
    for (auto &[id, other_robot] : g_other_robots) {
        if (other_robot.cargo_num_ < other_robot.volume_ && cp->GetDistance(other_robot.position_) < dis) {
            res++;
        }
    }
    return res;
}

void LandScheduler::Occupy() {
    std::map<int, bool> flag;
    for (const auto &cargo : cargo_ptr_vec_) {
        if (cargo->value_ >= 1400) {
            std::priority_queue<std::pair<int, int>> heap;
            for (auto &robot : g_robots) {
                if (flag[robot.id_]) {
                    continue;
                }
                heap.push({-cargo->GetDistance(robot.GetPosition()), robot.id_});
            }

            if (!heap.empty()) {
                auto [dis, id] = heap.top();
                dis = -dis;
                if (RoNumAroundCargo(cargo, dis) == 0) {
                    flag[id] = true;
                    if (g_robots[id].FullCargo()) {
                        g_robots[id].SetAddtionalTarget(cargo);
                        // break;
                    }
                }
            }
        }
    }
}

void LandScheduler::MatchCargoBerth() noexcept {
    std::fill(cargo_berth_dist_.begin(), cargo_berth_dist_.end(), INF);
    const auto &activated_berth_groups = g_berth_group_manager.GetActivatedBerthGroup();

    for (const auto &cargo : cargo_ptr_vec_) {
        const int cargo_id = cargo->GetID();

        // 虚拟货物向虚拟泊位连边
        if (cargo->IsVirtual()) {
            graph_ptr_->SetCargoBerth(cargo_id, VIRTUAL_BERTH_ID);
            continue;
        }

        // 其他货物找最近的泊位
        int target_berth_id = -1;
        int distance = INF;  // 货物到泊位的距离

        for (const auto &berth_group : activated_berth_groups) {
            for (int x : berth_group.group_member_) {
                const auto &berth = g_berths[x];
                const int cur_dist = berth.GetLandDistance(cargo->position_);
                if (cur_dist < distance) {
                    distance = cur_dist;
                    target_berth_id = berth.id_;
                }
            }
        }

        cargo_berth_dist_[cargo_id] = distance;
        if (target_berth_id != -1) {
            graph_ptr_->SetCargoBerth(cargo_id, target_berth_id);
        }
    }
}

void LandScheduler::CalcRobotCargoBenefit() noexcept {
#ifdef LOCAL_TIMER
    Timer timer("LandScheduler::CalcRobotCargoBenefit", 10);
#endif  // LOCAL_TIMER

    // 暂存货物收益，选比较优的若干个进行匹配
    static std::vector<std::pair<double, CargoPtr>> benefit_vec;

    // 记录自己的机器人都在哪些位置
    static std::set<Position> self_robot_position;
    self_robot_position.clear();
    for (const auto &robot : g_robots) {
        self_robot_position.insert(robot.GetPosition());
    }

    // 枚举所有机器人——货物组合，计算收益
    for (const auto &robot : g_robots) {
        if (robot.FullCargo()) {
            continue;
        }

        benefit_vec.clear();

        // 机器人已经走过的距离
        int distance3 = INF;
        for (auto &berth : g_berths) {
            distance3 = std::min(distance3, berth.GetLandDistance(robot.GetPosition()));
        }

        for (const auto &cargo : cargo_ptr_vec_) {
            const int cargo_id = cargo->GetID();

            // 如果是虚拟点
            if (cargo->IsVirtual()) {
                double distance = cargo->GetDistance(robot.GetPosition());
                graph_ptr_->SetMisionBenifit(robot.id_, cargo->GetID(), 20.0 / (distance + 200));
                continue;
            }

            // ! 如果货物为普通货物，或者货物到机器人的距离大于了 MAX_ROBOT_CARGO_MATCH_DISTANCE 就直接跳过
            if (cargo->value_ < MIN_ADVANCED_CARGO_VALUE ||
                cargo->GetDistance(robot.GetPosition()) > MAX_ROBOT_CARGO_MATCH_DISTANCE) {
                continue;
            }

            // 如果货物已经被占用，就放弃
            if (cargo->IsOccupied() ||
                self_robot_position.count(cargo->position_) && robot.GetPosition() != cargo->position_) {
                continue;
            }

            double distance1 = cargo->GetDistance(robot.GetPosition());  // 机器人到货物的距离
            double distance2 = cargo_berth_dist_[cargo->GetID()];        // 货物到泊位的距离

            // 如果货物寿命不足，就放弃
            if (cargo->GetRemainingLife() < 200 + distance1) {
                continue;
            }

            // 如果货物到不了泊位，就放弃
            if (distance2 > 1e5) {
                continue;
            }

            // 整个任务的总距离
            double distance = 0.2 * distance2 + distance1 + 50;
            double benefit = 1.0 * cargo->value_ / distance;
            benefit_vec.push_back({benefit, cargo});
        }

        // 对于该机器人，选前若干优的货物进行匹配
        int cargo_num = std::min(static_cast<int>(benefit_vec.size()), MAX_CARGO_NUM_FOR_ROBOT_MATCH);
        std::partial_sort(benefit_vec.begin(), benefit_vec.begin() + cargo_num, benefit_vec.end(),
                          std::greater<>());

        for (int i = 0; i < cargo_num; i++) {
            const auto &[benefit, cargo_ptr] = benefit_vec[i];
            graph_ptr_->SetMisionBenifit(robot.id_, cargo_ptr->GetID(), benefit);
        }
    }
}

void LandScheduler::DivideBerthGroups() {
#ifdef DONT_USE_MFMC_LIMITATIONO
    std::set<int> tmp;
    for (int i = 0; i < g_berths.size(); i++) {
        tmp.insert(i);
    }
    graph_ptr_->SetBerthGroupLimitation(tmp, 100);
    return;
#endif

    const auto &activated_berth_groups = g_berth_group_manager.GetActivatedBerthGroup();
    int activated_group_num = activated_berth_groups.size();

    for (const auto &group : activated_berth_groups) {
        graph_ptr_->SetBerthGroupLimitation(group.group_member_, g_robot_cnt / activated_group_num + 10);
        // std::cerr << "Limitation: ";
        // for (int x : group.group_member_) {
        //     std::cerr << x << " ";
        // }
        // std::cerr << "num = " << g_robot_cnt / activated_group_num + 5 << std::endl;
    }

    std::set<int> virtual_berth = {VIRTUAL_BERTH_ID};
    graph_ptr_->SetBerthGroupLimitation(virtual_berth, INF);
}


void LandScheduler::UpdateMatch() {
#ifdef LOCAL_TIMER
    Timer timer("LandScheduler::UpdateMatch()", 3);
#endif  // LOCAL_TIMER

    // 这一轮的匹配
    std::map<int, int> match_result = graph_ptr_->MatchRobotCargo();
    std::vector<int> new_target(g_robots.size(), -1);

    for (int i = 0; i < g_robots.size(); i++) {
        if (match_result.count(i)) {
            new_target[i] = match_result[i];
        } else {
            new_target[i] = -1;
        }
    }

#ifdef DONT_USE_ROBOT_AVOID_SWING
    // 弃用机器人防摆头算法
    for (auto &robot : g_robots) {
        // ! 正在等待大模型的机器人，一定不要修改
        if (robot.FullCargo() || robot.IsWaitingForLLM()) {
            continue;
        }
        if (match_result.count(robot.id_)) {
            int cargo_id = (match_result[robot.id_]);
            robot.SetCargoPtr(cargo_ptr_vec_[cargo_id]);
        } else {
            robot.SetCargoPtr(nullptr);
        }
    }
    return;
#endif

    // 记录上一轮的匹配

    std::vector<int> old_target(g_robots.size(), -1);
    std::unordered_set<CargoPtr> ptr_set;
    for (auto ptr : cargo_ptr_vec_) {
        ptr_set.insert(ptr);
    }

    for (auto &robot : g_robots) {
        // ! 正在等待大模型的机器人，一定不要修改
        if (robot.FullCargo() || robot.IsWaitingForLLM()) {
            continue;
        }

        auto ptr = robot.GetCargoPtr();

        // 这里非常重要。如果这个货物指针指向的货物是非法的，那么一定要提前删掉。
        if (ptr_set.count(ptr) == 0) {
            ptr = nullptr;
            robot.SetCargoPtr(nullptr);
        }
        if (ptr != nullptr && ptr->IsDeleted()) {
            ptr = nullptr;
            robot.SetCargoPtr(nullptr);
        }

        if (ptr != nullptr && !ptr->IsDeleted()) {
            old_target[robot.id_] = robot.GetCargoPtr()->GetID();
        }
    }

#ifdef LOCAL_TEST
    /**
     * @test 验证所有返回的匹配，不重不漏。验证旧匹配合法
     */
    std::set<int> stx, sty;
    for (auto [x, y] : match_result) {
        assert(stx.count(x) == 0 && sty.count(y) == 0);
        stx.insert(x);
        sty.insert(y);
    }

    for (int i = 0; i < g_robots.size(); i++) {
        if (g_robots[i].FullCargo() || g_robots[i].GetCargoPtr() == nullptr ||
            g_robots[i].GetCargoPtr()->IsDeleted()) {
            continue;
        }

        for (int j = i + 1; j < g_robots.size(); j++) {
            if (g_robots[j].FullCargo() || g_robots[j].GetCargoPtr() == nullptr ||
                g_robots[j].GetCargoPtr()->IsDeleted()) {
                continue;
            }

            assert(g_robots[i].GetCargoPtr()->GetID() != g_robots[j].GetCargoPtr()->GetID());
            assert(old_target[i] != old_target[j] || old_target[i] == -1);
            assert(new_target[i] != new_target[j] || new_target[i] == -1);
        }
    }
#endif

    // 并查集，将所有目标相关的机器人化为一个集合

    DisjointSetUnion dsu(g_robots.size());

    for (int i = 0; i < g_robots.size(); i++) {
        if (g_robots[i].FullCargo() || g_robots[i].IsWaitingForLLM()) {
            continue;
        }
        for (int j = i + 1; j < g_robots.size(); j++) {
            if (g_robots[i].FullCargo() || g_robots[i].IsWaitingForLLM()) {
                continue;
            }
            if (old_target[i] == new_target[j] && old_target[i] != -1) {
                dsu.Merge(i, j);
            }
            if (old_target[j] == new_target[i] && old_target[j] != -1) {
                dsu.Merge(i, j);
            }
        }
    }

    // 根据并查集，决定每个组是否改变决策

    auto robot_groups = dsu.GetSets();

    for (const auto &group : robot_groups) {
        // ! 正在等待大模型的机器人，一定不要修改
        if (std::any_of(group.begin(), group.end(), [&](int x) { return g_robots[x].IsWaitingForLLM(); })) {
            continue;
        }

        double old_benefit = 0;
        double new_benefit = 0;
        for (int robot_id : group) {
            old_benefit += graph_ptr_->GetMissionBenifit(robot_id, old_target[robot_id]);
            new_benefit += graph_ptr_->GetMissionBenifit(robot_id, new_target[robot_id]);
        }


        // TODO 这里的阈值, 不一定是最合理的.
        if ((new_benefit - old_benefit) / (old_benefit + EPS) > -INF) {
            for (int robot_id : group) {
                auto &robot = g_robots[robot_id];
                int cargo_id = new_target[robot_id];
                if (cargo_id != -1) {
                    robot.SetCargoPtr(cargo_ptr_vec_[cargo_id]);
                } else {
                    robot.SetCargoPtr(nullptr);
                }
            }
        }
    }

#ifdef LOCAL_TEST
    /**
     * @test 验证本轮的匹配不会发生冲突
     */
    int schedule_cnt = 0;
    for (int i = 0; i < g_robots.size(); i++) {
        if (g_robots[i].FullCargo()) {
            schedule_cnt += 1;
        }

        if (g_robots[i].FullCargo() || g_robots[i].GetCargoPtr() == nullptr ||
            g_robots[i].GetCargoPtr()->IsDeleted()) {
            continue;
        }
        for (int j = i + 1; j < g_robots.size(); j++) {
            if (g_robots[j].FullCargo() || g_robots[j].GetCargoPtr() == nullptr ||
                g_robots[j].GetCargoPtr()->IsDeleted()) {
                continue;
            }
            assert(g_robots[i].GetCargoPtr()->GetID() != g_robots[j].GetCargoPtr()->GetID());
        }
        if (g_robots[i].GetCargoPtr() != nullptr) {
            schedule_cnt += 1;
        }
    }

    // fprintf(stderr, "frame %d : %d g_robots has target.", g_frame_id, schedule_cnt);
    // std::cerr << std::endl;
#endif
}

LandScheduler g_land_scheduler;