#include "ocean_scheduler.hpp"

ShipPlanMaker::ShipPlanMaker() {
    for (int i = 0; i < g_berths.size(); i++)
        berths_id_.push_back(i);

    ship_id_ = 0;
}

ShipPlanMaker::ShipPlanMaker(int ship_id) : ship_id_(ship_id) {
    for (int i = 0; i < g_berths.size(); i++)
        berths_id_.push_back(i);
}

ShipPlanMaker::ShipPlanMaker(int ship_id, std::vector<int> berths_id)
    : ship_id_(ship_id), berths_id_(berths_id) {}

void ShipPlanMaker::Clear() {
    permutation_.clear();
}

void ShipPlanMaker::ResetBerthIDs(std::vector<int> berths_id) {
    berths_id_.assign(berths_id.begin(), berths_id.end());
    Clear();
}

void ShipPlanMaker::RandomGlobal() {
    int n = berths_id_.size();

    permutation_.clear();

    for (int i = 0; i < n; ++i) {
        permutation_.push_back(i);  // 生成初始序列 1, 2, ..., n
    }

    for (int i = 1; i < n; i++) {
        int p = rand() % i;

        std::swap(permutation_[p], permutation_[i]);
    }

    // std::random_device rd;
    // std::mt19937 g(rd());

    // std::shuffle(permutation_.begin(), permutation_.end(), g); // 随机打乱序列

    permutation_.resize(rand() % n + 1);  // 只取前 k 个数作为排列
}

void ShipPlanMaker::RandomLocal() {
    int n = permutation_.size();
    int m = berths_id_.size();

    if (rand() % 10 < 8 && n > 1) {
        int u = rand() % n;
        int v = rand() % (n - 1);

        v += (v >= u);

        // fprintf(stderr, "swap pair (%d, %d)\n", u, v);

        std::swap(permutation_[u], permutation_[v]);
    } else if (rand() % 10 < 6 && n < m) {
        std::vector<int> appear(m, 0);
        int p;

        for (int i : permutation_)
            appear[i] = 1;

        for (p = 0; p < m && appear[p]; p++)
            ;

        // fprintf(stderr, "add %d\n", p);

        permutation_.push_back(p);
    } else {
        if (n > 0)
            permutation_.pop_back();
        else
            RandomLocal();
    }
}

void ShipPlanMaker::GetLocalBest() {
    std::vector<int> local_best(permutation_);
    double local_best_value = CalcValue();

    for (int i = 0; i < 100; i++) {
        RandomLocal();

        double current_value = CalcValue();

        if (current_value < local_best_value) {
            permutation_.assign(local_best.begin(), local_best.end());
        } else {
            local_best_value = current_value;
            local_best.assign(permutation_.begin(), permutation_.end());
        }
    }
}

void ShipPlanMaker::GetGlobalBestTSP() {
    Clear();
    std::vector<int> global_best(permutation_);
    double global_best_value = CalcValue();

    for (int i = 0; i < 100; i++) {
        RandomGlobal();
        GetLocalBest();

        double current_value = CalcValue();

        if (current_value > global_best_value) {
            global_best.assign(permutation_.begin(), permutation_.end());
            global_best_value = current_value;
        }
    }

    permutation_.assign(global_best.begin(), global_best.end());
}

void ShipPlanMaker::GetGlobalBestForce() {
    std::vector<int> global_best;
    double global_best_value = 0;

    for (int k = 0; k < (1 << berths_id_.size()); k++) {
        permutation_.clear();

        for (int i = 0; i < berths_id_.size(); i++)
            if ((k >> i) & 1) permutation_.push_back(i);

        do {
            double current_value = CalcValue();

            if (current_value > global_best_value) {
                global_best.assign(permutation_.begin(), permutation_.end());
                global_best_value = current_value;
            }
        } while (std::next_permutation(permutation_.begin(), permutation_.end()));
    }

    permutation_.assign(global_best.begin(), global_best.end());
}

void ShipPlanMaker::GetGlobalBest() {
    if (berths_id_.size() < 9) {
        GetGlobalBestForce();
    } else {
        GetGlobalBestTSP();
    }
}

int ShipPlanMaker::GetNextBerth() {
    GetGlobalBest();

    return (permutation_.size()) ? berths_id_[permutation_.front()] : -1;
}

double ShipPlanMaker::CalcValue() {
    int cframe = g_frame_id;

    // Ship& ship = g_ships[ship_id_];
    Position cpos = g_ships[ship_id_].GetPostion();
    int cdir = g_ships[ship_id_].GetDir();
    int ccap = g_ships[ship_id_].GetCap();
    int cvalue = g_ships[ship_id_].GetValue();
    static int recover_time = 10;

    // cframe += g_map.GetDeliveryPointsDist(cpos, cdir);

    for (int i : permutation_) {
        // transfer to real berth id
        i = berths_id_[i];
        // go to berth
        cframe += g_berths[i].GetOceanDistance(cpos, cdir) + recover_time;
        cpos = g_berths[i].core_pos_;
        cdir = -1;

        // load cargo
        int take_cargo_num = std::min(ccap, g_berths[i].GetCargoNum());
        // cvalue += take_cargo_num;
        cvalue += g_berths[i].CalcValue(take_cargo_num) * (!!take_cargo_num);
        cframe += take_cargo_num / g_berths[i].velocity_ + (take_cargo_num % g_berths[i].velocity_ != 0);
        ccap -= take_cargo_num;
    }

    // go to delivery point
    cframe += g_map.GetDeliveryPointsDist(cpos, cdir);

    return (cframe > MAX_FRAME_NUM - 6) ? -INF : 1.0 * cvalue / (cframe - g_frame_id + 1);
}

void ShipPlanMaker::Print() {
    int cframe = g_frame_id;

    Ship &ship = g_ships.front();
    Position cpos = ship.GetPostion();
    int cdir = ship.GetDir();
    int ccap = ship.GetCap();
    int cvalue = ship.GetValue();
    static int recover_time = 10;

    std::cerr << "frame: " << g_frame_id << std::endl;

    for (int i : permutation_) {
        i = berths_id_[i];

        // go to berth
        cframe += g_berths[i].GetOceanDistance(cpos, cdir) + recover_time;
        cpos = g_berths[i].core_pos_;
        cdir = -1;

        // load cargo
        int take_cargo_num = std::min(ccap, g_berths[i].GetCargoNum());
        cvalue += g_berths[i].CalcValue(take_cargo_num) * (!!take_cargo_num);
        cframe += take_cargo_num / g_berths[i].velocity_ + (take_cargo_num % g_berths[i].velocity_ != 0);
        ccap -= take_cargo_num;

        fprintf(stderr, "p frame %d: berth %d; cap %d; value %d\n", cframe, i, ccap, cvalue);
    }

    // go to delivery point
    cframe += g_map.GetDeliveryPointsDist(cpos, cdir);

    fprintf(stderr, "p frame %d: sent; cap %d; value %d; v: %.3f\n", cframe, ccap, cvalue,
            1.0 * cvalue / (cframe - g_frame_id + 1));
}

void OceanScheduler::ShipDFS(double now, int j, double &sum, std::vector<int> &tmp, std::vector<int> &res) {
    if (j >= g_ships.size()) {
        if (now > sum) {
            sum = now;
            res = tmp;
        }
        return;
    }
    if (g_ships[j].GetStatus() != 2 && g_ships[j].GetCap() * g_caplim >= g_ship_capacity) {
        for (auto &i : g_berths) {
            bool used = false;
            for (auto k : tmp) {
                if (k == i.id_) {
                    used = true;
                    break;
                }
            }
            if (used) continue;
            tmp.push_back(i.id_);
            // std::cerr<<'/'<<i.GetOceanDist(g_ships[j].GetPostion(),g_ships[j].GetDir())<<'\n';
            double value = 1.0 + i.CalcValue(g_ships[j].GetCap());
            double distance = i.GetOceanDistance(g_ships[j].GetPostion(), g_ships[j].GetDir());
            ShipDFS(now + value / (10 + distance), j + 1, sum, tmp, res);
            tmp.pop_back();
        }
    }
    tmp.push_back(-1);
    ShipDFS(now, j + 1, sum, tmp, res);
    tmp.pop_back();
}

void OceanScheduler::ScheduleShips() {
#ifdef LOCAL_TIMER
    Timer("ScheduleShips");
#endif

    while (sspm.GetShipNum() < g_ships.size()) {
        sspm.AddShip();
    }

    for (int ship_id = 0; ship_id < sspm.GetShipNum(); ship_id++) {
        g_ships[ship_id].SetTar(sspm.GetNextBerth(ship_id));

        // if (g_frame_id % 100 == 0) {
        //     int tar = g_ships[ship_id].GetTar();
        //     std::cerr << "ship " << ship_id << " info" << std::endl;
        //     std::cerr << "cap: " << g_ships[ship_id].GetCap() << std::endl;
        //     std::cerr << "tar: " << g_ships[ship_id].GetTar() << std::endl;
        //     std::cerr << "pos: " << g_ships[ship_id].GetPostion()[0] << ", "
        //               << g_ships[ship_id].GetPostion()[1] << std::endl;
        //     if (tar != -1) std::cerr << "aim: " << g_berths[tar].core_pos_ << std::endl;

        //     std::cerr << "sspm info" << std::endl;
        //     sspm_.Print();
        // }
    }

    // for (Ship& ship : g_ships)
    // {
    //     ShipPlanMaker spm(ship.id_);
    //     ship.SetTar(spm.GetNextBerth());

    //     if (g_frame_id % 100 == 0)
    //     {
    //         int ship_id = ship.GetID();
    //         int tar = g_ships[ship_id].GetTar();
    //         std::cerr << "ship " << ship_id << " info" << std::endl;
    //         std::cerr << "cap: " << g_ships[ship_id].GetCap() << std::endl;
    //         std::cerr << "tar: " << g_ships[ship_id].GetTar() << std::endl;
    //         std::cerr << "pos: " << g_ships[ship_id].GetPostion()[0] << ", " <<
    //         g_ships[ship_id].GetPostion()[1] << std::endl; if (tar != -1)
    //             std::cerr << "aim: " << g_berths[tar].core_pos_[0] << ", " <<
    //             g_berths[tar].core_pos_[1] << std::endl;

    //         // std::cerr << "sspm info" << std::endl;
    //         // sspm_.Print();

    //     }
    // }
}

void OceanScheduler::ScheduleShipsTemp() {
    // while (sspm.GetShipNum() < g_ships.size()) {
    //     sspm.AddShip();
    // }
#ifdef LOCAL_TIMER
    Timer timer("OceanScheduler::ScheduleShipsTemp", 2);
#endif
    for (auto &i : g_ships) {
        i.SetTar(sspm.GetNextBerthTemp(i.id_));
    }
}

int ShipsPlanMaker::GetShipNum() {
    return ship_berth_set_.size();
}

void ShipsPlanMaker::AddShip() {
    // if (GetShipNum() == 0) {
    //     KmeanBerths();
    // }

    ship_berth_set_.push_back(-1);
    step_.push_back(0);

    SetBerthSet(GetShipNum() - 1);

    std::cerr << "add ship ok" << std::endl;
}

void ShipsPlanMaker::SetBerthSet(int ship_id) {
    if (ship_berth_set_[ship_id] != -1) {
        used_set_[ship_berth_set_[ship_id]] = 0;
    }

    step_[ship_id] = 0;

    int max_set_id = -1, max_cargo_num = 0;

    for (int set_id = 0; set_id < g_berth_group_manager.GetAllBerthGroup().size(); set_id++) {
        if (used_set_[set_id] == 1) {
            continue;
        }

        int cur_cargo_num = GetBerthSetCargoNum(set_id);

        if (cur_cargo_num > max_cargo_num) {
            max_cargo_num = cur_cargo_num;
            max_set_id = set_id;
        }
    }

    // max_set_id = 0;

    ship_berth_set_[ship_id] = max_set_id;

    if (max_set_id != -1) {
        used_set_[max_set_id] = 1;
    }
}

int ShipsPlanMaker::GetBerthSetCargoNum(int set_id) {
    int sum = 0;

    for (int bid : g_berth_group_manager.GetAllBerthGroup()[set_id].group_member_) {
        sum += g_berths[bid].GetCargoNum();
    }

    return sum;
}

int ShipsPlanMaker::StepToBerthID(int step, int set_id) {
    std::vector<int> berth_set;
    for (auto &berth_id : g_berth_group_manager.GetAllBerthGroup()[set_id].group_member_) {
        if (g_berths[berth_id].GetShipNum() == 0) {
            berth_set.push_back(berth_id);
        }
    }

    int set_size = berth_set.size();
    // int set_size = berth_sets_[set_id].size();

    if (set_size <= 1) {
        return 0;
    }

    int k = step % (set_size * 2 - 2);

    if (k >= set_size) {
        k = (set_size * 2 - 2) - k;
    }

    return berth_set[k];
}

int ShipsPlanMaker::GetNextBerthTemp(int ship_id) {
    if (g_ships[ship_id].GetCap() == 0) {
        return -1;
    }
    if (g_ships[ship_id].GetCaNum() &&
        g_map.GetDeliveryPointsDist(g_ships[ship_id].GetPostion(), g_ships[ship_id].GetDir()) >
            MAX_FRAME_NUM - 50 - g_frame_id) {
        return -1;
    }
    int id = ship_tar_hash[ship_id % g_berth_group_manager.GetAllBerthGroup().size()];
    int tarber = *g_berth_group_manager.GetAllBerthGroup()[id].group_member_.begin();
    int ttvl = -INF;
    if (g_ships[ship_id].GetTar() != -1 && g_berths[g_ships[ship_id].GetTar()].GetShipNum() == 0) {
        tarber = g_ships[ship_id].GetTar();
        ttvl = ttvl = g_berths[tarber].tot_value * 1.2;
    }
    for (auto &berth : g_berth_group_manager.GetAllBerthGroup()[id].group_member_) {
        if (g_berths[berth].GetShipNum() == 0) {
            if (g_berths[berth].tot_value > ttvl) {
                tarber = berth;
                ttvl = g_berths[berth].tot_value;
            }
        }
    }
    return tarber;
}

int ShipsPlanMaker::GetNextBerth(int ship_id) {
    if (g_ships[ship_id].GetCap() == 0) {
        return -1;
    }

    int set_id = ship_berth_set_[ship_id];

    if (set_id == -1 || GetBerthSetCargoNum(set_id) == 0) {
        SetBerthSet(ship_id);
    }

    if (set_id == -1) {
        return -1;
    } else {
        int cur_next_berth_id = StepToBerthID(step_[ship_id], set_id);

        step_[ship_id] += (g_berths[cur_next_berth_id].GetCargoNum() == 0);

        return StepToBerthID(step_[ship_id], set_id);
    }

    std::cerr << "next berth ok" << std::endl;
}

void ShipsPlanMaker::Print() {
    std::cerr << "set info" << std::endl;

    std::cerr << "set cargo num: ";
    for (int i = 0; i < g_berth_group_manager.GetAllBerthGroup().size(); i++) {
        std::cerr << GetBerthSetCargoNum(i) << " ";
    }
    std::cerr << std::endl;

    std::cerr << "ship set: ";
    for (int i = 0; i < GetShipNum(); i++) {
        std::cerr << ship_berth_set_[i] << " ";
    }
    std::cerr << std::endl;
}

OceanScheduler g_ocean_scheduler;
ShipsPlanMaker sspm;