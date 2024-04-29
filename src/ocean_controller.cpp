#include "ocean_controller.hpp"

#include "map.hpp"

int ShipDist(int i, int j) {
    Position p1 = g_ships[i].GetPostion();
    Position p2 = g_ships[j].GetPostion();

    return GetL1(p1, p2);
}

bool OceanController::CheckShipMayCrash(int i, int j) {
    return ShipDist(i, j) < ship_crash_limit;
}

bool OceanController::CheckShipCrash(int ship_a, int ship_b) {
    Instruction ins_a = g_ships[ship_a].CalculateInstruction();
    Instruction ins_b = g_ships[ship_b].CalculateInstruction();

    ShipPositions sp_a_new = g_ships[ship_a].GetPostionsAfterInstruction(ins_a);
    ShipPositions sp_b_new = g_ships[ship_b].GetPostionsAfterInstruction(ins_b);

    ShipPositions sp_a_cur = g_ships[ship_a].GetPostions();
    ShipPositions sp_b_cur = g_ships[ship_b].GetPostions();

    for (Position& pa : sp_a_new)
        for (Position& pb : sp_b_cur)
            if ((pa[0] == pb[0]) && (pa[1] == pb[1]) && !g_map.PointInMainChannel(pa)) return true;

    for (Position& pa : sp_a_cur)
        for (Position& pb : sp_b_new)
            if ((pa[0] == pb[0]) && (pa[1] == pb[1]) && !g_map.PointInMainChannel(pa)) return true;

    for (Position& pa : sp_a_new)
        for (Position& pb : sp_b_new)
            if ((pa[0] == pb[0]) && (pa[1] == pb[1]) && !g_map.PointInMainChannel(pa)) return true;

    return false;
}

void OceanController::ControlShips() {
    for (int i = 0, ship_num = g_ships.size(); i < ship_num; i++) {
        Instruction cur_ins = g_ships[i].CalculateInstruction();

        if (g_ships[i].GetStatus() != 0) {
            g_ships[i].AddInstruction(cur_ins);
        } else {
            if (cur_ins.first == "ship" || cur_ins.first == "rot")
                g_ships[i].AddInstruction(AstarFindNextInstruction(i));
            else
                g_ships[i].AddInstruction(cur_ins);
        }

        if (g_frame_id % 100 == 0) {
            fprintf(stderr, "ship value: %d pos: (%d, %d)\n", g_ships[i].GetValue(),
                    g_ships[i].GetPostion()[0], g_ships[i].GetPostion()[1]);

            if (g_ships[i].GetTar() != -1) {
                Position p = g_berths[g_ships[i].GetTar()].core_pos_;
                fprintf(stderr, "aim pos: (%d, %d)\n", p[0], p[1]);
            }
        }
    }
}

void OceanController::ControlShips2() {
    int ship_num = g_ships.size();

    for (int i = 0; i < ship_num; i++) {
        if (g_ships[i].GetStatus() != 0) {
            g_ships[i].AddInstruction(g_ships[i].CalculateInstruction());

            continue;
        }

        bool is_crash = 0;

        for (int j = 0; j < ship_num && (is_crash == 0); j++) {
            if (i != j && CheckShipMayCrash(i, j) && CheckShipCrash(j, i)) {
                is_crash |= 1;
            }
        }

        auto [nxt_p, nxt_d] = g_ships[i].GetNextPD();
        if (is_crash == 0 && CheckShipCollision(nxt_p, nxt_d)) {
            is_crash = 1;
        }

        if (is_crash) {
            g_ships[i].AddInstruction(g_ships[i].ResToIns(rand() % 3));
        } else {
            g_ships[i].AddInstruction(g_ships[i].CalculateInstruction());
        }

        // Position p = g_ships[i].GetPostion();
        // int d = g_ships[i].GetDir();
        // std::pair<Position, int> nxt[3];
        // nxt[0] = {p + DIR[d ^ 1], d};
        // nxt[1] = {p + DIR[d] + DIR[d ^ ((d >> 1) | 2)], d ^ 3 ^ (d >> 1)};
        // nxt[2] = {p + DIR[d ^ 3 ^ (d >> 1)] + DIR[d ^ 3 ^ (d >> 1)], d ^ ((d >> 1) |
        // 2)};
    }
}

bool OceanController::CheckShipCollision(Position nxt, int dir) {
    ShipPositions self = Ship::GetPostions(nxt, dir);
    for (auto [id, other_ship] : g_other_ships) {
        ShipPositions another = Ship::GetPostions(other_ship.position_, other_ship.direction_);
        for (Position& a : self)
            for (Position& b : another)
                if ((a[0] == b[0]) && (a[1] == b[1]) && !g_map.PointInMainChannel(a)) {
                    return true;
                }
    }
    return false;
}


Instruction OceanController::AstarFindNextInstruction(int ship_index) {
    astar.LoadShip(ship_index);

    if (astar.CheckEasy()) {
        return g_ships[ship_index].CalculateInstruction();
    } else {
        return g_ships[ship_index].ResToIns(astar.FindNextStep());
    }
}

int Astar::CalcHcost(Position& pos, int& dir) {
    if (g_ships[ship_index_].GetTar() == -1) {
        return g_map.GetDeliveryPointsDist(pos, dir);
    } else {
        return g_berths[g_ships[ship_index_].GetTar()].GetOceanDistance(pos, dir);
    }
}

bool Astar::CheckStop(Position& pos, int& dir) {
    Position origin_pos = g_ships[ship_index_].GetPostion();
    int origin_dir = g_ships[ship_index_].GetDir();

    return (CalcHcost(pos, dir) < 1) || (CalcHcost(origin_pos, origin_dir) - CalcHcost(pos, dir) > 20);
}

bool Astar::CheckEasy() {
    // return 0;
    return other_ships_pos_.empty();
}

void Astar::LoadShip(int ship_index) {
    other_ships_pos_.clear();
    ship_index_ = ship_index;

    for (int i = 0, ship_num = g_ships.size(); i < ship_num; i++) {
        if (i != ship_index_) {
            ShipPositions sp = g_ships[i].GetPostions();

            for (Position& p : sp) {
                if (GetL1(p, g_ships[ship_index_].GetPostion()) < 10 && !g_map.PointInMainChannel(p)) {
                    other_ships_pos_.push_back(p);
                }
            }
        }
    }

    for (auto& [id, other_ship] : g_other_ships) {
        ShipPositions sp = Ship::GetPostions(other_ship.position_, other_ship.direction_);

        for (Position& p : sp) {
            if (GetL1(p, g_ships[ship_index_].GetPostion()) < 10) {
                other_ships_pos_.push_back(p);
            }
        }
    }
}

bool Astar::CheckCrash(Position& pos, int& dir) {
    ShipPositions sp = Ship::GetPostions(pos, dir);
    int min_L1 = INF;

    for (Position& pos : other_ships_pos_) {
        for (Position& p : sp) {
            min_L1 = std::min(min_L1, GetL1(pos, p));
        }
    }

    return (min_L1 < 1);
}

int Astar::FindNextStep() {
    pos_stack_.clear();  // 位置
    dir_stack_.clear();  // 方向
    dfs_stack_.clear();  // 搜索了哪几步（3 就代表搜过了 0，1 没搜 2）
    res_stack_.clear();  // 去下一步选的是哪一步（0，1 旋转，2 前进）

    pos_stack_.push_back(g_ships[ship_index_].GetPostion());
    dir_stack_.push_back(g_ships[ship_index_].GetDir());
    dfs_stack_.push_back(0);
    res_stack_.push_back(-1);

    for (int max_dfs_times = 1e2; !dfs_stack_.empty() && (!!max_dfs_times); max_dfs_times--) {
        Position pos = pos_stack_.back();
        int dir = dir_stack_.back();
        int& dfs = dfs_stack_.back();
        int& res = res_stack_.back();

        if (CheckStop(pos, dir)) {
            break;
        }

        if (dfs == 7) {
            pos_stack_.pop_back();
            dir_stack_.pop_back();
            dfs_stack_.pop_back();
            res_stack_.pop_back();
        } else {
            next_pos_[0] = pos + DIR[dir] + DIR[dir];
            next_pos_[1] = pos + DIR[dir] + DIR[g_ships[ship_index_].Rot(dir, 0)];
            next_pos_[2] = pos + DIR[dir];

            next_dir_[0] = g_ships[ship_index_].Rot(dir, 0);
            next_dir_[1] = g_ships[ship_index_].Rot(dir, 1);
            next_dir_[2] = dir;

            for (int kb = 0; kb < 3; kb++) {
                if (((dfs >> kb) & 1) || (CheckCrash(next_pos_[kb], next_dir_[kb]))) {
                    next_cost_[kb] = INF;
                } else {
                    next_cost_[kb] = CalcHcost(next_pos_[kb], next_dir_[kb]);
                }
            }

            int next_res = -1, min_cost = INF;

            for (int kb = 0; kb < 3; kb++) {
                if (next_cost_[kb] < min_cost) {
                    min_cost = next_cost_[kb];
                    next_res = kb;
                }
            }

            if (next_res != -1) {
                dfs |= (1 << next_res);
                res = next_res;

                pos_stack_.push_back(next_pos_[next_res]);
                dir_stack_.push_back(next_dir_[next_res]);
                dfs_stack_.push_back(0);
                res_stack_.push_back(-1);
            } else {
                dfs = 7;
                res = -1;
            }
        }
    }

    if (g_frame_id % 100 == 1) {
        fprintf(stderr, "ship value: %d\n", g_ships[ship_index_].GetValue());

        std::cerr << "path print: ";
        for (int i = 0, sz = dfs_stack_.size(); i < sz; i++) {
            // if (i < 10 || (sz - i) < 10)
            fprintf(stderr, "pos(%d, %d) res %d cost %d-> ", pos_stack_[i][0], pos_stack_[i][1],
                    res_stack_[i], CalcHcost(pos_stack_[i], dir_stack_[i]));
        }
        std::cerr << std::endl;

        if (g_ships[ship_index_].GetTar() != -1) {
            Position p = g_berths[g_ships[ship_index_].GetTar()].core_pos_;
            fprintf(stderr, "aim pos: (%d, %d)\n", p[0], p[1]);
        }
    }


    if (dfs_stack_.size() && CheckStop(pos_stack_.back(), dir_stack_.back())) {
        int res = res_stack_.front();

        return (res == -1) ? rand() % 3 : res;
    }

    return rand() % 3;
}

OceanController g_ocean_controller;