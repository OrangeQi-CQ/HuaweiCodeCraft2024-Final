#include "ship.hpp"

std::vector<Ship> g_ships;
std::map<int, bool> g_ship_owned;
std::map<int, OtherShip> g_other_ships;

Ship::Ship(int id, Position pos, int dir, int status)
    : id_(id),
      position_(pos),
      direction_(dir),
      capacity_(g_ship_capacity),
      volume_(g_ship_capacity),
      location_(-1),
      next_berth_(-1),
      status_(status) {}

Ship::Ship(Ship &&other) noexcept
    : id_(other.id_),
      volume_(other.volume_),
      position_(std::move(other.position_)),
      direction_(other.direction_),
      capacity_(other.capacity_),
      status_(other.status_),
      next_berth_(other.next_berth_),
      location_(other.location_),
      value_token_(other.value_token_),
      instructions_(std::move(other.instructions_)) {
    other.direction_ = 0;
    other.capacity_ = 0;
    other.status_ = 0;
    other.next_berth_ = -1;
    other.location_ = 0;
    other.value_token_ = 0;
}

void Ship::InputFrame() {
    int id;
    std::cin >> id;
    g_ship_owned[id] = true;
    capacity_ = g_ship_capacity - g_other_ships[id].cargo_num_;
    position_ = g_other_ships[id].position_;
    direction_ = g_other_ships[id].direction_;
    status_ = g_other_ships[id].status_;

    for (auto &berth : g_berths) {
        if (!berth.InBerth(position_)) {
            continue;
        }
        for (auto berths : g_berth_group_manager.GetAllBerthGroup()) {
            if (!berths.group_member_.count(berth.id_)) {
                continue;
            }
            for (auto col : berths.group_member_) {
                if (col != berth.id_) g_berths[col].SetOurShip(false);
            }
        }
        berth.SetOurShip(true);
    }
    if (g_map.GetDeliveryPointsDist(position_, direction_) < 2 && capacity_ == volume_ && value_token_) {
        // std::cerr << "frame " << g_frame_id << " : clear value" << std::endl;
        ClearValue();
        // next_berth_=0;
    }
}

void Ship::OutputFrame() {
    Instruction ins = GetOutputInstruction();

    std::string s = ins.first;
    int p = ins.second;

    if (!EmptyInstructioins()) {
        instructions_.pop_back();
    }

    if (s == "wait") {
        return;
    }

    std::cout << s << " " << id_;
    if (p != -1) {
        std::cout << " " << p;
    }
    std::cout << std::endl;
}

void Ship::Drive() {
    if (status_ == 1) {
        return;
    }
    if (status_ == 2 && capacity_ && g_berths[location_].CalcValue(1)) {
        Take();
        return;
    }

    if (status_ == 2) {
        instructions_.push_back({"dept", -1});
        return;
    }

    if (capacity_ * g_caplim < volume_) {
        next_berth_ = -1;
        int ins = g_map.GetDeliveryPointsMarch(position_, direction_);
        if (ins < 2) {
            instructions_.push_back({"rot", ins});
        } else if (ins == 2) {
            instructions_.push_back({"ship", -1});
        }
        return;
    }

    int ins = g_berths[next_berth_].GetOceanMarch(position_, direction_);
    if (ins < 2) {
        instructions_.push_back({"rot", ins});
    } else if (ins == 2) {
        instructions_.push_back({"ship", -1});
    }
}


void Ship::Arrive() {
    if (next_berth_ == -1) {
        return;
    }
    if (g_berths[next_berth_].InBerth(position_)) {
        location_ = next_berth_;
        instructions_.push_back({"berth", -1});
        status_ = 1;
        if (location_ == -1) {
            capacity_ = volume_;
        }
    }
}

void Ship::PlanningFrame() {
    Arrive();
    Drive();
}


void Ship::Take() {
    std::pair<int, int> take_result = g_berths[location_].TakeCargo(capacity_);
    capacity_ -= take_result.first;
    value_token_ += take_result.second;
}

Instruction Ship::CalculateInstruction() {
    if (status_ == 1) {
        return {"wait", -1};
    }

    if (status_ == 0) {
        if (next_berth_ != -1 && g_berths[next_berth_].InBerth(position_)) {
            // berth
            SetLoadBerth(next_berth_);

            return {"berth", -1};
        } else {
            // execute last instructions
            if (instructions_.size()) {
                Instruction current_instruction = instructions_.back();
                instructions_.pop_back();
                return current_instruction;
            }

            // move
            if (0) {
                // move to delivery points
                next_berth_ = -1;

                return ResToIns(g_map.GetDeliveryPointsMarch(position_, direction_));
            } else {
                // move to target berth
                if (next_berth_ == -1) {
                    return ResToIns(g_map.GetDeliveryPointsMarch(position_, direction_));
                } else
                    return ResToIns(g_berths[next_berth_].GetOceanMarch(position_, direction_));
            }
        }
    } else if (status_ == 2) {
        // dept or take
        if (location_ == -1) {
            std::cerr << "location error" << std::endl;
        }

        if (capacity_ && next_berth_ != -1) {
            Take();
            // g_berths[location_].SetOurShip(true);
            return {"wait", -1};
        } else {
            // SetLoadBerth(-1);
            // g_berths[location_].SetOurShip(false);
            return {"dept", -1};
        }

        // if (capacity_ && g_berths[location_].CalcValue(1)) {
        //     Take();

        //     return {"wait", -1};
        // } else {
        //     // SetLoadBerth(-1);

        //     return {"dept", -1};
        // }
    }

    return {"wait", -1};
}


int Ship::GetCaNum() const {
    return volume_ - capacity_;
}

Instruction Ship::ResToIns(int res) {
    if (res == 2) {
        return {"ship", -1};
    } else {
        return {"rot", res};
    }
}


int Ship::GetStatus() const {
    return status_;
}

int Ship::GetCap() const {
    return capacity_;
}

void Ship::SetTar(int val) {
    next_berth_ = val;
}

int Ship::GetTar() const {
    return next_berth_;
}

Position Ship::GetPostion() const {
    return position_;
}

ShipPositions Ship::GetPostions(Position pos, int dir) {
    int k = 0;
    std::array<Position, 6> res;
    switch (dir) {
        case 0:
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 3; j++) {
                    res[k++] = pos + Position{i, j};
                }
            }
            break;

        case 1:
            for (int i = -1; i < 1; i++) {
                for (int j = -2; j < 1; j++) {
                    res[k++] = pos + Position{i, j};
                }
            }
            break;

        case 2:
            for (int i = -2; i < 1; i++) {
                for (int j = 0; j < 2; j++) {
                    res[k++] = pos + Position{i, j};
                }
            }
            break;

        case 3:
            for (int i = 0; i < 3; i++) {
                for (int j = -1; j < 1; j++) {
                    res[k++] = pos + Position{i, j};
                }
            }
            break;
    }

    return res;
}


ShipPositions Ship::GetPostions() {
    return GetPostions(position_, direction_);
}

ShipPositions Ship::GetPostionsAfterInstruction(Instruction ins) {
    if (ins.first == "rot") {
        if (ins.second == 1) {
            return GetPostions(position_ + DIR[direction_] + DIR[direction_ ^ 3 ^ (direction_ >> 1)],
                               direction_ ^ (2 | (direction_ >> 1)));
        }
        return GetPostions(position_ + DIR[direction_] + DIR[direction_], direction_ ^ 3 ^ (direction_ >> 1));
    } else if (ins.first == "ship") {
        return GetPostions(position_ + DIR[direction_], direction_);
    } else {
        return GetPostions();
    }
}

int Ship::GetDir() const {
    return direction_;
}


int Ship::Rot(int dir, int rot) {
    int ha = ROT_HASH[dir];

    if (rot == 0) {
        ha = (ha + 1) % 4;
    } else {
        ha = (ha - 1 + 4) % 4;
    }

    return ROT_RANK[ha];

    return ROT_RANK[(ROT_HASH[dir] + rot * rot + rot + 1) % 4];
}


void Ship::AddInstruction(Instruction ins) {
    instructions_.push_back(ins);
}

void Ship::ClearInstruction() {
    instructions_.clear();
}

Instruction Ship::GetOutputInstruction() {
    if (instructions_.size())
        return instructions_.back();
    else
        return {"wait", -1};
}

void Ship::OutputInstructions() {
    for (Instruction &ins : instructions_) {
        std::cerr << ins.first << " ";
        if (ins.second != -1) std::cerr << ins.second << " ";
    }

    std::cerr << std::endl;
}

bool Ship::EmptyInstructioins() {
    return !instructions_.size();
}

void Ship::SetLoadBerth(int berth_id) {
    location_ = berth_id;
}

int Ship::GetLoadBerth() {
    return location_;
}

bool Ship::IsFull() {
    return capacity_ * g_caplim < volume_;
}

int Ship::GetID() {
    return id_;
}

void Ship::AddValue(int num) {
    value_token_ += num;
}

void Ship::ClearValue() {
    value_token_ = 0;
    location_ = -1;
    // if (g_frame_id > 14000)
    //     fprintf(stderr, "ship %d delivery at frame %d\n", id_, g_frame_id);
    // std::cerr << "ship " << std::endl;
}

int Ship::GetValue() const {
    return value_token_;
}

std::pair<Position, int> Ship::GetNextPD() {
    Position p = GetPostion();
    int d = GetDir();
    std::pair<Position, int> nxt[3];
    nxt[0] = {p + DIR[d ^ 1], d};
    nxt[1] = {p + DIR[d] + DIR[d ^ ((d >> 1) | 2)], d ^ 3 ^ (d >> 1)};
    nxt[2] = {p + DIR[d ^ 3 ^ (d >> 1)] + DIR[d ^ 3 ^ (d >> 1)], d ^ ((d >> 1) | 2)};

    Instruction ins = CalculateInstruction();
    if (ins.first == "rot") {
        if (ins.second == 1) {
            return nxt[2];
        }
        return nxt[1];
    } else if (ins.first == "ship") {
        return nxt[0];
    } else {
        return {position_, direction_};
    }
}