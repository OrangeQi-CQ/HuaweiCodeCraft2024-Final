#include "robot.hpp"

std::vector<Robot> g_robots;
std::map<int, int> g_robot_volume;
std::map<int, int> g_self_robot_volume;
std::map<int, bool> g_robot_owned;

Robot::Robot(int id, Position pos, int vol) : id_(id), position_(pos), volume_(vol) {}

Robot::Robot(Robot &&other) noexcept
    : id_(other.id_),
      cargo_num_(other.cargo_num_),
      volume_(other.volume_),
      target_berth_id_(other.target_berth_id_),
      position_(std::move(other.position_)),
      instructions_(std::move(other.instructions_)),
      cargo_ptr_(std::move(other.cargo_ptr_)),
      move_dir_(other.move_dir_),
      lst_instruction_(std::move(other.lst_instruction_)),
      question_ptr_(std::move(other.question_ptr_)),
      addtional_target_(std::move(other.addtional_target_)) {}

int Robot::value_sent_ = 0;

void Robot::InputFrame() {
    int id;
    std::cin >> id;
    g_robot_owned[id] = true;
    assert(id == id_);
    cargo_num_ = g_other_robots[id].cargo_num_;
    position_ = g_other_robots[id].position_;
    addtional_target_ = nullptr;

    if (FullCargo()) {
        assert(GetCargoPtr() != nullptr);
    }

    move_dir_ = 4;
}

void Robot::OutputFrame() {
    // 到达货物所在地，尝试取货，并向大模型咨询
    if (!FullCargo() && GetCargoPtr() != nullptr && position_ == GetCargoPtr()->position_) {
        // 如果没有问题，就提问
        if (question_ptr_ == nullptr) {
            std::cout << "get " << id_ << std::endl;
#ifdef LOCAL_DEBUG_LLM
            fprintf(stderr, "\033[31mframe id %d, robot %d ask for llm\n\033[0m", g_frame_id.load(), id_);
#endif
            GetCargoPtr()->Delete();
            cargo_num_++;
            return;
        }

        // 等待大模型回答
        if (!question_ptr_->valid_) {
            return;
        }

        AnswerQuestion();
#ifdef LOCAL_DEBUG_LLM
        fprintf(stderr, "\033[32mframe_id %d, robot %d get answer\n\033[0m", g_frame_id.load(), id_);
#endif
        question_ptr_ = nullptr;
        // cargo_num_++;
        // GetCargoPtr()->Delete();
        assert(cargo_ptr_.top()->IsDeleted());
    }

    // 到达泊位，送货
    // if (cargo_num_ && target_berth_id_ != -1) {
    //     std::cerr << "ops" << std::endl;
    //     while(1);
    // }
    if (cargo_num_ && target_berth_id_ != -1 && g_berths[target_berth_id_].InBerth(position_)) {
        // while(1);
        while (cargo_ptr_.size() > cargo_num_)
            cargo_ptr_.pop();
        std::cout << "pull " << id_ << std::endl;
        // fprintf(stderr, "robot %d sent value %d\n", id_, cargo_ptr_.top()->value_);
        Robot::value_sent_ += cargo_ptr_.top()->GetValue();
        g_berths[target_berth_id_].AddCargos(cargo_ptr_.top()->GetValue());
        cargo_num_--;
        SetCargoPtr(nullptr);
        if (cargo_num_ == 0) target_berth_id_ = -1;
        lst_instruction_ = "pull";
        return;
    }

    // 发出移动指令
    if (move_dir_ != 4) {
        std::cout << "move " << id_ << " " << move_dir_ << std::endl;
        move_dir_ = 4;
        lst_instruction_ = "move";
        return;
    }
}

bool Robot::FullCargo() const noexcept {
    return cargo_num_ == volume_;
}

Position Robot::GetPosition() const noexcept {
    return position_;
}

void Robot::SetCargoPtr(const CargoPtr &cargo) noexcept {
    while (cargo_ptr_.size() > cargo_num_) {
        cargo_ptr_.pop();
    }
    if (cargo == nullptr) {
        return;
    }
    // if (cargo->IsVirtual()) {
    //     std::cerr << g_frame_id.load() << " Robot " << id_ << " set virtual point " << cargo->GetID() <<
    //     std::endl;
    // } else {
    //     std::cerr << g_frame_id.load() << " Robot " << id_ << " set real point " << cargo->GetID() <<
    //     std::endl;
    // }
    cargo_ptr_.push(cargo);
}

CargoPtr Robot::GetCargoPtr() const noexcept {
    assert(cargo_ptr_.size() <= cargo_num_ + 1);
    if (cargo_ptr_.size() <= cargo_num_ && !FullCargo()) {
        return nullptr;
    }
    // if(cargo_ptr_.empty()) return nullptr;
    return cargo_ptr_.top();
}

void Robot::SetTargetBerthID(int id) noexcept {
    target_berth_id_ = id;
}

int Robot::GetTargetBerthID() const noexcept {
    return target_berth_id_;
}

void Robot::SetMoveDirection(int direction) noexcept {
    move_dir_ = direction;
}

int Robot::GetMoveDirection() noexcept {
    return move_dir_;
}

void Robot::AskQuestion(std::string question) noexcept {
    if (question_ptr_ != nullptr) {
        return;
    }
    QuestionPtr question_ptr = std::make_shared<Question>(question, g_frame_id.load());
    question_ptr_ = question_ptr;
    g_question_solver.Submit(question_ptr);
}

void Robot::AnswerQuestion() noexcept {
    int ans = question_ptr_->answer_;
    printf("ans %d %d\n", id_, ans);

#ifdef LOCAL_DEBUG_LLM
    assert(question_ptr_->valid_);
    fprintf(stderr, "ans %d %d\n", id_, ans);
#endif
}

void Robot::SetAddtionalTarget(CargoPtr car) noexcept {
    addtional_target_ = car;
}

CargoPtr Robot::GetAddtionalTarget() const noexcept {
    return addtional_target_;
}

bool Robot::IsWaitingForLLM() const noexcept {
    return question_ptr_ != nullptr;
}

std::map<int, OtherRobot> g_other_robots;