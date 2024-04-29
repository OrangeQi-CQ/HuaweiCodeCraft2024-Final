#include "cargo.hpp"

CargoManager g_cargo_manager;

Cargo::Cargo(Position pos, int value, int appear_frame, std::shared_ptr<Grid> dist_gird_ptr)
    : position_(pos), value_(value), appear_frame_(appear_frame), dist_grid_ptr_(dist_gird_ptr) {
    std::vector<Position> src = {position_};
}

Cargo::Cargo(Cargo &&other) noexcept
    : value_(other.value_),
      appear_frame_(other.appear_frame_),
      position_(std::move(other.position_)),
      id_(other.id_),
      delete_(other.delete_),
      dist_grid_ptr_(std::move(other.dist_grid_ptr_)),
      is_occupied_(other.is_occupied_) {
    other.id_ = 0;
    other.delete_ = false;
    other.dist_grid_ptr_ = nullptr;
    other.is_occupied_ = false;
}

Cargo::~Cargo() {
    GridPool::GetInstance().Free(dist_grid_ptr_);
}

bool Cargo::IsVirtual() const noexcept {
    return is_virtual_;
}

int Cargo::GetValue() const noexcept {
    return value_;
}

int Cargo::GetID() const noexcept {
    return this->id_;
}

void Cargo::SetID(int id) noexcept {
    this->id_ = id;
}

int Cargo::GetDistance(Position pos) const noexcept {
    return dist_grid_ptr_->Get(pos);
}

int Cargo::GetDirection(Position pos) const noexcept {
    return dist_grid_ptr_->GetDescentDirection(pos);
}

const Grid &Cargo::GetDistanceGrid() const noexcept {
    return *dist_grid_ptr_;
}

void Cargo::Delete() noexcept {
    delete_ = true;
}

bool Cargo::IsDeleted() const noexcept {
    return delete_;
}

int Cargo::GetRemainingLife() const noexcept {
    return (appear_frame_ + 1000) - g_frame_id;
}

void Cargo::SetIsOccupied(bool be) noexcept {
    is_occupied_ = be;
}

bool Cargo::IsOccupied() const noexcept {
    return is_occupied_;
}

void *CargoManagerThreadFunc(void *arg) {
    CargoManager *g_cargo_manager = static_cast<CargoManager *>(arg);
    g_cargo_manager->InitCargoDistGrid();
    return nullptr;
}

CargoManager::CargoManager() : cargo_exist_grid_(0) {}

CargoManager::~CargoManager() {
    {
        std::lock_guard lg(task_list_mtx_);
        task_list_.clear();
    }
    {
        std::lock_guard lg(ready_queue_mtx_);
        while (!ready_queue_.empty()) {
            ready_queue_.pop();
        }
    }

    for (auto &thread : cargo_init_threads_) {
        thread.join();
    }
}

void CargoManager::AddVirtualCargo(const Position &position, std::shared_ptr<Grid> distance_grid_ptr) {
    int id = cargo_ptr_vec_.size();
    cargo_ptr_vec_.push_back(std::make_shared<Cargo>(position, 0, -1, distance_grid_ptr));
    cargo_ptr_vec_.back()->SetID(id);
    cargo_ptr_vec_.back()->is_virtual_ = true;
}

int CargoManager::tot_bfs_cnt_ = 0;
int CargoManager::tot_erase_cnt_ = 0;

void CargoManager::StartAsynTask() {
    for (int i = 0; i < CARGO_INIT_THREAD_NUM; i++) {
        cargo_init_threads_.emplace_back([this]() {
            while (true) {
                InitCargoDistGrid();
            }
        });
    }

#if !defined(_WIN32) && !defined(_WIN64)
    // 绑定到 CPU 1 上
    for (auto &thread : cargo_init_threads_) {
        pthread_t handle = thread.native_handle();
        cpu_set_t cpu_set;
        CPU_ZERO(&cpu_set);
        CPU_SET(1, &cpu_set);
        int ret = pthread_setaffinity_np(handle, sizeof(cpu_set), &cpu_set);
        assert(ret == 0);
    }
#endif
}

void CargoManager::InputFrame() {
    // 读入这一帧新增的货物
    int cnt;
    std::cin >> cnt;
    g_cargo_num += cnt;

    while (cnt--) {
        int x, y, value;  // NOLINT
        std::cin >> x >> y >> value;

        if (value == 0) {
            cargo_exist_grid_[x][y] = 0;
            continue;
        }

        // 只要是高级货，当帧立即搜索
        if (value > MIN_ADVANCED_CARGO_VALUE) {
            // std::cerr << "frame " << g_frame_id.load() << " advanced cargo " << value << std::endl;

            std::shared_ptr<Grid> grid_ptr = GridPool::GetInstance().Alloc();
            Position pos{x, y};
            g_map.MultisourceBFS(std::vector{pos}, *grid_ptr, CARGO_BFS_MAX_DEPTH);
            CargoPtr cargo_ptr = std::make_shared<Cargo>(pos, value, g_frame_id.load(), grid_ptr);
            cargo_ptr_vec_.push_back(cargo_ptr);
            tot_bfs_cnt_ += 1;
        }

        cargo_exist_grid_[x][y] = 1;
        cargo_join_queue_.push(std::array<int, 4>{x, y, value, g_frame_id});
    }
}

void CargoManager::UpdateAsynTaskList() {
    // 将货物缓存中的货物提交到任务队列
    if (task_list_mtx_.try_lock()) {
        while (!cargo_join_queue_.empty()) {
            auto [x, y, value, appear_frame_id] = cargo_join_queue_.front();
            cargo_join_queue_.pop();

            int dist_to_berth = INF;
            for (const auto &berth : g_berths) {
                dist_to_berth = std::min(dist_to_berth, berth.GetLandDistance(Position{x, y}));
            }

            task_list_.push_back(CargoMetadata(Position{x, y}, value, appear_frame_id, dist_to_berth));
        }
        task_list_mtx_.unlock();
    }

    // 将处理好的货物加入 vector
    if (ready_queue_mtx_.try_lock()) {
        int max_cnt = 10;
        while (!ready_queue_.empty()) {
            auto cargo_ptr = ready_queue_.front();
            ready_queue_.pop();
            if (cargo_exist_grid_.Get(cargo_ptr->position_)) {
                cargo_ptr_vec_.push_back(cargo_ptr);
                // fprintf(stderr, "\033[33mCargo Delay %d frames\033[0m", g_frame_id.load() -
                // cargo_ptr->appear_frame_); std::cerr << std::endl;
            }
        }
        ready_queue_mtx_.unlock();
    }

    // 若货物超出数量上限，删掉价值最低的若干个
    std::sort(cargo_ptr_vec_.begin(), cargo_ptr_vec_.end(), [this](const CargoPtr &lhs, const CargoPtr &rhs) {
        // 虚拟点是一定不能被删掉的
        if (lhs->IsVirtual() && !rhs->IsVirtual()) {
            return true;
        }
        if (!lhs->IsVirtual() && rhs->IsVirtual()) {
            return false;
        }
        // 按价值排序
        return lhs->value_ > rhs->value_;
    });

    while (cargo_ptr_vec_.size() > MAX_TOT_CARGO_NUM) {
        cargo_ptr_vec_.pop_back();
    }
}

void CargoManager::RefreshCargos() {
    // 暂存其他队伍的机器人位置
    std::set<Position> otropo;
    for (auto &[id, robot] : g_other_robots) {
        otropo.insert(robot.position_);
    }

    // 刷新货物数组，扔掉剩余寿命不足或者已经被取走的货物
    std::vector<CargoPtr> cargos_new;
    for (const auto &cargo : cargo_ptr_vec_) {
        // 虚拟货物一定保留
        if (cargo->IsVirtual()) {
            cargos_new.push_back(cargo);
            continue;
        }

        if (cargo->GetRemainingLife() > 20 && !cargo->IsDeleted() &&
            cargo_exist_grid_.Get(cargo->position_) == 1) {
            cargo->SetIsOccupied(otropo.count(cargo->position_));
            cargos_new.push_back(cargo);
        } else {
            for (auto &[id, other_robot] : g_other_robots) {
                if (g_map.Manhattan(other_robot.position_, cargo->position_) <= 1 &&
                    other_robot.cargo_change_) {
                    other_robot.cargo_value_.push(cargo->value_);
                }
            }
        }
    }
    std::swap(cargos_new, cargo_ptr_vec_);

    // 刷新货物 id
    int id = 0;
    for (auto &cargo : cargo_ptr_vec_) {
        cargo->SetID(id++);
    }
}

std::vector<CargoPtr> CargoManager::GetAllCargoPtr() const {
#ifdef LOCAL_TEST
    /**
     * @test 检查一下所有的货物有没有重复
     */
    std::set<int> st;
    for (auto &cargo_ptr : cargo_ptr_vec_) {
        assert(st.count(cargo_ptr->GetID()) == 0);
        st.insert(cargo_ptr->GetID());
    }
#endif
    return cargo_ptr_vec_;
}

void CargoManager::InitCargoDistGrid() {
    std::optional<CargoMetadata> cargo_metadata;
    {
        std::lock_guard lg(task_list_mtx_);

        // 删掉比较老的货物
        for (auto it = task_list_.begin(); it != task_list_.end();) {
            auto lst = it;
            it++;
            if (g_frame_id > lst->appear_frame_ + 50 || cargo_exist_grid_.Get(lst->position_) == 0) {
                task_list_.erase(lst);
                tot_erase_cnt_ += 1;
            }
        }

        // 没货物了就退出（原内存 bug）
        if (task_list_.empty()) {
            return;
        }

        // 选出一个合适的货物进行 bfs
        auto it = std::max_element(task_list_.begin(), task_list_.end(),
                                   [&](const CargoMetadata &lhs, const CargoMetadata &rhs) {
                                       return static_cast<double>(lhs.value_) / lhs.dist_to_berth_ <
                                              static_cast<double>(rhs.value_) / rhs.dist_to_berth_;
                                   });

        cargo_metadata = *it;
        task_list_.erase(it);
    }

    // ? 及时 yield 或许会好一些
    std::this_thread::yield();

    {
#ifdef LOCAL_TIMER
        // 测试耗时
        // volatile Timer timer("Cargo BFS", 1);
#endif  // LOCAL_TIMER
        auto [pos, value, appear_frame, dist_to_berth] = cargo_metadata.value();
        std::shared_ptr<Grid> grid_ptr = GridPool::GetInstance().Alloc();

        g_map.MultisourceBFS(std::vector{pos}, *grid_ptr, CARGO_BFS_MAX_DEPTH);
        CargoPtr cargo_ptr = std::make_shared<Cargo>(pos, value, appear_frame, grid_ptr);
        {
            std::lock_guard lg(ready_queue_mtx_);
            ready_queue_.push(cargo_ptr);
        }

        tot_bfs_cnt_ += 1;
    }

    // ? 及时 yield 或许会好一些
    std::this_thread::yield();
}

std::vector<CargoPtr> g_virtual;