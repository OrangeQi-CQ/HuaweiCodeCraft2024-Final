#include "frame.hpp"

static void CheckOK() {
    std::string s;
    std::cin >> s;
    assert(s == "OK");
}

void Init() {
    // 初始化地图
    g_map.Init();

    // 初始化泊位
    std::queue<BerthMetaData> berth_metadata_que;
    std::cin >> g_num_berth;

    for (int i = 0; i < g_num_berth; i++) {
        int id, x, y, velocity;
        std::cin >> id >> x >> y >> velocity;
        berth_metadata_que.emplace(id, Position{x, y}, velocity);
    }
    g_num_berth = g_berths.size();
    g_map.InitAllBerths(berth_metadata_que);

    // 寻找最佳连通块
    g_map.FindConnectedComponent();

    // 船的容积
    std::cin >> g_ship_capacity;
    fprintf(stderr, "g_ship_capacity = %d\n", g_ship_capacity);

    CheckOK();

    g_berth_group_manager.InitGroups();

    g_berths[*g_berth_group_manager.GetAllBerthGroup()[ship_tar_hash[0]].group_member_.begin()].SetOurShip(
        true);

    std::cout << "OK" << std::endl;

    g_question_solver.StartAsynTask();
    g_cargo_manager.StartAsynTask();
    volatile auto &pool = GridPool::GetInstance();  // 避免编译器优化，提前分配 GridPool 的堆内存

    int virtual_cnt = 0;
    for (int i = 150; i <= 750; i += 50) {
        for (int j = 150; j <= 750; j += 50) {
            Position pos{i, j};

            if (!g_map.IsField(pos)) {
                continue;
            }

            // 舍弃距离泊位不超过 200 的位置
            auto [berth_id, berth_dist] = g_map.FindNearestBerth(pos);
            if (berth_dist < 200) {
                continue;
            }

            virtual_cnt += 1;
            std::shared_ptr<Grid> distance_grid_ptr = GridPool::GetInstance().Alloc();
            g_map.MultisourceBFS(std::vector<Position>{pos}, *distance_grid_ptr);
            g_cargo_manager.AddVirtualCargo(pos, distance_grid_ptr);

            // std::cerr << "virtual point: " << i << " " << j << " distance to berth: " << berth_dist <<
            // std::endl;
        }
    }
    assert(virtual_cnt < MAX_VIRTUAL_CARGO_NUM);

    std::cerr << "Init OK." << std::endl;
}

static void InputFrame() {
    // 读入货物
    g_cargo_manager.InputFrame();

    // 读入所有机器人信息
    int total_robot_num;
    std::cin >> total_robot_num;
    for (int i = 0; i < total_robot_num; i++) {
        int id, num, x, y;
        OtherRobot rob;
        std::cin >> id >> num >> x >> y;
        rob.id_ = id;
        rob.cargo_num_ = num;
        g_robot_volume[id] = std::max(std::max(1, num), g_robot_volume[id]);
        rob.volume_ = g_robot_volume[id];
        rob.position_ = Position{x, y};
        rob.cargo_change_ = g_other_robots[id].cargo_num_ != num;
        rob.cargo_value_ = g_other_robots[id].cargo_value_;
        g_other_robots[id] = rob;
    }

    // 读入所有轮船信息
    int total_ship_num;
    std::cin >> total_ship_num;
    for (int i = 0; i < total_ship_num; i++) {
        int id, num, x, y, dir, sta;
        OtherShip shi;
        std::cin >> id >> num >> x >> y >> dir >> sta;
        shi.id_ = id;
        shi.cargo_num_ = num;
        shi.position_ = {x, y};
        shi.direction_ = dir;
        shi.status_ = sta;
        shi.cargo_change_ = num - g_other_ships[id].cargo_num_;
        g_other_ships[id] = shi;
    }

    // 读入机器人
    std::cin >> g_robot_cnt;
    for (auto &robot : g_robots) {
        robot.InputFrame();
    }
    for (int i = g_robots.size(); i < g_robot_cnt; i++) {
        int id;
        std::cin >> id;
        g_robot_owned[id] = true;
        Position pos = g_other_robots[id].position_;
        // std::cin >> id >> status >> x >> y;
        g_robots.emplace_back(i, pos, g_self_robot_volume[i]);
    }

    // 读入问题
    int question_num;
    std::cin >> question_num;
    for (int i = 0; i < question_num; i++) {
        int robot_private_id;
        std::string question;
        std::cin >> robot_private_id;
        std::getline(std::cin, question);
        // std::cerr << robot_private_id << " " << question << "\n";
        g_robots[robot_private_id].AskQuestion(question);
#ifdef LOCAL_DEBUG_LLM
        const auto &robot = g_robots[robot_private_id];
        CargoPtr cargo_ptr = robot.GetCargoPtr();
        if (cargo_ptr == nullptr || robot.GetPosition() != cargo_ptr->position_) {
            fprintf(stderr, "robot id = %d\n", robot.id_);
        }
        assert(cargo_ptr != nullptr);
        assert(robot.GetPosition() == cargo_ptr->position_);
#endif
    }

    // for(auto &berth:g_berths){
    //     berth.SetOurShip(false);
    // }

    // 读入轮船
    std::cin >> g_ship_cnt;
    for (auto &ship : g_ships) {
        ship.InputFrame();
    }
    for (int i = g_ships.size(); i < g_ship_cnt; i++) {
        int id, num_cargos, dir, status;
        std::cin >> id;
        Position pos;
        num_cargos = g_other_ships[id].cargo_num_;
        pos = g_other_ships[id].position_;
        dir = g_other_ships[id].direction_;
        status = g_other_ships[id].status_;
        g_ships.emplace_back(i, pos, dir, status);
    }

    for (auto &i : g_robot_owned) {
        g_other_robots.erase(i.first);
    }
    for (auto &i : g_ship_owned) {
        g_other_ships.erase(i.first);
    }

    g_cargo_manager.RefreshCargos();

    for (auto &berth : g_berths) {
        berth.AddShipNum(-berth.GetShipNum());
        for (auto &i : g_other_ships) {
            if (berth.InBerth(i.second.position_)) {
                berth.AddShipNum(1);
            }
        }
    }

    CheckOK();
}

static void PlanFrame() {
    Timer timer("Plan Frame", 15);
    g_land_scheduler.ScheduleRobots();
    g_ocean_scheduler.ScheduleShipsTemp();
    g_land_controller.ControlRobots();
    g_ocean_controller.ControlShips();
}

static void OutputFrame() {
    for (auto &[id, other_robot] : g_other_robots) {
        if (other_robot.cargo_value_.empty()) {
            continue;
        }
        for (auto &berth : g_berths) {
            if (berth.InBerth(other_robot.position_)) {
                while (!other_robot.cargo_value_.empty()) {
                    berth.AddCargos(other_robot.cargo_value_.top());
                    other_robot.cargo_value_.pop();
                }
            }
        }
    }

    for (auto &[id, other_ship] : g_other_ships) {
        if (other_ship.cargo_change_ <= 0) {
            continue;
        }
        for (auto &berth : g_berths) {
            if (berth.InBerth(other_ship.position_)) {
                berth.TakeCargo(other_ship.cargo_change_);
            }
        }
    }

    for (auto &robot : g_robots) {
        robot.OutputFrame();
    }
    for (auto &ship : g_ships) {
        ship.OutputFrame();
    }
}

static void BuyRobotShip() {
    const std::vector<BerthGroup> &all_berth_groups = g_berth_group_manager.GetAllBerthGroup();
    int all_berth_group_num = all_berth_groups.size();

    int gsc = g_ship_cnt % all_berth_group_num;
    int gscm1 = (gsc - 1 + all_berth_group_num) % all_berth_group_num;

    // 希望在这个泊位附近买船
    int buy_ship_berth_id = *all_berth_groups[ship_tar_hash[gsc]].group_member_.begin();

    // 希望在这个泊位附近买机器人
    int buy_robot_berth_id = *all_berth_groups[ship_tar_hash[gscm1]].group_member_.begin();

    // 买船
    if (MAX_SHIP_NUM > 0) {
        // 买第一条船
        if (g_frame_id % 5 == 0 && g_ship_cnt == 0 && g_money >= 8000) {
            fprintf(stderr, "frame %d, buy a ship\n", g_frame_id.load());
            g_map.BuyShip(buy_ship_berth_id);
        }

        // 买其他船
        if (g_frame_id % 10 == 0 && g_ship_cnt < MAX_SHIP_NUM && g_money >= 8000 &&
            (g_robot_cnt / (g_ship_cnt + 1) >= MAX_ROBOT_NUM / (MAX_SHIP_NUM + 1) ||
             g_robot_cnt >= MAX_ROBOT_NUM)) {
            fprintf(stderr, "frame %d, buy a ship\n", g_frame_id.load());
            g_map.BuyShip(buy_ship_berth_id);
        }
    }

    // 买机器人
    if (MAX_ROBOT_NUM > 0) {
        // 第一个机器人一定是另一种
        if (g_robot_cnt <= 0 && g_robot_cnt < MAX_ROBOT_NUM && g_frame_id < 5) {
            g_map.BuyRobot(ROBOT_TYPE ^ 1, buy_ship_berth_id);
            g_self_robot_volume[g_robot_cnt] = (ROBOT_TYPE ^ 1) + 1;
        }

        // 买其他机器人
        if (g_robot_cnt < MAX_ROBOT_NUM && g_frame_id >= 5 && g_frame_id % 2 == 0 && g_money >= 2000) {
            // 希望按比例买机器人
            if ((g_robot_cnt / (g_ship_cnt + 1) < MAX_ROBOT_NUM / (MAX_SHIP_NUM + 1) || g_frame_id <= 100 ||
                 g_ship_cnt >= MAX_SHIP_NUM)) {
                g_map.BuyRobot(ROBOT_TYPE, buy_robot_berth_id);
                // 记录购买的机器人设定的容量。
                g_self_robot_volume[g_robot_cnt] = ROBOT_TYPE + 1;
            }
        }
    }

    if (g_frame_id > 2500) {
        g_caplim = 99;
    }
}

void Control() {
    int cur_frame_id = 0;

    std::vector<int> berth_cargo_cnt(g_berths.size(), 0);
    int last_robot_value = 0;

    int frame_id;

    while (std::cin >> frame_id) {
        cur_frame_id++;
        g_frame_id = frame_id;

        if (g_frame_id % DEBUG_PRINT_FREQUENCY == 0) {
            fprintf(stderr,
                    "[frame]%d  [processed_frame]%d  [money]%d  [robot]%d  [ship]%d  [value_to_berth]%d  "
                    "[task_queue_size]%d  [cargo_manager_size]%d",
                    g_frame_id.load(), cur_frame_id, g_money, g_robot_cnt, g_ship_cnt, Robot::value_sent_,
                    (int)g_question_solver.Size(), (int)g_cargo_manager.cargo_ptr_vec_.size());
            std::cerr << std::endl;
            std::cerr << "CargoManager Total Init Cargos Num: " << CargoManager::tot_bfs_cnt_ << std::endl;
            std::cerr << "CargoManager Total Erase Cargos Num: " << CargoManager::tot_erase_cnt_ << std::endl;

            std::cerr << "ship value: ";
            for (Ship &ship : g_ships) {
                std::cerr << ship.GetValue() << " ";
            }
            std::cerr << std::endl;
        }
        std::cin >> g_money;

        InputFrame();
        PlanFrame();
        OutputFrame();
        BuyRobotShip();

        std::cout << "OK" << std::endl;

        // 在一帧的输出都结束后，再进行 cargo_manager 的任务队列的处理
        g_cargo_manager.UpdateAsynTaskList();
    }

    fprintf(stderr, "Process %d frames.\n", cur_frame_id);
    fprintf(stderr, "Value of cargos sent to g_berths: %d.\n", Robot::value_sent_);
    fprintf(stderr, "Value of cargos token by g_ships: %d.\n", g_value_token);
    fprintf(stderr, "Total buy %d g_robots, %d g_ships.\n", g_robot_cnt, g_ship_cnt);
    fprintf(stderr, "Total %d cargos appear.\n", g_cargo_num);
    fprintf(stderr, "Total earn %d g_money\n", g_money - 25000 + g_robot_cnt * 2000 + g_ship_cnt * 8000);

    for (const auto &berth : g_berths) {
        fprintf(stderr, "Berth %d: value %d\tnum %d.\n", berth.id_, berth.tot_value, berth.tot_cnt);
    }
    for (const auto &ship : g_ships) {
        fprintf(stderr, "Ship %d remain %d cargos.\n", ship.id_, ship.GetCaNum());
    }

    g_question_solver.Kill();
}