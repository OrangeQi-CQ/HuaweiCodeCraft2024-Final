#include <numeric>

#if !defined(_WIN32) && !defined(_WIN64)
#include <sched.h>
#include <unistd.h>
#endif

#include "frame.hpp"


int main() {
#ifdef USE_MFMC
    std::cerr << "DEFINE USE_MFMC" << std::endl;
#endif

#ifdef AVOID_SWING
    std::cerr << "DEFINE AVOID_SWING" << std::endl;
#endif

#ifdef RAND
    unsigned rand_seed = static_cast<unsigned>(time(nullptr));
    srand(rand_seed);
    std::cerr << "srand = " << rand_seed << std::endl;
#endif

#ifdef LOCAL
    std::cerr << "DEFINE LOCAL" << std::endl;
#endif

#ifdef DEBUG
    fprintf(stderr, "DEFIND DEBUG\n");
#endif

#if !defined(_WIN32) && !defined(_WIN64)
    // 将 main 线程绑定到 cpu 0 上
    pthread_t main_thread_id = pthread_self();
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(0, &cpu_set);
    pthread_setaffinity_np(main_thread_id, sizeof(cpu_set), &cpu_set);

    fprintf(stderr, "Set CPU!\n");
#endif

    Init();     // 初始化
    Control();  // 控制所有帧

    return 0;
}