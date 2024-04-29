#pragma once
#include <curl/curl.h>
#include <pthread.h>

#include <functional>
#include <future>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

#include "config.hpp"
#include "utils.hpp"


/**
 * @struct 储存一次问答
 */
struct Question {
    Question(const std::string &question, int frame_id);
    std::atomic_bool valid_{false};  // 是否已经计算完成
    std::string question_;           // 问题
    int answer_;                     // 答案
    int frame_id_;                   // 发出提问的帧号
};
using QuestionPtr = std::shared_ptr<Question>;

/**
 * @brief curl 请求的回调函数
 */
static size_t WriteCallback(char *ptr, size_t size, size_t nmemb, std::string *data);

/**
 * @brief 像 LLM 发送请求，回答问题。
 */
int AskForLLM(std::string question);

/**
 * @brief pthread 的接口
 */
template <int ThreadNum>
void *PthreadFunction(void *arg);

/**
 * @class 用来向大模型提问，内部实现了一个异步线程池
 */
template <int ThreadNum>
class QuestionSolver {
public:
    QuestionSolver();

    ~QuestionSolver();

    /**
     * @brief 启动像大模型提问的线程池
     */
    void StartAsynTask();

    /**
     * @brief 提交一个请求
     */
    void Submit(QuestionPtr question_ptr);

    /**
     * @brief 获取任务的数量
     */
    int Size();

    /**
     * @brief 关闭线程池
     */
    void Kill();

private:
    /**
     * @brief 线程函数
     */
    void Worker();

    std::queue<QuestionPtr> task_queue_;
    std::atomic_bool stop_{false};
    std::vector<std::thread> threads_;
    std::mutex task_queue_mtx_;
};

extern QuestionSolver<LLM_THREAD_NUM_0 + LLM_THREAD_NUM_1> g_question_solver;