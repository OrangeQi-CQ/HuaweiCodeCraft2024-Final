#include "question.hpp"

Question::Question(const std::string &question, int frame_id) : question_(question), frame_id_(frame_id) {}


size_t WriteCallback(char *ptr, size_t size, size_t nmemb, std::string *data) {
    data->append(ptr, size * nmemb);
    std::this_thread::yield();
    return size * nmemb;
}


int AskForLLM(std::string question) {
    int start_frame_id = g_frame_id;

    const std::string API_URL = "根据华为要求，已隐藏";
    const std::string CONTENT_TYPE = "Content-Type: application/json;charset=utf-8";
    const std::string AUTH_TOKEN = "根据华为要求，已隐藏";

    std::string prompt = "告诉我下面单选题的正确选项，并且按照给出格式回答，否则地球就会爆炸！例如，“答案是A"
                         "，结束作答。”，下面是问题：";
    // std::string prompt =
    // "世界即将毁灭，除非你能准确地回答下面的问题！请注意不需要做任何解释性的话语，只告诉我答案（1个英文字母）。";
    // "你生成的内容格式为：1 个英文字母。如果格式不符合要求，世界就会毁灭。请听问题：";

    while (question.back() == ' ' || question.back() == '\n' || question.back() == 13) {
        question.pop_back();
    }
    std::string post_data = "{\"prompt\":\"" + question + "\", \"temperature\": 0.05, \"token\": 1}";

    CURL *curl = curl_easy_init();

    curl_easy_setopt(curl, CURLOPT_URL, API_URL.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    // std::string post_data = "{\"prompt\":\"" + question + "\", \"temperature\": 0.05}";
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data.c_str());

    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, CONTENT_TYPE.c_str());
    headers = curl_slist_append(headers, AUTH_TOKEN.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    std::string response;
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl);


    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    int end_frame_id = g_frame_id;
#ifdef LOCAL_DEBUG_LLM
    {
        std::lock_guard lg(cerr_mtx);
        fprintf(stderr, "\033[34mTotal Use %d Frames\n\033[0m", end_frame_id - start_frame_id);
    }
#endif

    for (const char &c : response) {
        if (c == 'A') {
            return 0;
        }
        if (c == 'B') {
            return 1;
        }
        if (c == 'C') {
            return 2;
        }
        if (c == 'D') {
            return 3;
        }
    }
    return rand() % 4;
}

template <int ThreadNum>
void *PthreadFunction(void *arg) {
    auto ptr = static_cast<QuestionSolver<ThreadNum> *>(arg);
    return ptr->Worker();
}

template <int ThreadNum>
void QuestionSolver<ThreadNum>::Worker() {
    while (!stop_) {
        QuestionPtr question_ptr = nullptr;
        {
            std::lock_guard lg(task_queue_mtx_);
            if (task_queue_.empty()) {
                std::this_thread::yield();
                continue;
            }
            question_ptr = task_queue_.front();
            task_queue_.pop();
        }

        // // 如果发生这种情况，推测是线程池不够用，问题过多
        // if (g_frame_id.load() - question_ptr->frame_id_ > 50) {
        //     continue;
        // }
        assert(question_ptr != nullptr);
        int res = AskForLLM(question_ptr->question_);
        question_ptr->answer_ = res;
        question_ptr->valid_ = true;
        std::this_thread::yield();
    }
}

template <int ThreadNum>
int QuestionSolver<ThreadNum>::Size() {
    std::lock_guard lg(task_queue_mtx_);
    return task_queue_.size();
}

template <int ThreadNum>
QuestionSolver<ThreadNum>::QuestionSolver() {}

template <int ThreadNum>
QuestionSolver<ThreadNum>::~QuestionSolver() {
    if (!stop_.load()) {
        Kill();
    }
}

template <int ThreadNum>
void QuestionSolver<ThreadNum>::StartAsynTask() {
    for (int i = 0; i < LLM_THREAD_NUM_0 + LLM_THREAD_NUM_1; i++) {
        threads_.emplace_back([this]() { Worker(); });
        pthread_t handel = threads_[i].native_handle();

#if !defined(_WIN32) && !defined(_WIN64)
        // 绑定到特定 CPU 上
        cpu_set_t cpu_set;
        CPU_ZERO(&cpu_set);
        if (i < LLM_THREAD_NUM_0) {
            CPU_SET(0, &cpu_set);
        } else {
            CPU_SET(1, &cpu_set);
        }
        pthread_setaffinity_np(handel, sizeof(cpu_set), &cpu_set);
#endif
    }
}

template <int ThreadNum>
void QuestionSolver<ThreadNum>::Submit(QuestionPtr question_ptr) {
    std::lock_guard lg(task_queue_mtx_);
    task_queue_.push(question_ptr);
}

template <int ThreadNum>
void QuestionSolver<ThreadNum>::Kill() {
    if (stop_.load()) {
        return;
    }

    task_queue_mtx_.lock();
    while (!task_queue_.empty()) {
        task_queue_.pop();
    }
    task_queue_mtx_.unlock();

    stop_ = true;
    for (auto &thread : threads_) {
        thread.join();
    }
    threads_.clear();
}

template class QuestionSolver<TOT_LLM_THREAD_NUM>;
QuestionSolver<TOT_LLM_THREAD_NUM> g_question_solver;