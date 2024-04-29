#include "../src/question.hpp"

#include <fstream>

int main() {
    std::string qaPath = "./judge/linux/";

    std::ifstream file(qaPath + "qa.txt");

    std::vector<std::string> qaVec;
    std::vector<std::string> ansVec;

    if (file.is_open()) {
        std::string line;
        int lineNum = 0;
        // 使用 std::getline 读取文件的每一行
        while (std::getline(file, line)) {
            if (lineNum % 3 == 0) {
                qaVec.push_back(line);
            } else if (lineNum % 3 == 1) {
                ansVec.push_back(line);
            }

            if (lineNum > 150) {
                break;
            }

            lineNum++;
            // std::cerr << line << std::endl;
        }
    }


    for (int i = 0; i < qaVec.size(); i++) {
        std::string question = qaVec[i];
        std::string ans = ansVec[i];
        int res;

        std::cerr << question << std::endl;
        std::cerr << (res = AskForLLM(question)) << std::endl;
        std::cerr << ans << std::endl;

        if (ans[0] - 'A' != res) {
            std::cerr << "fuc";

            for (; rand() % 20 != 0;)
                std::cerr << "k";

            std::cerr << std::endl;
        }
    }

    // std::string question = " 海上运输中使用的木质包装材料，根据国际标准，需要进行哪种处理？ A. 防水处理 B.
    // 防火处理 C. 熏蒸处理 D. 防虫处理"; std::cout << question << std::endl << AskForLLM(question) <<
    // std::endl;
}