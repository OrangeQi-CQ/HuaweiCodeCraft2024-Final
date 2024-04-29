#include "../src/question.hpp"

int main() {
    std::string question = "下列哪种材质常用于制作快递包装中的密封袋，具有防水和防撕裂的特点？ A. 塑料袋 B. 纸质袋 C. 布质袋 D. 金属袋";
    std::cout << question << std::endl << AskForLLM(question) << std::endl;
}