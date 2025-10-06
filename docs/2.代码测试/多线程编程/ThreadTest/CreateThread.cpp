#include <iostream>
#include <thread>

// 1. 普通函数作为线程入口
void background_task(int id) {
    std::cout << "线程 " << id << " 正在执行，线程ID: " 
              << std::this_thread::get_id() << std::endl;
}

// 2. Lambda表达式作为线程入口
auto lambda_task = [](const std::string& message) {
    std::cout << "Lambda线程: " << message << std::endl;
};

int main() {
    // 创建并启动线程
    std::thread t1(background_task, 1); // 传递函数指针和参数
    std::thread t2(lambda_task, "Hello from Lambda!"); // 传递Lambda和参数

    // 等待线程完成 (重要！)
    t1.join(); // 主线程阻塞，直到t1执行完毕
    t2.join(); // 主线程阻塞，直到t2执行完毕

    std::cout << "主线程结束。" << std::endl;
    return 0;
}