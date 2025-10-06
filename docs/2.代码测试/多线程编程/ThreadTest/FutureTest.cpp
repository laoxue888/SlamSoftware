#include <future>
#include <iostream>
#include <thread>

// 模拟一个耗时的计算任务
int compute_heavy_task() {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return 42;
}

int main() {
    // 异步启动任务
    std::future<int> result = std::async(std::launch::async, compute_heavy_task);

    // ... 主线程可以同时做其他工作 ...
    for(int i = 0; i < 5; ++i) {
        std::cout << "主线程工作中..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // 获取异步任务结果（如果需要，会阻塞等待）
    int value = result.get();
    std::cout << "异步任务结果为: " << value << std::endl;
    return 0;
}