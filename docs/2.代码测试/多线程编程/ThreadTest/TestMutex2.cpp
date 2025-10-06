#include <thread>
#include <mutex>
#include <vector>
#include <iostream>

std::mutex g_mutex; // 全局互斥锁
int shared_counter = 0;

void safe_increment(int iterations) {
    for (int i = 0; i < iterations; ++i) {
        std::lock_guard<std::mutex> lock(g_mutex); // 构造即加锁
        ++shared_counter;
        // lock 析构时自动解锁
    }
}

int main() {
    // 使用安全的增量函数
    shared_counter = 0; // 重置计数器
    std::thread t1(safe_increment, 100000);
    std::thread t2(safe_increment, 100000);

    t1.join();
    t2.join();

    std::cout << "最终计数器值: " << shared_counter << std::endl; // 正确输出 200000
    return 0;
}
