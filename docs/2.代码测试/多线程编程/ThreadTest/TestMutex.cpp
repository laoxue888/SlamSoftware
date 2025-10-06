#include <thread>
#include <mutex>
#include <vector>
#include <iostream>

std::mutex g_mutex; // 全局互斥锁
int shared_counter = 0;

void increment_counter(int iterations) {
    for (int i = 0; i < iterations; ++i) {
        g_mutex.lock();   // 进入临界区前加锁
        ++shared_counter; // 安全地修改共享数据
        g_mutex.unlock(); // 离开临界区后解锁
    }
}

int main() {
    std::thread t1(increment_counter, 100000);
    std::thread t2(increment_counter, 100000);

    t1.join();
    t2.join();

    std::cout << "最终计数器值: " << shared_counter << std::endl; // 正确输出 200000
    return 0;
}
