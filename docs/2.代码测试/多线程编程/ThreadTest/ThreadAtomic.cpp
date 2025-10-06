#include <atomic>
#include <thread>
#include <iostream>

std::atomic<int> atomic_counter(0);

void atomic_increment(int iterations) {
    for (int i = 0; i < iterations; ++i) {
        ++atomic_counter; // 原子操作，线程安全
    }
}

int main() {
    std::thread t1(atomic_increment, 100000);
    std::thread t2(atomic_increment, 100000);

    t1.join();
    t2.join();

    // 正确输出 200000
    std::cout << "最终计数器值: " << atomic_counter.load() << std::endl;
    return 0;
}