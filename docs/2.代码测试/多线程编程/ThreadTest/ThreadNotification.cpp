#include <queue>
#include <condition_variable>
#include <thread>
#include <mutex>
#include <iostream>

std::queue<int> data_queue;
std::mutex queue_mutex;
std::condition_variable data_cond;

// 生产者线程
void data_producer() {
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            data_queue.push(i);
            std::cout << "生产数据: " << i << std::endl;
        }
        data_cond.notify_one(); // 通知一个等待的消费者
    }
}

// 消费者线程
void data_consumer() {
    while (true) {
        std::unique_lock<std::mutex> lock(queue_mutex);
        // 等待条件成立：队列非空。等待时会自动释放锁，被唤醒后重新获取锁。
        data_cond.wait(lock, []{ return !data_queue.empty(); });

        int data = data_queue.front();
        data_queue.pop();
        lock.unlock(); // 尽早释放锁

        std::cout << "消费数据: " << data << std::endl;
        if (data == 9) break; // 结束条件
    }
}

int main() {
    std::thread producer(data_producer);
    std::thread consumer(data_consumer);

    producer.join();
    consumer.join();

    return 0;
}