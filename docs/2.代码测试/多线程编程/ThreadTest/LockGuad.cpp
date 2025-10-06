#include <iostream>
#include <mutex>
#include <thread>
 
std::mutex g_mutex; // 全局互斥锁
int shared_counter = 0;
 
void worker(int iterations)
{
    std::lock_guard<std::mutex> lg(g_mutex);  // lock_guard 方式上锁
    for (int i = 0; i < iterations; ++i) {
        ++shared_counter; // 安全地修改共享数据
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "worker thread is done." << std::endl;
}  // lock_guard 不支持手动解锁，会在此自动释放锁
 
void another_worker(int iterations)
{
    std::unique_lock<std::mutex> ul(g_mutex);  // unique_lock 方式上锁
    for (int i = 0; i < iterations; ++i) {
        ++shared_counter; // 安全地修改共享数据
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "another worker thread is done." << std::endl;
    ul.unlock();  // 手动释放锁
    //do something...
}  // 如果锁未释放，unique_lock 会在此自动释放锁
 
int main()
{
    std::thread t1(worker, 100000);
    std::thread t2(another_worker, 100000);
    t1.join();
    t2.join();

    std::cout << "最终计数器值: " << shared_counter << std::endl; // 正确输出 200000
    return 0;
}