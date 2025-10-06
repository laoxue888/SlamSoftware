cmake_minimum_required(VERSION 3.10)
project(ThreadTest)

set(CMAKE_CXX_STANDARD 11)

add_executable(CreateThread CreateThread.cpp)
add_executable(TestMutex TestMutex.cpp)
add_executable(TestMutex2 TestMutex2.cpp)
add_executable(ThreadNotification ThreadNotification.cpp)
add_executable(ThreadAtomic ThreadAtomic.cpp)
add_executable(LockGuad LockGuad.cpp)
add_executable(FutureTest FutureTest.cpp)