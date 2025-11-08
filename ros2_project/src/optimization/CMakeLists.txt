cmake_minimum_required(VERSION 3.8)

set(CMAKE_TOOLCHAIN_FILE "/opt/vcpkg/scripts/buildsystems/vcpkg.cmake")
project(optimization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 指定为C++11 版本
set(CMAKE_CXX_STANDARD 11)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(console_bridge REQUIRED)
find_package(image_transport REQUIRED)
set(OpenCV_ROOT "${VCPKG_INSTALLED_DIR}/x64-linux/share/opencv3")
find_package(OpenCV REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Pangolin CONFIG REQUIRED)
# find_package(spdlog CONFIG REQUIRED) # vcpkg install spdlog
find_package(Ceres CONFIG REQUIRED)
find_package(g2o CONFIG REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${OPENGL_INCLUDE_DIR}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../thirdparties/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../thirdparties/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../thirdparties/ORB_SLAM2/lib/libORB_SLAM2.so
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 编译monocular_fisheye节点
add_executable(monocular_fisheye src/monocular_fisheye.cpp)
target_link_libraries(monocular_fisheye
${LIBS}
)

# 编译ceres_demo节点
add_executable(ceres_demo src/ceres_demo.cpp)
target_link_libraries(ceres_demo
Ceres::ceres
)

# 编译g2o_demo节点
add_executable(g2o_demo src/g2o_demo.cpp)
target_link_libraries(g2o_demo
g2o::core g2o::stuff g2o::types_icp g2o::types_sba
)

ament_target_dependencies(
  monocular_fisheye
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

ament_target_dependencies(
  ceres_demo
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

ament_target_dependencies(
  g2o_demo
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)


# 安装节点可执行文件
install(TARGETS
  monocular_fisheye
  ceres_demo
  g2o_demo
  DESTINATION lib/${PROJECT_NAME})


ament_package()
