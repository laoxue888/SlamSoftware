cmake_minimum_required(VERSION 3.16)

# 使用vcpkg安装的库
set(CMAKE_TOOLCHAIN_FILE "/opt/vcpkg/scripts/buildsystems/vcpkg.cmake")

project(SlamSoftware VERSION 0.1 LANGUAGES CXX)

add_subdirectory(thirdparty)

set(CMAKE_AUTOUIC ON) 
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

set(OpenCV_ROOT "${VCPKG_INSTALLED_DIR}/x64-linux/share/opencv3")
find_package(OpenCV REQUIRED)

# find_package(Sophus CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Pangolin CONFIG REQUIRED)

# 采用set(CMAKE_AUTOUIC ON)的方式处理ui文件，需要将ui文件夹加入AUTOUIC的搜索目录，这样AUTOUIC处理时才能找到ui文件
list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS "ui")

# 包含项目头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/thirdparty/ORB_SLAM2/include)

set(HEADER_DIR ${CMAKE_SOURCE_DIR}/include)

# 包含源文件目录
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)

# 包含UI文件目录
set(UI_DIR ${CMAKE_SOURCE_DIR}/ui)

set(PROJECT_SOURCES
        ${SOURCE_DIR}/main.cpp
        ${SOURCE_DIR}/mainwindow.cpp
        ${HEADER_DIR}/mainwindow.h
        ${UI_DIR}/mainwindow.ui
        # ${SOURCE_DIR}/trajectory_error.cpp
        # ${HEADER_DIR}/trajectory_error.h
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(SlamSoftware
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET SlamSoftware APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(SlamSoftware SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(SlamSoftware
            ${PROJECT_SOURCES}
        )
    endif()
endif()

# target_link_libraries(SlamSoftware PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Sophus::Sophus ${Pangolin_LIBRARIES})
target_link_libraries(SlamSoftware PRIVATE 
                    Qt${QT_VERSION_MAJOR}::Widgets 
                    ${OpenCV_LIBS}
                    ${EIGEN3_LIBS}
                    ${Pangolin_LIBRARIES}
                    ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so # 从上一级目录开始找
                    ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o.so
                    ${PROJECT_SOURCE_DIR}/thirdparty/ORB_SLAM2/lib/libORB_SLAM2.so
                    )

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.SlamSoftware)
endif()
set_target_properties(SlamSoftware PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS SlamSoftware
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(SlamSoftware)
endif()
