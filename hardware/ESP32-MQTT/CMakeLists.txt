# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)


option(HOST_BACKUP "Backup to the host server" OFF) # ON or OFF

if(HOST_BACKUP)
    #备份到宿主机: 要清理一次运行后再运行编译后才可以实现备份
    set(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
    set(SAVE_DIR /home/xsf/hgfs/SlamSoftwareHardware/ESP32MQTTVideo)

    set(BACKUPLIST .vscode main CMakeLists.txt dependencies.lock pytest_mqtt_tcp.py README.md sdkconfig sdkconfig.ci sdkconfig.ci.p4_eppp
                    sdkconfig.ci.p4_wifi sdkconfig.ci.ppp_connect sdkconfig.old)

    foreach(item ${BACKUPLIST})
        execute_process(COMMAND rm -rf ${SAVE_DIR}/${item})
        execute_process(COMMAND rsync -av ${PROJECT_DIR}/${item} ${SAVE_DIR})
    endforeach()

endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mqtt_tcp)
