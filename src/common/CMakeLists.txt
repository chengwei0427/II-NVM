
add_library(${PROJECT_NAME}.common
        timer/timer.cc
        utility.cpp
        cloudMap.cpp
        algo/static_imu_init.cc
        cloud_convert/cloud_convert.cc
        )

# 使用ament_target_dependencies管理ROS2依赖
ament_target_dependencies(${PROJECT_NAME}.common
  ${ros2_lib}
)

target_link_libraries(${PROJECT_NAME}.common
        ${third_party_libs}
        )

install(TARGETS  ${PROJECT_NAME}.common
        DESTINATION lib)