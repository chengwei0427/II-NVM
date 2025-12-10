
add_library(${PROJECT_NAME}.common
        timer/timer.cc
        utility.cpp
        cloudMap.cpp
        algo/static_imu_init.cc
        cloud_convert/cloud_convert.cc
        )

target_link_libraries(${PROJECT_NAME}.common
        ${third_party_libs}
        )

install(TARGETS  ${PROJECT_NAME}.common
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )