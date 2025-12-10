
add_library(${PROJECT_NAME}.lio
        lio_utils.cpp
        lidarodom.cpp
        lidarodom.cpp
        )

target_link_libraries(${PROJECT_NAME}.lio
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

install(TARGETS  ${PROJECT_NAME}.lio
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
