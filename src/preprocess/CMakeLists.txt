
add_library(${PROJECT_NAME}.pre
        cloud_convert2.cc
        )

target_link_libraries(${PROJECT_NAME}.pre
        ${third_party_libs}
        )

install(TARGETS  ${PROJECT_NAME}.pre
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )