
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
        DESTINATION lib)