add_executable(${PROJECT_NAME} main.cpp  )  

ament_target_dependencies(${PROJECT_NAME} ${ros2_lib})

target_link_libraries(${PROJECT_NAME} 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.lio
                         ${third_party_libs}
                        )

install(TARGETS ${PROJECT_NAME}   
    DESTINATION lib/${PROJECT_NAME}
)