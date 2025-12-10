add_executable(${PROJECT_NAME} main.cpp  )  
target_link_libraries(${PROJECT_NAME} 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.lio
                         ${third_party_libs}
                        )

