add_executable(${PROJECT_NAME}_normal main_normal.cpp  )    #   test loam_tbb_ic_normal
target_link_libraries(${PROJECT_NAME}_normal 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.pre
                         ${PROJECT_NAME}.lio
                         ${third_party_libs}
                        )

add_executable(testNormal testNormal.cpp  )
target_link_libraries(testNormal
                  ${PROJECT_NAME}.common
                  ${PROJECT_NAME}.pre
                  ${PROJECT_NAME}.lio
                  ${third_party_libs}
                 )
