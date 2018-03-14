add_executable( my_node LKtracking.cpp test_camera.cpp)
target_link_libraries( my_node ${THIRD_PARTY_LIBS} ${PROJECT_NAME})

add_executable( ORBtracker ORBtracking.cpp)
target_link_libraries( ORBtracker ${THIRD_PARTY_LIBS} ${PROJECT_NAME})

add_executable( test_camera ORBtracking.cpp)
target_link_libraries( test_camera ${THIRD_PARTY_LIBS} ${PROJECT_NAME})