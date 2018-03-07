add_executable( my_node LKtracking.cpp)
target_link_libraries( my_node ${THIRD_PARTY_LIBS} ${PROJECT_NAME})

add_executable( ORBtracker ORBtracking.cpp)
target_link_libraries( ORBtracker ${THIRD_PARTY_LIBS} ${PROJECT_NAME})