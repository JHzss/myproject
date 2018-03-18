add_executable( LK LKtracking.cpp)
target_link_libraries( LK ${THIRD_PARTY_LIBS} ${PROJECT_NAME})

#add_executable( ORBtracker ORBtracking.cpp)
#target_link_libraries( ORBtracker ${THIRD_PARTY_LIBS} ${PROJECT_NAME})
#
#add_executable( test_camera test_camera.cpp)
#target_link_libraries( test_camera ${THIRD_PARTY_LIBS} ${PROJECT_NAME})