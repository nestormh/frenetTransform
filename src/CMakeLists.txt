add_executable(testFrenet frenettransform.cpp frenetplotter.cpp 
    /home/nestor/Dropbox/ros/hydro/ull-isaatc/src/grull_ackermann_base_local_planner/src/trajectory.cpp
    main.cpp 
)
  
include_directories(testFrenet
#     GPULibSVM
    ${EIGEN3_INCLUDE_DIR}
#     ${PCL_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIR}
#     ${Boost_INCLUDE_DIR}
#     emon.a
#     ${CUDA_INCLUDE_DIRS}
    /usr/local/dislin
    /opt/ros/hydro/include
    /home/nestor/Dropbox/ros/hydro/ull-isaatc/src/grull_ackermann_base_local_planner/include/
)

link_directories(testFrenet
    /usr/local/dislin/lib
)

target_link_libraries(testFrenet
  ${EIGEN3_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${CUDA_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${PROTOBUF_LIBRARIES}
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_
#   ${CGAL_CGAL_LIBRARY}
#   ${GMP_LIBRARIES}
#   ${CUDA_CURAND_LIBRARIES}
#   ${OPENGL_LIBRARIES}
#   ${GLUT_LIBRARIES}
#   emon
#     /usr/local/dislin/libdiscpp.so
    /usr/local/dislin/lib/dislin-10.3.a
    /usr/local/dislin/libdislin.a
    /usr/local/dislin/libdislin.so
    /usr/local/dislin/libdislnc.a
    /usr/local/dislin/lib/dislin_d-10.3.so
    /usr/local/dislin/lib/discpp-10.3.so
)