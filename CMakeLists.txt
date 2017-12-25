cmake_minimum_required(VERSION 2.8.3)
project(test_svo2)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vs_common
  cv_bridge
  image_transport
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(
    svo2.0/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

set(SVO_LIBS
    # ${PROJECT_SOURCE_DIR}/svo2.0/lib/libeigen_catkin.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libeigen_checks.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libfast.so
    # ${PROJECT_SOURCE_DIR}/svo2.0/lib/libgflags_catkin.so
    # ${PROJECT_SOURCE_DIR}/svo2.0/lib/libgflags_nothreads.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libgflags.so
    # ${PROJECT_SOURCE_DIR}/svo2.0/lib/libglog_catkin.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libglog.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libopengv.so
    # ${PROJECT_SOURCE_DIR}/svo2.0/lib/libpangolin_catkin.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libpangolin.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/librandom_generators.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/librpg_common_ros.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/librpg_common.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libsvo_common.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libsvo_direct.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libsvo_img_align.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libsvo.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libsvo_test_utils.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libsvo_tracker.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libvikit_cameras.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libvikit_common.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libvikit_ros.so
    ${PROJECT_SOURCE_DIR}/svo2.0/lib/libvikit_solver.so
    )

add_executable(demo_play_video src/demo_play_video.cpp src/svo_factory.cpp)
target_link_libraries(demo_play_video ${catkin_LIBRARIES} ${SVO_LIBS} ${OpenCV_LIBS})

add_executable(demo_mono_node src/demo_mono_node.cpp src/svo_factory.cpp)
target_link_libraries(demo_mono_node ${catkin_LIBRARIES} ${SVO_LIBS} ${OpenCV_LIBS})

add_executable(demo_stereo_node src/demo_stereo_node.cpp src/svo_factory.cpp)
target_link_libraries(demo_stereo_node ${catkin_LIBRARIES} ${SVO_LIBS} ${OpenCV_LIBS})