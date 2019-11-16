cmake_minimum_required(VERSION 3.0.2)
#cmake_minimum_required(VERSION 2.8)
project(adi_tof_kinetic)
add_compile_options(-std=c++11)

set(PICO_LIBS 	${PROJECT_SOURCE_DIR}/dependencies/PicoZenseSDK_DCAM710/Lib/x64/libpicozense_api.so)


find_package(catkin REQUIRED COMPONENTS
  #cv_bridge
  image_transport
  image_view
  sensor_msgs
  tf
)
find_package(Qt5 COMPONENTS Core Widgets Gui Test Concurrent OpenGL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package( CATKIN_DEPENDS
  #cv_bridge
  image_transport
  sensor_msgs
)

# ${catkin_INCLUDE_DIRS} #picoZense API header file is here (/picozense/include/)

include_directories( include
  ${catkin_INCLUDE_DIRS} #picoZense API header file is here (/picozense/include/)
  ${OpenCV_INCLUDE_DIRS}
)
message(STATUS "catkin_include_dirs: " ${catkin_INCLUDE_DIRS})
message(STATUS "opencv_include_dirs: " ${OpenCV_INCLUDE_DIRS})

add_executable(dcam710_publisher src/dcam710_publisher.cpp)
add_dependencies(dcam710_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(dcam710_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PICO_LIBS} ${PICO_INCLUDE})

message(STATUS "opencv_libraries: " ${OpenCV_LIBRARIES})
