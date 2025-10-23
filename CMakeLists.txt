cmake_minimum_required(VERSION 3.16)
project(falcon_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

include(cmake/CPM.cmake)

CPMAddPackage(
   NAME libnifalcon
   GIT_REPOSITORY "https://github.com/libnifalcon/libnifalcon"
   VERSION "1.0.2"
   GIT_TAG "f7c727aa24403ea10dc1507d7d396eeb7d55564a"
   OPTIONS
   "BUILD_STATIC OFF"
)

catkin_package()

include_directories(${libnifalcon_SOURCE_DIR}/include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(driver src/driver.cpp)
add_dependencies(driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(driver ${catkin_LIBRARIES} nifalcon)

install(TARGETS driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/grip_state nodes/example_p_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config udev urdf robots meshes launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
