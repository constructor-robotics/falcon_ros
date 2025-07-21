cmake_minimum_required(VERSION 3.10)
project(falcon_ros)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 joy
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_NIFALCON REQUIRED libnifalcon)
set(PC_NIFALCON_LIBRARIES ${PC_NIFALCON_LIBRARIES} nifalcon_cli_base)
include_directories(${PC_NIFALCON_INCLUDE_DIRS})
link_directories(${PC_NIFALCON_LIBRARY_DIRS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(driver src/driver.cpp)
add_dependencies(driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(driver ${catkin_LIBRARIES} ${PC_NIFALCON_LIBRARIES})

install(TARGETS driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
