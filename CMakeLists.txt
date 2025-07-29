cmake_minimum_required(VERSION 3.10)
project(falcon_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_NIFALCON REQUIRED libnifalcon)
# upstream libnifalcon.pc file provides wrong library names for some reason
set(PC_NIFALCON_LIBRARIES nifalcon nifalcon_cli_base)
include_directories(${PC_NIFALCON_INCLUDE_DIRS})
set(NIFALCON_LIBRARIES)
foreach(lib ${PC_NIFALCON_LIBRARIES})
  find_library(${lib}_LIBRARIES
    NAMES ${lib}
    PATHS ${PC_NIFALCON_LIBRARY_DIRS}
    NO_DEFAULT_PATH
  )
  if(NOT ${lib}_LIBRARIES)
    message(FATAL_ERROR "Could not find ${lib} library in ${PC_NIFALCON_LIBRARY_DIRS}")
  endif()
  list(APPEND NIFALCON_LIBRARIES ${${lib}_LIBRARIES})
endforeach()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(driver src/driver.cpp)
add_dependencies(driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(driver ${catkin_LIBRARIES} ${NIFALCON_LIBRARIES})

install(TARGETS driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/grip_state nodes/example_p_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
