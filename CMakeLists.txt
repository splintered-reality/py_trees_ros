##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(py_trees_ros)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()


##############################################################################
# Installs
##############################################################################

 catkin_install_python(
     PROGRAMS
         scripts/mock_battery
         scripts/mock_led_strip
         scripts/mock_component
         scripts/mock_safety_sensors
         scripts/tutorial_dashboard
         scripts/tutorial_tree
     DESTINATION
         ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
