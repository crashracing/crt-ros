include_directories(
  include/
)

# CRTBot Hardware Interface
add_library(crtbot_hw_interface
  src/crtbot_hw_interface.cpp
)
target_link_libraries(crtbot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(crtbot_hw_main src/crtbot_hw_main.cpp)
target_link_libraries(crtbot_hw_main
  crtbot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    crtbot_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    crtbot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/crtbot_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
