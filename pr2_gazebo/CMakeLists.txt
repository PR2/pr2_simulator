cmake_minimum_required(VERSION 2.8.3)
project(pr2_gazebo)

find_package(catkin REQUIRED)

catkin_package()

# Install Gazebo Scripts
install(
  PROGRAMS
    scripts/pr2_fingertip_pressure_contact_translator.py
    scripts/pr2_simulate_torso_spring.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Gazebo launch files
install(
  DIRECTORY
    config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY
    launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install some gazebo test models
install(
  DIRECTORY
    urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

# testing
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  roslaunch_add_file_check(launch/pr2_empty_world.launch)

  set(ENV{ROSDEP_SOURCE_PATH} /tmp/)
  execute_process(COMMAND rosdep resolve rostest RESULT_VARIABLE rosdep_result)
  if(rosdep_result)
    message(STATUS "rosdep is not initialized, run rosdep init and update, please wait for a moment...")
    execute_process(COMMAND rosdep init)
    execute_process(COMMAND rosdep update)
  endif()

  add_rostest(test/collisions/test_slide.launch)

  add_rostest(test/mechanism_controllers/test_base.launch)
  if("$ENV{ROS_DISTRO}" STRGREATER "indigo") # indigo does not have tuck_arms_main.py in deb package
    add_rostest(test/mechanism_controllers/test_arm.launch)
  endif()
  add_rostest(test/mechanism_controllers/hztest_pr2_mechanism.launch)
  add_rostest(test/mechanism_controllers/hztest_rostime.launch)
  add_rostest(test/mechanism_controllers/hztest_pr2_odom.launch)

  add_rostest(test/sensors/test_scan.launch)
  add_rostest(test/sensors/hztest_pr2_scan.launch)

  if("$ENV{ROS_DISTRO}" STRGREATER "lunar") # indigo,kinetic,lunar does could not run image test on travis
    add_rostest(test/sensors/test_camera.launch)
    add_rostest(test/sensors/hztest_pr2_image.launch)
    add_rostest(test/sensors/hztest_pr2_stereo.launch)
  endif()

  add_rostest(test/spawn_test/test_pr2_spawn_with_offset.launch)
endif()
