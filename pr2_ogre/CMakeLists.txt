cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(pr2_ogre)

# find needed paths
find_ros_package(pr2_defs)
find_ros_package(pr2_ogre)
find_ros_package(ogre)

# build the ogre mesh files from *.stl (and *.stlb from convex decomposition)
file(GLOB pr2_stl_files ${pr2_defs_PACKAGE_PATH}/meshes/*.stl)
set(pr2_gen_files "")

set(pr2_out_path ${CMAKE_CURRENT_SOURCE_DIR}/Media/models)

MAKE_DIRECTORY(${pr2_out_path})

foreach(it ${pr2_stl_files})
  get_filename_component(basename ${it} NAME_WE)

  # convert to ogre files
  add_custom_command(
    OUTPUT ${pr2_out_path}/${basename}.mesh
    COMMAND rosrun 
    ARGS ogre_tools stl_to_mesh ${it} ${pr2_out_path}/${basename}.mesh
    DEPENDS ${it})
  
  set(pr2_gen_files ${pr2_gen_files} ${pr2_out_path}/${basename}.mesh)
endforeach(it)

# process convexified mesh files
file(GLOB pr2_convex_stl_files ${pr2_defs_PACKAGE_PATH}/meshes/convex/*.stlb)

set(pr2_convex_out_path ${CMAKE_CURRENT_SOURCE_DIR}/Media/models/convex)

MAKE_DIRECTORY(${pr2_convex_out_path})

foreach(it ${pr2_convex_stl_files})
  get_filename_component(basename ${it} NAME_WE)

  # convert to ogre files
  add_custom_command(
    OUTPUT ${pr2_convex_out_path}/${basename}.mesh
    COMMAND rosrun 
    ARGS ogre_tools stl_to_mesh ${it} ${pr2_convex_out_path}/${basename}.mesh
    DEPENDS ${it})
  
  set(pr2_gen_files ${pr2_gen_files} ${pr2_convex_out_path}/${basename}.mesh)
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${pr2_gen_files})




