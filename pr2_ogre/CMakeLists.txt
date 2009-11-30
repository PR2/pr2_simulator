cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

# find needed paths
rosbuild_find_ros_package(pr2_defs)
rosbuild_find_ros_package(pr2_description)
rosbuild_find_ros_package(pr2_ogre)
rosbuild_find_ros_package(ogre)

set(pr2_gen_files "")

# build the ogre mesh files from *.stl (and *.stlb from convex decomposition)
file(GLOB_RECURSE pr2_description_stl_files RELATIVE ${pr2_description_PACKAGE_PATH}/meshes ${pr2_description_PACKAGE_PATH}/meshes/*/*.stl ${pr2_description_PACKAGE_PATH}/meshes/*/convex/*.stlb)

set(pr2_description_out_path ${CMAKE_CURRENT_SOURCE_DIR}/Media/models)

MAKE_DIRECTORY(${pr2_description_out_path})

foreach(it ${pr2_description_stl_files})
  get_filename_component(relative_path ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  # create subdirectory
  add_custom_command(
    OUTPUT ${pr2_description_out_path}/${relative_path}
    # CMake 2.4 doesn't offer the make_directory command.
    #COMMAND ${CMAKE_COMMAND} -E make_directory
    COMMAND mkdir -p
    ARGS ${pr2_description_out_path}/${relative_path})

  # convert to ogre files
  add_custom_command(
    OUTPUT ${pr2_description_out_path}/${relative_path}/${basename}.mesh
    COMMAND rosrun 
    ARGS ogre_tools stl_to_mesh ${pr2_description_PACKAGE_PATH}/meshes/${it} ${pr2_description_out_path}/${relative_path}/${basename}.mesh
    DEPENDS ${pr2_description_out_path}/${relative_path} ${pr2_description_PACKAGE_PATH}/meshes/${it})
  
  set(pr2_gen_files ${pr2_gen_files} ${pr2_description_out_path}/${relative_path}/${basename}.mesh)
endforeach(it)

# build the ogre mesh files from *.stl (and *.stlb from convex decomposition)
file(GLOB_RECURSE pr2_defs_stl_files RELATIVE ${pr2_defs_PACKAGE_PATH}/meshes ${pr2_defs_PACKAGE_PATH}/meshes/*.stl ${pr2_defs_PACKAGE_PATH}/meshes/convex/*.stlb)

set(pr2_defs_out_path ${CMAKE_CURRENT_SOURCE_DIR}/Media/models)
MAKE_DIRECTORY(${pr2_defs_out_path})

foreach(it ${pr2_defs_stl_files})
  get_filename_component(relative_path ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  # create subdirectory
  add_custom_command(
    OUTPUT ${pr2_defs_out_path}/${relative_path}
    # CMake 2.4 doesn't offer the make_directory command.
    #COMMAND ${CMAKE_COMMAND} -E make_directory
    COMMAND mkdir -p
    ARGS ${pr2_defs_out_path}/${relative_path})

  # convert to ogre files
  add_custom_command(
    OUTPUT ${pr2_defs_out_path}/${relative_path}/${basename}.mesh
    COMMAND rosrun 
    ARGS ogre_tools stl_to_mesh ${pr2_defs_PACKAGE_PATH}/meshes/${it} ${pr2_defs_out_path}/${relative_path}/${basename}.mesh
    DEPENDS ${pr2_defs_out_path}/${relative_path} ${pr2_defs_PACKAGE_PATH}/meshes/${it})
  
  set(pr2_gen_files ${pr2_gen_files} ${pr2_defs_out_path}/${relative_path}/${basename}.mesh)
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${pr2_gen_files})




