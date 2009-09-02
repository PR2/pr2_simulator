cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ikea_ogre)

# find needed paths
find_ros_package(ikea_objects)
find_ros_package(ogre)

MESSAGE(STATUS "\nikea_objects path  ${ikea_objects_PACKAGE_PATH}\n")
if(EXISTS ${ikea_objects_PACKAGE_PATH})
  # build the ogre mesh files from *.stl (and *.stlb from convex decomposition)
  file(GLOB ikea_objects_stl_files ${ikea_objects_PACKAGE_PATH}/meshes/convex/*.stlb)
  set(ikea_objects_gen_files "")

  set(ikea_objects_out_path ${CMAKE_CURRENT_SOURCE_DIR}/Media/models/ikea_objects)

  MAKE_DIRECTORY(${ikea_objects_out_path})

  foreach(it ${ikea_objects_stl_files})
    get_filename_component(basename ${it} NAME)

    # convert to ogre files
    add_custom_command(
      OUTPUT ${ikea_objects_out_path}/${basename}.mesh
      COMMAND rosrun 
      ARGS ogre_tools stl_to_mesh ${it} ${ikea_objects_out_path}/${basename}.mesh
      DEPENDS ${it})
    
    set(ikea_objects_gen_files ${ikea_objects_gen_files} ${ikea_objects_out_path}/${basename}.mesh)
  endforeach(it)

  add_custom_target(ikea_objects_media_files ALL DEPENDS ${ikea_objects_gen_files})
endif(EXISTS ${ikea_objects_PACKAGE_PATH})

