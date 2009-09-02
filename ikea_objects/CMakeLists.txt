
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ikea_objects)

# xacro file generation
find_ros_package(ikea_objects)
find_ros_package(xacro)
find_ros_package(ivcon)
find_ros_package(convex_decomposition)

# iterate through all the obj files
file(GLOB ikea_objects_obj_files ${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.obj)
set(ikea_objects_gen_files "")

foreach(it ${ikea_objects_obj_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME)

  IF ( ${basename} MATCHES "_convex" )
    message("ignoring stale .._convex.* file:",${basename})
  ELSE ( ${basename} MATCHES "_convex" )

    #convex directory
    add_custom_command(
      OUTPUT ${basepath}/convex
      COMMAND mkdir -p
      ARGS ${basepath}/convex)
    set(ikea_objects_gen_files ${ikea_objects_gen_files} ${basepath}/convex)

    #create stl files from obj files
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}.stlb
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${it} ${basepath}/convex/${basename}.stlb
      DEPENDS ${it})
    set(ikea_objects_gen_files ${ikea_objects_gen_files} ${basepath}/convex/${basename}.stlb)

#     #move obj files to convex directory
#     add_custom_command(
#       OUTPUT ${basepath}/convex/${basename}
#       COMMAND cp
#       ARGS ${it} ${basepath}/convex/${basename}
#       DEPENDS ${it} ${basepath}/convex)
#     set(ikea_objects_gen_files ${ikea_objects_gen_files} ${basepath}/convex/${basename})
#  
#     #convex decomp in convex directory
#     add_custom_command(
#       OUTPUT ${basepath}/convex/${basename}_convex.obj
#       COMMAND ${convex_decomposition_PACKAGE_PATH}/convex_decomposition/bin/convex_decomposition
#       ARGS ${basepath}/convex/${basename} -v64 -p0.01 -c0.01 -d7
#       DEPENDS ${basepath}/convex/${basename})
#     set(ikea_objects_gen_files ${ikea_objects_gen_files} ${basepath}/convex/${basename}_convex.obj)
#  
#  
#     #create convex stl files for convex decomposition obj files
#     add_custom_command(
#       OUTPUT ${basepath}/convex/${basename}_convex.stlb
#       COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
#       ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stlb
#       DEPENDS ${basepath}/convex/${basename}_convex.obj)
#     set(ikea_objects_gen_files ${ikea_objects_gen_files} ${basepath}/convex/${basename}_convex.stlb)


  ENDIF ( ${basename} MATCHES "_convex" )

endforeach(it)

add_custom_target(media_files ALL DEPENDS ${ikea_objects_gen_files})

