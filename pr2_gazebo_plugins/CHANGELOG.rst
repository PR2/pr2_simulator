^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2022-04-12)
------------------
* Support Noetic (`#150 <https://github.com/pr2/pr2_simulator/issues/150>`_) from k-okada/ga
  use catkin_dev to use default veresion of Gazebo, see https://github.com/ros-simulation/gazebo_ros_pkgs/pull/571 for more info
  package.xml: use liborocos-kdl for ROS_PYTHON_VERSION=3
* Contributors: Kei Okada

2.0.14 (2019-04-29)
-------------------
* Merge pull request `#146 <https://github.com/PR2/pr2_simulator/issues/146>`_ from k-okada/fix_stretch
  remove gazebo depends
* remove gazebo depends
  since gazebo_plugins already depend on gazebo via gazebo_dev and this solves build farm error on stretch http://build.ros.org/job/Mbin_dsv8_dSv8__pr2_gazebo_plugins__debian_stretch_arm64__binary/
* Contributors: Kei Okada

2.0.13 (2019-04-24)
-------------------

2.0.12 (2019-04-08)
-------------------
* update travis.yml (`#144 <https://github.com/PR2/pr2_simulator/issues/144>`_)
* use GAZEBO_MAJOR_VERSION to switch 7(kinetic)/9(melodic)
* fixes for compile with gazebo 9 (`#143 <https://github.com/PR2/pr2_simulator/issues/143>`_)
* Contributors: Dave Feil-Seifer, Kei Okada

2.0.11 (2018-08-20)
-------------------
* Merge pull request `#140 <https://github.com/PR2/pr2_simulator/issues/140>`_ from furushchev/fix-power-monitor
  fix gazebo_ros_power_monitor plugin
* cleanup pr2_power_monitor plugin codes
* fix to work gazebo_ros_power_monitor plugin
* Contributors: Furushchev, Kei Okada

2.0.10 (2018-03-18)
-------------------

2.0.9 (2018-02-18)
------------------

2.0.8 (2018-02-14)
------------------
* Merge pull request `#135 <https://github.com/pr2/pr2_simulator/issues/135>`_ from k-okada/kinetic-devel
  fix more cmake warnings
* fix warning on The dependency target pr2_gazebo_plugins_gencfg does not exists
* fix compile error on catkin_package() DEPENDS on 'SDF' but neither 'SDF_INCLUDE_DIRS' nor...
* Merge pull request `#130 <https://github.com/pr2/pr2_simulator/issues/130>`_ from k-okada/driver_base
  pr2_gazebo_plugins is no longer depend on driver_base
* Merge pull request `#128 <https://github.com/pr2/pr2_simulator/issues/128>`_ from k-okada/maintainer
  change maintainer to ROS orphaned package maintainer
* pr2_gazebo_plugins is no longer depend on driver_base
* change maintainer to ROS orphaned package maintainer
* Merge pull request `#120 <https://github.com/pr2/pr2_simulator/issues/120>`_ from v4hn/missing_gazebo_flags
  add missing GAZEBO_CXX_FLAGS
* add missing GAZEBO_CXX_FLAGS
  This is required to compile the plugins with Gazebo 7.
  add_compile_options requires cmake 2.8.12 which is around in ubuntu since 14.04.
* Contributors: Devon Ash, Kei Okada, v4hn

2.0.5 (2015-03-13)
------------------
* Removed dependency on PCL for pr2_Gazebo_plugins
* Contributors: TheDash

2.0.4 (2015-02-11)
------------------

2.0.3 (2014-10-07)
------------------
* Changelogs
* Changelogs
* Contributors: TheDash

* Changelogs
* Contributors: TheDash
