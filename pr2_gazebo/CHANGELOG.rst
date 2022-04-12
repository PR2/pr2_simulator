^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2022-04-12)
------------------
* Support Noetic (`#150 <https://github.com/pr2/pr2_simulator/issues/150>`_)
  test/sensors/test_camera.py: noetic does not need to str(image.data) and rotate()
  use bigger cup
  relax hztest in hztest_pr2_scan.launch
  fix for python3: 2to3 -w -f except .
  fix for python3: 2to3 -w -f print .
  remove rosdep update from CMakeLists.txt
  hztest_pr2_image.launch: relax hzerror for caemra test
  add robot_state_publisher to pr2_gazebo/package.xml
* Contributors: Kei Okada

2.0.14 (2019-04-29)
-------------------
* Merge pull request `#146 <https://github.com/PR2/pr2_simulator/issues/146>`_ from k-okada/fix_stretch
  remove gazebo depends
* use custom empty.world which does not require downloading models
* Contributors: Kei Okada

2.0.13 (2019-04-24)
-------------------
* enable test code in pr2_simulator (`#145 <https://github.com/PR2/pr2_simulator/issues/145>`_)
* pr2_no_controllers.launch: remove deprecated --ros_namespace /gazebo. This is not used long time, but using argparse cache this error. https://github.com/ros-simulation/gazebo_ros_pkgs/commit/46ac000ea34d006d9f37a2392c2146bbd9e6d1be
* package.xml: add more run_depend and test_depend, also export gazebo_model_path
* Contributors: Kei Okada

2.0.12 (2019-04-08)
-------------------

2.0.11 (2018-08-20)
-------------------
* Merge pull request `#142 <https://github.com/PR2/pr2_simulator/issues/142>`_ from pushkalkatara/kinetic-devel
  Issue `#141 <https://github.com/PR2/pr2_simulator/issues/141>`_
* Issue `#141 <https://github.com/PR2/pr2_simulator/issues/141>`_
* Contributors: Kei Okada, Pushkal Katara

2.0.10 (2018-03-18)
-------------------
* Merge pull request (`#139 <https://github.com/pr2/pr2_simulator/issues/139>`_)
  remove find_package(gazebo), to solve -pthread linking errors
* remove find_package(gazebo), to sovle -pthread linking errors
* Contributors: Kei Okada

2.0.9 (2018-02-18)
------------------

2.0.8 (2018-02-14)
------------------
* Merge pull request `#135 <https://github.com/pr2/pr2_simulator/issues/135>`_ from k-okada/kinetic-devel
  fix more cmake warnings
* fix compile error on catkin_package() DEPENDS on 'SDF' but neither 'SDF_INCLUDE_DIRS' nor...
* Merge pull request `#134 <https://github.com/pr2/pr2_simulator/issues/134>`_ from furushchev/arg-kinect
  pr2_gazebo: add arg KINECT
* Merge pull request `#133 <https://github.com/pr2/pr2_simulator/issues/133>`_ from k-okada/125
  Removed repeated arg "gui", see https://github.com/PR2/pr2_simulator/pull/125
* Merge pull request `#132 <https://github.com/pr2/pr2_simulator/issues/132>`_ from furushchev/headless
  [pr2_gazebo/launch/pr2_empty_world.launch] add headless/debug arguments for gazebo
* Merge pull request `#131 <https://github.com/pr2/pr2_simulator/issues/131>`_ from MatthewRueben/hydro-devel
  Rename "gazebo" package to "gazebo_ros"
* Merge pull request `#128 <https://github.com/pr2/pr2_simulator/issues/128>`_ from k-okada/maintainer
  change maintainer to ROS orphaned package maintainer
* Removed repeated arg "gui", see https://github.com/PR2/pr2_simulator/pull/125
* change maintainer to ROS orphaned package maintainer
* pr2_gazebo: add arg KINECT
* [pr2_gazebo/launch/pr2_empty_world.launch] add headless/debug argument for gazebo
* Merge pull request `#118 <https://github.com/pr2/pr2_simulator/issues/118>`_ from PR2/revert-117-indigo-devel
  Revert "Added argument passing for Kinect2"
* Revert "Added argument passing for Kinect2"
* Merge pull request `#117 <https://github.com/pr2/pr2_simulator/issues/117>`_ from archielee/indigo-devel
  Added argument passing for Kinect2
* Added argument passing for Kinect2
* Changed all instances of "gazebo" package to "gazebo_ros". The "gazebo" package was renamed "gazebo_ros" in hydro.
* Contributors: Devon Ash, Furushchev, Kei Okada, Yuki Furuta, archielee, elliot

2.0.5 (2015-03-13)
------------------

2.0.4 (2015-02-11)
------------------

2.0.3 (2014-10-07)
------------------
* Changelogs
* Changelogs
* Bug fix thanks to vovakkk @github
* Contributors: TheDash

* Changelogs
* Bug fix thanks to vovakkk @github
* Contributors: TheDash

* Bug fix thanks to vovakkk @github
* Contributors: TheDash
