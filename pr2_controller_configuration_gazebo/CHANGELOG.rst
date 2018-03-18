^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_controller_configuration_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.10 (2018-03-18)
-------------------

2.0.9 (2018-02-18)
------------------
* Merge pull request `#137 <https://github.com/pr2/pr2_simulator/issues/137>`_ from v4hn/config-build-depends
  Config build depends
* remove build-dependencies on gazebo from configuration package
  The CMakeLists.txt does not build anything.
  It should not find-package things it does not need during build.
* add build and run depend to gazebo
  (note that pr2_gazebo  depend on pr2_controller_configuration_gazebo)
* Contributors: Kei Okada, v4hn

2.0.8 (2018-02-14)
------------------
* Merge pull request `#135 <https://github.com/pr2/pr2_simulator/issues/135>`_ from k-okada/kinetic-devel
  fix more cmake warnings
* fix compile error on catkin_package() DEPENDS on 'SDF' but neither 'SDF_INCLUDE_DIRS' nor...
* Merge pull request `#133 <https://github.com/pr2/pr2_simulator/issues/133>`_ from k-okada/125
  Removed repeated arg "gui", see https://github.com/PR2/pr2_simulator/pull/125
* Merge pull request `#128 <https://github.com/pr2/pr2_simulator/issues/128>`_ from k-okada/maintainer
  change maintainer to ROS orphaned package maintainer
* Removed repeated arg "gui", see https://github.com/PR2/pr2_simulator/pull/125
* change maintainer to ROS orphaned package maintainer
* Contributors: Kei Okada

2.0.5 (2015-03-13)
------------------

2.0.4 (2015-02-11)
------------------

2.0.3 (2014-10-07)
------------------
* Changelogs
* Changelogs
* Corrected typo in configuration of pr2-joint-effort-controllers.
* Contributors: Georg Bartels, TheDash

* Changelogs
* Corrected typo in configuration of pr2-joint-effort-controllers.
* Contributors: Georg Bartels, TheDash

* Corrected typo in configuration of pr2-joint-effort-controllers.
* Contributors: Georg Bartels
