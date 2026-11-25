## Launch files

### pr2_empty_world.launch

Launch a PR2 in an empty world. This is the most basic PR2 simulation
configuration, and is useful for testing basic interaction with the
simulator.

### pr2.launch

Spawn a simulated PR2, assuming that an instance of Gazebo is already
up. This launch file is normally not used directly, but rather included
in another launch file, as in `pr2_empty_world.launch`.

This file is meant to mirror the behavior
[pr2_bringup](https://index.ros.org/p/pr2_bringup)`/pr2.launch`, which
is used to bringup a physical PR2, including default controllers.

### pr2_no_arms.launch

Spawn a simulated PR2 without arms, assuming that an instance of Gazebo
is already up. This launch file is normally not used directly, but
rather included in another launch file.

### pr2_no_controllers.launch

Spawn a simulated PR2 without the default controllers running, assuming
that an instance of Gazebo is already up. This launch file is normally
not used directly, but rather included in another launch file.

### Controller launch files

The launch files in `pr2_gazebo/controllers` are included by the
higher-level launch files documented above. The controller launch files
themselves should be modified only be advanced users.

## Scripts

The programs in `pr2_gazebo/scripts` are for developer testing only.

## Troubleshooting

Please see the
[simulator_gazebo troubleshooting](simulator_gazebo/Troubleshooting)
guide for tips on how to debug issues with `pr2_gazebo`.

## Troubleshooting

Please see the
[simulator_gazebo troubleshooting](simulator_gazebo/Troubleshooting)
guide for tips on how to debug issues with `pr2_gazebo`.
