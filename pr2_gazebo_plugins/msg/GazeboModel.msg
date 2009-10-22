# This is a message that hold the data necessary to spawn a model in Gazebo
# this message is used by SpawnModel service calls to RosFactory plugin.

string model_name                        # name of the model to be spawned in Gazebo
string robot_model                       # contains either URDF or Gazebo XML robot model
uint32 xml_type                          # specifies what the content of robot_model is
string robot_namespace                   # ros namespace for all the ros interface of the model
geometry_msgs/Pose initial_pose          # initial pose in the gazebo world frame
uint32 disable_urdf_joint_limits = 0     # special flag to disable URDF joint limits
uint32 URDF                  = 0         # enum: robot_model contains XML string for an URDF model
uint32 GAZEBO_XML            = 1         # enum: robot_model contains XML string for an URDF model
uint32 URDF_PARAM_NAME       = 2         # enum: robot_model contains parameter name containing URDF XML
uint32 GAZEBO_XML_PARAM_NAME = 3         # enum: robot_model contains parameter name containing Gazebo Model XML
uint32 URDF_FILE_NAME        = 4         # enum: robot_model contains file name containing URDF XML
uint32 GAZEBO_XML_FILE_NAME  = 5         # enum: robot_model contains file name containing Gazebo Model XML

