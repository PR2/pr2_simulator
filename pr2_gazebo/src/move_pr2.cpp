#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <gazebo_msgs/BodyRequest.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <std_srvs/Empty.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    //goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(3.0);
    ros::Duration wait_time(20.0);
    actionlib::SimpleClientGoalState state = traj_client_->sendGoalAndWait(goal,wait_time,wait_time);
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The arm traj finished with state [%s]",state.toString().c_str());
    else
      ROS_INFO("The arm traj failed with state [%s]",state.toString().c_str());
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    goal.trajectory.points[ind].positions[6] = 0.0;
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.25;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    goal.trajectory.points[ind].positions[6] = 0.0;
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(6.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

class PR2Controllers
{

   PR2Controllers()
   {
   }

   ~PR2Controllers()
   {
   }

   void waitForControllers()
   {
   }

   void stopControllers()
   {
   }

   void startControllers()
   {
   }


};



// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
private:
  GripperClient* gripper_client_;  

public:
  //Action client initialization
  Gripper(std::string action_topic){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient(action_topic, true);
    
    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(60.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = 150.0;  // use -1 to not limit effort (negative)
    
    ROS_INFO("Sending open goal");

    ROS_WARN("cancelling all gripper actionclient goals");
    gripper_client_->cancelAllGoals();

    bool success = false;
    while (!success)
    {
      actionlib::SimpleClientGoalState state =
        gripper_client_->sendGoalAndWait(open,ros::Duration(50.0),ros::Duration(30.0));
      bool finished_before_timeout = gripper_client_->waitForResult(ros::Duration(50.0));

      if (finished_before_timeout)
      {
        ROS_INFO("The gripper opened.");
        success = true;
      }
      else
      {
        ROS_WARN("The gripper failed to open [%s].",gripper_client_->getState().toString().c_str());
      }

      // final check if goal reached (doesn't seem to work)
      // success = (gripper_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
      // ROS_ERROR("state [%d] success [%d]", gripper_client_->getState().state_, success);
    }
  }

  //Close the gripper
  void close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently
    
    ROS_INFO("Sending squeeze goal");
    if (gripper_client_->sendGoalAndWait(squeeze,ros::Duration(5.0),ros::Duration(3.0))
        == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
    gripper_client_->sendGoal(squeeze);
  }
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle rh("");
  ros::Duration(1e-9).sleep();

  // wait for services
  std::string pause_srv_name = "/gazebo/pause_physics";
  ros::service::waitForService(pause_srv_name);

  std::string unpause_srv_name = "/gazebo/unpause_physics";
  ros::service::waitForService(unpause_srv_name);

  std::string clear_body_wrenches_srv_name = "/gazebo/clear_body_wrenches";
  ros::service::waitForService(clear_body_wrenches_srv_name);

  Gripper r_gripper("r_gripper_controller/gripper_action");
  Gripper l_gripper("l_gripper_controller/gripper_action");

  std::string switch_controller_srv_name = "/pr2_controller_manager/switch_controller";
  ros::service::waitForService(switch_controller_srv_name);

  std::string list_controllers_srv_name = "/pr2_controller_manager/list_controllers";
  ros::service::waitForService(list_controllers_srv_name);

  // wait for pr2 controllers
  ros::ServiceClient list_controller_client = rh.serviceClient<pr2_mechanism_msgs::ListControllers>(list_controllers_srv_name);
  pr2_mechanism_msgs::ListControllers list_controller;
  int controllers_count = 0;
  while (controllers_count != 8)
  {
    controllers_count = 0;
    list_controller_client.call(list_controller);
    for (unsigned int i = 0; i < list_controller.response.controllers.size() ; ++i)
    {
      std::string name = list_controller.response.controllers[i];
      std::string state = list_controller.response.state[i];
      if ((name == "r_gripper_controller" || name == "l_gripper_controller" ||
          name == "laser_tilt_controller" || name == "head_traj_controller" ||
          name == "r_arm_controller" || name == "l_arm_controller" ||
          name == "base_controller" || name == "torso_controller") && state == "running")
        controllers_count++;
      ROS_INFO("controller [%s] state [%s] count[%d]",name.c_str(), state.c_str(),controllers_count);
    }
  }

  // clear object wrench
  /*
  ros::ServiceClient clear_client = rh.serviceClient<gazebo_msgs::BodyRequest>(clear_body_wrenches_srv_name);
  gazebo_msgs::BodyRequest body_request;
  body_request.request.body_name = "object_1::coke_can";
  clear_client.call(body_request);
  */

  // open gripper
  r_gripper.open();
  r_gripper.open(); // only works after two calls
  //l_gripper.open();

  ROS_INFO("stopping controllers");
  // stop arm controller
  ros::ServiceClient switch_controller_client = rh.serviceClient<pr2_mechanism_msgs::SwitchController>(switch_controller_srv_name);
  pr2_mechanism_msgs::SwitchController switch_controller;
  switch_controller.request.start_controllers.clear();
  switch_controller.request.stop_controllers.clear();
  switch_controller.request.stop_controllers.push_back("r_gripper_controller");
  switch_controller.request.stop_controllers.push_back("l_gripper_controller");
  switch_controller.request.stop_controllers.push_back("laser_tilt_controller");
  switch_controller.request.stop_controllers.push_back("head_traj_controller");
  switch_controller.request.stop_controllers.push_back("r_arm_controller");
  switch_controller.request.stop_controllers.push_back("l_arm_controller");
  switch_controller.request.stop_controllers.push_back("base_controller");
  switch_controller.request.stop_controllers.push_back("torso_controller");
  switch_controller.request.strictness = pr2_mechanism_msgs::SwitchControllerRequest::STRICT;
  switch_controller_client.call(switch_controller);
  
  ROS_INFO("pausing physics");
  // pause simulation
  ros::ServiceClient ps_client = rh.serviceClient<std_srvs::Empty>(pause_srv_name);
  std_srvs::Empty ps;
  ps_client.call(ps);

  ROS_INFO("setting arm pose");
  // set arm model configuration
  std::string smcn = "/gazebo/set_model_configuration";
  ros::service::waitForService(smcn);
  ros::ServiceClient smc_client = rh.serviceClient<gazebo_msgs::SetModelConfiguration>(smcn);
  gazebo_msgs::SetModelConfiguration smc;
  smc.request.model_name = "pr2";
  smc.request.urdf_param_name = "robot_description";

  smc.request.joint_names.push_back("head_pan_joint"); 
  smc.request.joint_names.push_back("head_tilt_joint"); 
  smc.request.joint_names.push_back("laser_tilt_mount_joint");

  smc.request.joint_positions.push_back(-0.04695624787873154);
  smc.request.joint_positions.push_back(0.9571995901909744);
  smc.request.joint_positions.push_back(1.0137128597170113);



  smc.request.joint_names.push_back("r_upper_arm_roll_joint"); 
  smc.request.joint_names.push_back("r_shoulder_pan_joint"); 
  smc.request.joint_names.push_back("r_shoulder_lift_joint"); 
  smc.request.joint_names.push_back("r_forearm_roll_joint"); 
  smc.request.joint_names.push_back("r_elbow_flex_joint"); 
  smc.request.joint_names.push_back("r_wrist_flex_joint"); 
  smc.request.joint_names.push_back("r_wrist_roll_joint"); 
 
  smc.request.joint_positions.push_back(-1.6399960594910876);
  smc.request.joint_positions.push_back(-0.3689082990354322);
  smc.request.joint_positions.push_back(0.3212309310513026);
  smc.request.joint_positions.push_back(-3.5471901137256694);
  smc.request.joint_positions.push_back(-1.459007345767927);
  smc.request.joint_positions.push_back(-0.6873569547257024);
  smc.request.joint_positions.push_back(-1.4355428273114548);


  smc.request.joint_names.push_back("l_upper_arm_roll_joint"); 
  smc.request.joint_names.push_back("l_shoulder_pan_joint"); 
  smc.request.joint_names.push_back("l_shoulder_lift_joint"); 
  smc.request.joint_names.push_back("l_forearm_roll_joint"); 
  smc.request.joint_names.push_back("l_elbow_flex_joint"); 
  smc.request.joint_names.push_back("l_wrist_flex_joint"); 
  smc.request.joint_names.push_back("l_wrist_roll_joint"); 

  smc.request.joint_positions.push_back(1.6399982206352233);
  smc.request.joint_positions.push_back(2.1350018853307198);
  smc.request.joint_positions.push_back(-0.02000609892474703);
  smc.request.joint_positions.push_back(1.6399955487793614);
  smc.request.joint_positions.push_back(-2.070018727445361);
  smc.request.joint_positions.push_back(-1.6800004066137673);
  smc.request.joint_positions.push_back(1.3980012258720143);


  // 'l_gripper_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_motor_screw_joint', 'l_gripper_motor_slider_joint', 
  // 3.895490011392481e-05, 0.0023094056377544747, 0.0023094056377544747, 0.0023094056377544747, 0.0023094056377544747, 0.0, 0.0, 

/* need to set parallel links on the gripper too, but not exposed through urdf
  smc.request.joint_names.push_back("r_gripper_l_finger_joint"); 
  smc.request.joint_names.push_back("r_gripper_r_finger_joint"); 
  smc.request.joint_names.push_back("r_gripper_r_finger_tip_joint"); 
  smc.request.joint_names.push_back("r_gripper_l_finger_tip_joint"); 
  smc.request.joint_names.push_back("r_gripper_joint"); 
  smc.request.joint_names.push_back("r_gripper_motor_screw_joint"); 
  smc.request.joint_names.push_back("r_gripper_motor_slider_joint"); 

  smc.request.joint_positions.push_back(0.5014809427821072);
  smc.request.joint_positions.push_back(0.5014809427821072);
  smc.request.joint_positions.push_back(0.5014809427821072);
  smc.request.joint_positions.push_back(0.5014809427821072);
  smc.request.joint_positions.push_back(0.0859999741332338);
  smc.request.joint_positions.push_back(0.0);
  smc.request.joint_positions.push_back(0.0);
*/



  smc_client.call(smc);
  
  // unpause simulation
  ros::ServiceClient ups_client = rh.serviceClient<std_srvs::Empty>(unpause_srv_name);
  std_srvs::Empty ups;
  ups_client.call(ups);

  /* check another pose
  ROS_INFO("setting arm pose 2");
  gazebo_msgs::SetModelConfiguration smc1;
  smc1.request.model_name = "pr2";
  smc1.request.urdf_param_name = "robot_description";
  smc1.request.joint_names.push_back("r_upper_arm_roll_joint");
  smc1.request.joint_names.push_back("r_shoulder_pan_joint");
  smc1.request.joint_names.push_back("r_shoulder_lift_joint");
  smc1.request.joint_names.push_back("r_forearm_roll_joint");
  smc1.request.joint_names.push_back("r_elbow_flex_joint");
  smc1.request.joint_names.push_back("r_wrist_flex_joint");
  smc1.request.joint_names.push_back("r_wrist_roll_joint");
  smc1.request.joint_positions.push_back(0.0);
  smc1.request.joint_positions.push_back(-1.0);
  smc1.request.joint_positions.push_back(-0.2);
  smc1.request.joint_positions.push_back(0.0);
  smc1.request.joint_positions.push_back(-0.2);
  smc1.request.joint_positions.push_back(-0.2);
  smc1.request.joint_positions.push_back(0.0);
  smc_client.call(smc1);
 */

  ROS_INFO("restarting controllers");
  // start arm controller
  switch_controller.request.start_controllers.clear();
  switch_controller.request.start_controllers.push_back("r_gripper_controller");
  switch_controller.request.start_controllers.push_back("l_gripper_controller");
  switch_controller.request.start_controllers.push_back("laser_tilt_controller");
  switch_controller.request.start_controllers.push_back("head_traj_controller");
  switch_controller.request.start_controllers.push_back("r_arm_controller");
  switch_controller.request.start_controllers.push_back("l_arm_controller");
  switch_controller.request.start_controllers.push_back("base_controller");
  switch_controller.request.start_controllers.push_back("torso_controller");
  switch_controller.request.stop_controllers.clear();
  switch_controller_client.call(switch_controller);

  /* 
  ROS_INFO("closing grippers");
  r_gripper.close();
  //l_gripper.close();

  ROS_INFO("stopping controllers");
  // stop arm controller
  switch_controller.request.start_controllers.clear();
  switch_controller.request.stop_controllers.clear();
  switch_controller.request.stop_controllers.push_back("r_gripper_controller");
  switch_controller.request.stop_controllers.push_back("l_gripper_controller");
  switch_controller.request.stop_controllers.push_back("laser_tilt_controller");
  switch_controller.request.stop_controllers.push_back("head_traj_controller");
  switch_controller.request.stop_controllers.push_back("r_arm_controller");
  switch_controller.request.stop_controllers.push_back("l_arm_controller");
  switch_controller.request.stop_controllers.push_back("base_controller");
  switch_controller.request.stop_controllers.push_back("torso_controller");
  switch_controller.request.strictness = pr2_mechanism_msgs::SwitchControllerRequest::STRICT;
  switch_controller_client.call(switch_controller);

  ROS_INFO("resetting simulation");
  // reset simulation
  std::string rsn = "/gazebo/reset_simulation";
  ros::service::waitForService(rsn);
  ros::ServiceClient rs_client = rh.serviceClient<std_srvs::Empty>(rsn);
  std_srvs::Empty rs;
  rs_client.call(rs);

  ROS_INFO("stop all controllers");
  // start arm controller
  switch_controller.request.start_controllers.clear();
  switch_controller.request.start_controllers.push_back("r_gripper_controller");
  switch_controller.request.start_controllers.push_back("l_gripper_controller");
  switch_controller.request.start_controllers.push_back("laser_tilt_controller");
  switch_controller.request.start_controllers.push_back("head_traj_controller");
  switch_controller.request.start_controllers.push_back("r_arm_controller");
  switch_controller.request.start_controllers.push_back("l_arm_controller");
  switch_controller.request.start_controllers.push_back("base_controller");
  switch_controller.request.start_controllers.push_back("torso_controller");
  switch_controller.request.stop_controllers.clear();
  switch_controller_client.call(switch_controller);
  */

/*
  RobotArm arm;

  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());

  //sleep(5); // FIXME: seems startTrajectory finishes without waiting?

  // close gripper
  gripper.close();
*/

/*
  // pull object down
  ros::ServiceClient wrench_client = rh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench wrench;
  //wrench.request.body_name       = "object_1::object_1_base_link";
  //wrench.request.reference_frame = "object_1::object_1_base_link";
  wrench.request.body_name       = "object_1::coke_can";
  wrench.request.reference_frame = "object_1::coke_can";
  wrench.request.wrench.force.z = -25.0;
  wrench.request.start_time = ros::Time(0.0);
  wrench.request.duration = ros::Duration(-1.0);
  wrench_client.call(wrench);
*/



}

