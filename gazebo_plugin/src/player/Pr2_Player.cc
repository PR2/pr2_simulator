#include <libplayerc++/playerc++.h>
#include <iostream>

int main()
{
  // We throw exceptions on creation if we fail
  try
  {
    using namespace PlayerCc;
    using namespace std;

    // Create a player client object, using the variables assigned by the
    // call to parse_args()
    PlayerClient robot(PlayerCc::PLAYER_HOSTNAME, PlayerCc::PLAYER_PORTNUM);

    // Subscribe to the simulation proxy
    ActArrayProxy ap_l(&robot, 0); // left arm
    ActArrayProxy ap_r(&robot, 1); // right arm
    ActArrayProxy ap_c(&robot, 2); // caster steering
    ActArrayProxy ap_s(&robot, 3); // spine slider
    ActArrayProxy ap_h(&robot, 4); // head

    Position2dProxy flc(&robot, 0); // front left caster differential drive
    Position2dProxy frc(&robot, 1); // front right caster differential drive
    Position2dProxy rlc(&robot, 2); // rear left caster differential drive
    Position2dProxy rrc(&robot, 3); // rear right caster differential drive

    // Print out some stuff
    std::cout << robot << std::endl;

    flc.SetMotorEnable(true);
    frc.SetMotorEnable(true);
    rlc.SetMotorEnable(true);
    rrc.SetMotorEnable(true);

    for (;;)
    {
      // This blocks until new data comes
      robot.Read();

      flc.SetSpeed(1, 0);
      frc.SetSpeed(1, 0);
      rlc.SetSpeed(1, 0);
      rrc.SetSpeed(1, 0);

      ap_h.MoveTo(0, -M_PI/4.0); // head tilt
      ap_h.MoveTo(1,  M_PI/6.0); // head neck yaw
      ap_h.MoveTo(2, -M_PI/4.0); // hokuyo pitch
      ap_h.MoveTo(3,  M_PI/5.0); // left cam yaw
      ap_h.MoveTo(4, -M_PI/3.0); // left cam pitch
      ap_h.MoveTo(5, -M_PI/5.0); // right cam yaw
      ap_h.MoveTo(6,  M_PI/4.0); // right cam pitch

      ap_l.MoveTo(0,  M_PI/4.0);
      ap_l.MoveTo(1,  M_PI/4.0);
      ap_l.MoveTo(2,  M_PI/4.0);
      ap_l.MoveTo(3,  M_PI/5.0);
      ap_l.MoveTo(4,  M_PI/3.0);
      ap_l.MoveTo(5,  M_PI/2.0);
      ap_l.MoveTo(6,  M_PI/1.0);

      ap_r.MoveTo(0, -M_PI/3.0);
      ap_r.MoveTo(1, -M_PI/3.0);
      ap_r.MoveTo(2, -M_PI/3.0);
      ap_r.MoveTo(3, -M_PI/2.0);
      ap_r.MoveTo(4, -M_PI/3.0);
      ap_r.MoveTo(5, -M_PI/1.0);
      ap_r.MoveTo(6, -M_PI/2.0);

      ap_c.MoveTo(0, -    M_PI/4.0);
      ap_c.MoveTo(1, -3.0*M_PI/4.0);
      ap_c.MoveTo(2,      M_PI/4.0);
      ap_c.MoveTo(3,  3.0*M_PI/4.0);

      ap_s.MoveTo(0,0.5);

      player_actarray_actuator_t data0 = ap_l.GetActuatorData(0);
      printf("0 Pos[%f] Speed[%f]\n",data0.position, data0.speed);
      player_actarray_actuator_t data1 = ap_l.GetActuatorData(1);
      printf("1 Pos[%f] Speed[%f]\n",data1.position, data1.speed);
      player_actarray_actuator_t data2 = ap_l.GetActuatorData(2);
      printf("2 Pos[%f] Speed[%f]\n",data2.position, data2.speed);
    }
  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << "Error:" << e << std::endl;
    return -1;
  }
}
