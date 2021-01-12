#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#include <iostream>
using namespace std;

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]


  // can access parts of momentCmd via e.g. momentCmd.x
  // need the arm length parameter L, and the drag/thrust ratio kappa

  ///////////////////////////////////////////////////////////

//  cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
//  cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
//  cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
//  cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right
    
    // b = [Ttotal, taux/l, tauy/l, tauz/kappa].transpose
    float l = L/sqrtf(2.f);
    float b1 = collThrustCmd;
    float b2 = momentCmd.x / l;
    float b3 = momentCmd.y / l;
    float b4 = - momentCmd.z / kappa;

    cmd.desiredThrustsN[0] = (b1+b2+b3+b4)/4.f; // front left
    cmd.desiredThrustsN[1] = (b1-b2+b3-b4)/4.f; // front right
    cmd.desiredThrustsN[2] = (b1+b2-b3-b4)/4.f; // rear left
    cmd.desiredThrustsN[3] = (b1-b2-b3+b4)/4.f; // rear right
    
  ///////////////////////////////////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  //  can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  need parameters for moments of inertia Ixx, Iyy, Izz
  //  also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  /////////////////////////////////////////////////////////
  
    V3F u_bar = (pqrCmd - pqr);
//    printf("kpPQR: %.3f %.3f %.3f \n", kpPQR[0], kpPQR[1], kpPQR[2]);
//    printf("u_bar: %.3f %.3f %.3f \n", u_bar[0], u_bar[1], u_bar[2]);
//    u_bar = kpPQR * u_bar;
//    printf("u_bar: %.3f %.3f %.3f \n", u_bar[0], u_bar[1], u_bar[2]);
    V3F I;
    I.x = Ixx;
    I.y = Iyy;
    I.z = Izz;
    momentCmd = I * kpPQR * u_bar;

  ///////////////////////////////////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  //  rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  need the roll/pitch gain kpBank
  //  collThrustCmd is a force in Newtons! Convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////////////////////////////////
  float p = 0;
  float q = 0;
  if (collThrustCmd > 0) {
    float c = - collThrustCmd/mass;
    float bx_cmd = accelCmd.x/c;
    float by_cmd = accelCmd.y/c;
    
    bx_cmd = CONSTRAIN(bx_cmd,-maxTiltAngle, maxTiltAngle);
    by_cmd = CONSTRAIN(by_cmd,-maxTiltAngle, maxTiltAngle);
    
//    float bx_dot = (R(0,2) - bx_cmd) * kpBank;
//    float by_dot = (R(1,2) - by_cmd) * kpBank;
    float bx_dot = (bx_cmd - R(0,2)) * kpBank;
    float by_dot = (by_cmd - R(1,2)) * kpBank;
    p = (R(1,0) * bx_dot - R(0,0) * by_dot) / R(2,2);
    q = (R(1,1) * bx_dot - R(0,1) * by_dot) / R(2,2);
  };
    
  pqrCmd.x = p;
  pqrCmd.y = q;
  pqrCmd.z = 0;
    
  ///////////////////////////////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  //  rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  need the gain parameters kpPosZ and kpVelZ
  //  maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  return a force, not an acceleration
  //  for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////////////////////////////////

    float ubar = kpPosZ * (posZCmd - posZ) + kpVelZ * (velZCmd - velZ) + accelZCmd;
    integratedAltitudeError += (posZCmd - posZ) * dt;
    ubar += KiPosZ * integratedAltitudeError;
    float c = (ubar - 9.81f) / R(2,2);
    c = CONSTRAIN(c, -maxDescentRate / dt, maxAscentRate / dt);
    thrust = - c * mass;
  ///////////////////////////////////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0

  //  use the gain parameters kpPosXY and kpVelXY
  //  limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // ensure don't have any incoming z-component
    
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // initialize the returned desired acceleration to the feed-forward value.

  V3F accelCmd = accelCmdFF;

  ////////////////////////////////////////////////////////
    
    V3F maxVelCmd;
    // use mag() from V3F
    if (velCmd.mag() > maxSpeedXY) {
        // vector = unit vector (i.e. norm from V3F) * magnitude
        maxVelCmd = velCmd.norm() * maxSpeedXY;
//        printf("maxVelVector: %.3f %.3f %.3f \n", maxVelCmd.x, maxVelCmd.y, maxVelCmd.z);
    }
    else {
        maxVelCmd = velCmd;
    }
    
    float dx = posCmd.x - pos.x;
    float dx_dot = maxVelCmd.x - vel.x;
    float dy = posCmd.y - pos.y;
    float dy_dot = maxVelCmd.y - vel.y;
    
    float x_dd = kpPosXY * dx + kpVelXY * dx_dot + accelCmdFF.x;
    float y_dd = kpPosXY * dy + kpVelXY * dy_dot + accelCmdFF.y;
    
    accelCmd.x = CONSTRAIN(x_dd, -maxAccelXY, maxAccelXY);
    accelCmd.y = CONSTRAIN(y_dd, -maxAccelXY, maxAccelXY);
    
    if (accelCmd.mag() > maxAccelXY) {
        accelCmd = accelCmd.norm() * maxAccelXY;
    }

  //////////////////////////////////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]

  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  //////////////////////////////////////////////////////////

    yawRateCmd = kpYaw * (yawCmd - fmodf(yaw, 3.1415f));

  /////////////////////////////// ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
