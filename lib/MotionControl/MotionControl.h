#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "mbed.h"
#include <PID_Controller.h>
#include <kicker.h>
#include <kinematics.h>
#include <nRF24Communication.h>
#include <utils.h>

#define HALT_SPEED 8

#ifdef USING_NEW_MOTOR
  #define MOTOR_ENCODER_PULSES 2048
#else
  #define MOTOR_ENCODER_PULSES 1024
#endif

class MotionControl {
 public:
  MotionControl(Kinematics* kinematics);

  void init();

  Motors moveRobot(Vector desiredVector);
  Motors accelRobot(Vector desiredVector);
  Motors accelRobot(Vector desiredVector, Vector currentPosition);
  Motors slowRobot();
  void moveRobotPID(double m1, double m2, double m3, double m4);
  void moveRobotPID(double speed);
  void moveRobotPWM(double m1, double m2, double m3, double m4);
  void moveRobotPWM(double speed);
  void moveMotorPID(uint8_t idx, double speed);
  void moveMotorPWM(uint8_t idx, double speed);
  void updateDribbler(KickFlags& isKick);
  void getMotorsInfo(RobotInfo& telemetry);
  void getMotorsSpeed(Motors& m);
  void getMotorsDesiredSpeed(Motors& m);
  void getMotorsPWM(Motors& m);
  double getMotorSpeed(uint8_t idx);

  void getRobotSpeed(Vector& v);
  void getRobotSpeedOriginal(Vector& v);
  void getRobotSpeed(Vector& v, bool useOptimizedParamaters);

  bool robotIsMoving();
  void stopRobot();
  void stopMotor(uint8_t idx);
  void breakRobot(double haltSpeed = HALT_SPEED);

  Motors convertWheelSpeed(double vx, double vy, double w);
  Motors convertWheelSpeed(Vector& desiredVector);

  void printMotorsSpeed();
  void printMotorsPWM();
  void printDebugPID(double desired);
  void printDebugPID(Vector desired, bool debug = false);
  void printDesiredSpeed(Vector motionSpeed);
  void printDriblerSpeed();
  void validateNavigation();
  void setMoveIsLocked(bool robotMoveIsLocked);

 private:
  PID_Controller _M1;
  PID_Controller _M2;
  PID_Controller _M3;
  PID_Controller _M4;
  PID_Controller _M5;

  Kinematics* _kinematics;
  Motors _motorSpeeds;

  static bool shouldStop(double motorSpeed, double haltSpeed);
};

#endif // MOTION_CONTROL_H
