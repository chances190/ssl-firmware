#include "MotionControl.h"


MotionControl::MotionControl(Kinematics* kinematics) :
    _M1(M1_PWM, M1_DIR, M1_RST, M1_MODE, M1_CST, M1_BRK, TIM1, MOTOR_ENCODER_PULSES, 2), // sampling time 2ms
    _M2(M2_PWM, M2_DIR, M2_RST, M2_MODE, M2_CST, M2_BRK, TIM2, MOTOR_ENCODER_PULSES, 2),
    _M3(M3_PWM, M3_DIR, M3_RST, M3_MODE, M3_CST, M3_BRK, TIM8, MOTOR_ENCODER_PULSES, 2),
    _M4(M4_PWM, M4_DIR, M4_RST, M4_MODE, M4_CST, M4_BRK, TIM3, MOTOR_ENCODER_PULSES, 2),
    _M5(M5_PWM, M5_DIR, M5_RST, M5_MODE, M5_CST, M5_BRK, M5_TAC, M5_DIRO, 6, 70, DRIBBLER) {
  _kinematics = kinematics;
}

void MotionControl::init() {
  _M1.init();
  _M2.init();
  _M3.init();
  _M4.init();
  _M5.init();
}

Motors MotionControl::moveRobot(Vector desiredVector) {
  _motorSpeeds = this->convertWheelSpeed(desiredVector);
  moveRobotPID(_motorSpeeds.m1, _motorSpeeds.m2, _motorSpeeds.m3, _motorSpeeds.m4);
  return _motorSpeeds;
}

Motors MotionControl::accelRobot(Vector desiredVector) {
  Vector currVector;

  this->getRobotSpeedOriginal(currVector);

  Vector diffVector = desiredVector - currVector;
  double diffNorm = sqrt(diffVector.x * diffVector.x + diffVector.y * diffVector.y);
  if (diffNorm < 0.001)
    return this->moveRobot(desiredVector);
  // For bypassing acceleration limitation, use line below:
  return this->moveRobot(desiredVector);

  double diffMax = 0.1;
  double alpha = std::min((diffNorm), diffMax) / (diffNorm);

  desiredVector.x = currVector.x + diffVector.x * alpha;
  desiredVector.y = currVector.y + diffVector.y * alpha;

  // return this->moveRobot(desiredVector);
}

Motors MotionControl::accelRobot(Vector desiredVector, Vector currentPosition) {
  // Rotate vector with past robot's movement
  Vector fixDesiredVector = _kinematics->rotateVector(-currentPosition.w, desiredVector);
  return this->accelRobot(fixDesiredVector);
}

Motors MotionControl::slowRobot() {
  // Slow down robot
  Vector currVector;
  return this->accelRobot(currVector);
}

void MotionControl::moveRobotPID(double m1, double m2, double m3, double m4) {
  _M1.updateDesiredSpeed(m1);
  _M2.updateDesiredSpeed(m2);
  _M3.updateDesiredSpeed(m3);
  _M4.updateDesiredSpeed(m4);
}

void MotionControl::moveRobotPID(double speed) {
  moveRobotPID(speed, speed, speed, speed);
}

void MotionControl::moveRobotPWM(double m1, double m2, double m3, double m4) {
  _M1.runPWM(m1);
  _M2.runPWM(m2);
  _M3.runPWM(m3);
  _M4.runPWM(m4);
}

void MotionControl::moveRobotPWM(double speed) {
  _M1.runPWM(speed);
  _M2.runPWM(speed);
  _M3.runPWM(speed);
  _M4.runPWM(speed);
}

void MotionControl::moveMotorPID(uint8_t idx, double speed) {
  if (idx == 1) {
    _M1.updateDesiredSpeed(speed);
  } else if (idx == 2) {
    _M2.updateDesiredSpeed(speed);
  } else if (idx == 3) {
    _M3.updateDesiredSpeed(speed);
  } else if (idx == 4) {
    _M4.updateDesiredSpeed(speed);
  } else if (idx == 5) {
    _M5.runPID(speed, DRIBBLER);
  }
}

void MotionControl::moveMotorPWM(uint8_t idx, double speed) {
  if (idx == 1) {
    _M1.runPWM(speed);
  } else if (idx == 2) {
    _M2.runPWM(speed);
  } else if (idx == 3) {
    _M3.runPWM(speed);
  } else if (idx == 4) {
    _M4.runPWM(speed);
  } else if (idx == 5) {
    _M5.runPWM(speed);
  }
}

void MotionControl::updateDribbler(KickFlags& isKick) {
  // Control Dribbler
  if (isKick.dribbler) {
    double d_speed =
        fabs(isKick.dribblerSpeed) >= 0.1 ? isKick.dribblerSpeed : DEFAULT_DRIBLER_SPEED;
    this->moveMotorPWM(5, d_speed);
  } else {
    this->stopMotor(5);
  }
}

void MotionControl::getMotorsInfo(RobotInfo& telemetry) {
  telemetry.m.m1 = _M1.get_speed_rad_s();
  telemetry.m.m2 = _M2.get_speed_rad_s();
  telemetry.m.m3 = _M3.get_speed_rad_s();
  telemetry.m.m4 = _M4.get_speed_rad_s();
  telemetry.dribbler = _M5.get_speed_rad_s();
}

void MotionControl::getMotorsSpeed(Motors& m) {
  m.m1 = _M1.get_speed_rad_s();
  m.m2 = _M2.get_speed_rad_s();
  m.m3 = _M3.get_speed_rad_s();
  m.m4 = _M4.get_speed_rad_s();
}

void MotionControl::getMotorsDesiredSpeed(Motors& m) {
  m.m1 = _M1.get_desired_speed_rad_s();
  m.m2 = _M2.get_desired_speed_rad_s();
  m.m3 = _M3.get_desired_speed_rad_s();
  m.m4 = _M4.get_desired_speed_rad_s();
}

void MotionControl::getMotorsPWM(Motors& m) {
  m.m1 = _M1.get_current_pwm();
  m.m2 = _M2.get_current_pwm();
  m.m3 = _M3.get_current_pwm();
  m.m4 = _M4.get_current_pwm();
}

bool MotionControl::robotIsMoving() {
  Motors m;
  this->getMotorsSpeed(m);
  if (m.m1 == 0 && m.m2 == 0 && m.m3 == 0 && m.m4 == 0)
    return false;
  return true;
}

double MotionControl::getMotorSpeed(uint8_t idx) {
  double speed = 0;
  if (idx == 1) {
    speed = _M1.get_speed_rad_s();
  } else if (idx == 2) {
    speed = _M2.get_speed_rad_s();
  } else if (idx == 3) {
    speed = _M3.get_speed_rad_s();
  } else if (idx == 4) {
    speed = _M4.get_speed_rad_s();
  } else if (idx == 5) {
    speed = _M5.get_speed_rad_s();
  }
  return speed;
}

void MotionControl::getRobotSpeed(Vector& v) {
  Motors speeds;
  this->getMotorsSpeed(speeds);
  v = _kinematics->convertToVector(speeds);
}

void MotionControl::getRobotSpeedOriginal(Vector& v) {
  Motors speeds;
  this->getMotorsSpeed(speeds);
  v = _kinematics->convertToVectorOriginal(speeds);
}

void MotionControl::getRobotSpeed(Vector& v, bool useOptimizedParamaters) {
  if (useOptimizedParamaters) {
    this->getRobotSpeed(v);
  } else {
    this->getRobotSpeedOriginal(v);
  }
}

void MotionControl::stopRobot() {
  _M1.stop();
  _M2.stop();
  _M3.stop();
  _M4.stop();
  _M5.stop();
}

void MotionControl::stopMotor(uint8_t idx) {
  if (idx == 1) {
    _M1.stop();
  } else if (idx == 2) {
    _M2.stop();
  } else if (idx == 3) {
    _M3.stop();
  } else if (idx == 4) {
    _M4.stop();
  } else if (idx == 5) {
    _M5.stop();
  }
}

void MotionControl::breakRobot(double haltSpeed) {
  if (this->shouldStop(_M1.get_speed_rad_s(), haltSpeed))
    _M1.runPID(0);
  else
    _M1.stop();
  if (this->shouldStop(_M2.get_speed_rad_s(), haltSpeed))
    _M2.runPID(0);
  else
    _M2.stop();
  if (this->shouldStop(_M3.get_speed_rad_s(), haltSpeed))
    _M3.runPID(0);
  else
    _M3.stop();
  if (this->shouldStop(_M4.get_speed_rad_s(), haltSpeed))
    _M4.runPID(0);
  else
    _M4.stop();

  _motorSpeeds = _kinematics->clearSpeeds();
}

Motors MotionControl::convertWheelSpeed(double vx, double vy, double w) {
  return _kinematics->convertToWheel(vx, vy, w);
}

Motors MotionControl::convertWheelSpeed(Vector& desiredVector) {
  return _kinematics->convertToWheel(desiredVector.x, desiredVector.y, desiredVector.w);
}

void MotionControl::printMotorsSpeed() {
  printf("v-M1: %.3lf  | v-M2: %.3lf | v-M3: %.3lf  | v-M4: %.3lf\n",
         _M1.get_speed_rad_s(),
         _M2.get_speed_rad_s(),
         _M3.get_speed_rad_s(),
         _M4.get_speed_rad_s());
}

void MotionControl::printDesiredSpeed(Vector motionSpeed) {
  printf("vx: %.3lf  | vy: %.3lf | w: %.3lf\n", motionSpeed.x, motionSpeed.y, motionSpeed.w);
}

void MotionControl::printDebugPID(double desired) {
  printf("v %.3lf M1 %.3lf  M2 %.3lf M3 %.3lf M4 %.3lf\n",
         desired,
         _M1.get_speed_rad_s(),
         _M2.get_speed_rad_s(),
         _M3.get_speed_rad_s(),
         _M4.get_speed_rad_s());
}

void MotionControl::printDebugPID(Vector desired, bool debug) {
  Motors desiredSpeeds = _kinematics->convertToWheel(desired.x, desired.y, desired.w);
  if (debug) {
    printf(
        "V1 %.3lf | V2 %.3lf | V3 %.3lf | V4 %.3lf || M1 %.3lf | M2 %.3lf | M3 %.3lf | M4 %.3lf\n",
        desiredSpeeds.m1,
        desiredSpeeds.m2,
        desiredSpeeds.m3,
        desiredSpeeds.m4,
        _M1.get_speed_rad_s(),
        _M2.get_speed_rad_s(),
        _M3.get_speed_rad_s(),
        _M4.get_speed_rad_s());
  } else {
    printf("%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n",
           desiredSpeeds.m1,
           desiredSpeeds.m2,
           desiredSpeeds.m3,
           desiredSpeeds.m4,
           _M1.get_speed_rad_s(),
           _M2.get_speed_rad_s(),
           _M3.get_speed_rad_s(),
           _M4.get_speed_rad_s());
  }
}

bool MotionControl::shouldStop(double motorSpeed, double haltSpeed) {
  if (fabs(motorSpeed) > haltSpeed)
    return true;
  else
    return false;
}

void MotionControl::printDriblerSpeed() {
  printf("%.3f\n", _M5.dribbler_get_speed());
}

void MotionControl::validateNavigation() {
  Vector v_test;
  v_test.x = 1;
  _kinematics->validateSpeeds(v_test);
}

void MotionControl::setMoveIsLocked(bool robotMoveIsLocked) {
  _M1.setMotorIsLocked(robotMoveIsLocked);
  _M2.setMotorIsLocked(robotMoveIsLocked);
  _M3.setMotorIsLocked(robotMoveIsLocked);
  _M4.setMotorIsLocked(robotMoveIsLocked);
}

void MotionControl::printMotorsPWM() {
  printf("M1 %.3lf  M2 %.3lf M3 %.3lf M4 %.3lf\n",
         _M1.get_current_pwm(),
         _M2.get_current_pwm(),
         _M3.get_current_pwm(),
         _M4.get_current_pwm());
}
