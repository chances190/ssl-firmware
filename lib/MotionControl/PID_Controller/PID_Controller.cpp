
#include "PID_Controller.h"

PID_Controller::PID_Controller(PinName pwm,
                               PinName dir,
                               PinName rst,
                               PinName mode,
                               PinName cst,
                               PinName brk,
                               TIM_TypeDef* timer,
                               int pulsesPerRev,
                               int tsample,
                               MotorType M_TYPE) :
    motor(pwm, dir, rst, mode, cst, brk),
    last_error(MAX_ERROR_SIZE, 0) {
  enc = new QEI(pulsesPerRev, tsample, timer);
  pin_pwm = pwm;
  _tsample = tsample;
  if (M_TYPE == DRIBBLER)
    dribblerConfig();
}

PID_Controller::PID_Controller(PinName pwm,
                               PinName dir,
                               PinName rst,
                               PinName mode,
                               PinName cst,
                               PinName brk,
                               PinName channelA,
                               PinName channelB,
                               int pulsesPerRev,
                               int tsample,
                               MotorType M_TYPE) :
    motor(pwm, dir, rst, mode, cst, brk),
    last_error(MAX_ERROR_SIZE, 0) {
  enc = new QEInterruption(channelA, channelB, pulsesPerRev, tsample);
  pin_pwm = pwm;
  _tsample = tsample;
  if (M_TYPE == DRIBBLER)
    dribblerConfig();
}

void PID_Controller::init() {
  motor.init();
  enc->init();
  _controlTicker.attach(callback(this, &PID_Controller::update), chrono::milliseconds(_tsample));
  _controlTimeout.start();
}

void PID_Controller::reset() {
  _desired_rad_s = 0;
  _stopControl = true;
  motor.reset();
}

void PID_Controller::stop() {
  _desired_rad_s = 0;
  _stopControl = true;
  motor_timer.reset();
  motor.stop();
}

// Callback that updates the encoder frequency and PID control.
void PID_Controller::update() {
  enc->frequency();
  if (_stopControl ||
      (!motor_error && utils::timerMillisExpired(_controlTimeout, NRF_NO_MSG_TIME))) {
    stop();
  } else {
    runPID(_desired_rad_s);
  }
}

void PID_Controller::updateDesiredSpeed(double desired_rad_s) {
  if (!motor_error) {
    _stopControl = false;
    _desired_rad_s = desired_rad_s;
    _controlTimeout.reset();
    debug_if(DEBUG,
             "PID_Controller::updateDesiredSpeed: %lf, Flag: %d, Timeout: %lf\n",
             _desired_rad_s,
             _stopControl,
             utils::timerMillisRead(_controlTimeout));
  } else {
    _desired_rad_s = 0;
    if (motor_error_armed_warning) {
      utils::beep(300);
      motor_error_armed_warning = false;
    }
  }
}

double PID_Controller::runPID(double desired_rad_s, MotorType M_TYPE) {
  if (fabs(desired_rad_s) < MOTOR_MIN_SPEED_RAD_S && desired_rad_s != 0) {
    desired_rad_s = 0;
  }
  // Updates de Encoder
  double pwm;
  if (M_TYPE == DIRECTION)
    pwm = adjust_rad_s_to_pwm_pi(desired_rad_s); // Comment to use OLD PID
  else
    pwm = dribbler_adjust_rad_s_to_pwm(desired_rad_s);

  is_motor_running(desired_rad_s);
  if (!motor_error) {
    runPWM(pwm);
  } else {
    if (pin_pwm == M1_PWM) {
      Status::send(FAIL, MOTOR_1_LED);
    } else if (pin_pwm == M2_PWM) {
      Status::send(FAIL, MOTOR_2_LED);
    } else if (pin_pwm == M3_PWM) {
      Status::send(FAIL, MOTOR_3_LED);
    } else if (pin_pwm == M4_PWM) {
      Status::send(FAIL, MOTOR_4_LED);
    } else if (pin_pwm == M5_PWM) {
      Status::send(FAIL, DRIBBLER_LED);
    }
    motor.stop();
  }
}

void PID_Controller::runPWM(double desired_pwm) {
  double MAX_PWM_ALLOWED = (motor_is_locked) ? LOCKED_MAX_PWM_ALLOWED : STD_MAX_PWM_ALLOWED;

  double limited_pwm = std::min(desired_pwm, MAX_PWM_ALLOWED);
  double limited_pwm = std::max(desired_pwm, -MAX_PWM_ALLOWED);

  _pwm = limited_pwm;
  motor.run(_pwm);
  _speed_rad_s = enc->getSpeed();
}

double PID_Controller::adjust_rad_s_to_pwm(double desired_rad_s) {
  _speed_rad_s = enc->getSpeed();

  if ((alpha > 0) && (fabs(desired_rad_s - _speed_rad_s) > accel_diff_speed)) {
    desired_rad_s = (alpha * desired_rad_s) + ((1 - alpha) * _speed_rad_s);
  }

  double adjust_rad_s = pid(desired_rad_s, _speed_rad_s);

  double adjust_pwm =
      utils::rad_s_to_pwm(fmin(fmax(adjust_rad_s, -max_adjust_speed), max_adjust_speed));
  double desired_pwm = utils::rad_s_to_pwm(desired_rad_s);

  return desired_pwm + adjust_pwm;
}

double PID_Controller::adjust_rad_s_to_pwm_pi(double desired_rad_s) {
  _speed_rad_s = enc->getSpeed();
  double adjust_pwm;
  adjust_pwm = pi(desired_rad_s, _speed_rad_s);

  return adjust_pwm;
}

double PID_Controller::dribbler_adjust_rad_s_to_pwm(double desired_rad_s) {

  _speed_rad_s = dribbler_get_speed();

  if ((alpha > 0) && (fabs(desired_rad_s - _speed_rad_s) > accel_diff_speed)) {
    desired_rad_s = (alpha * desired_rad_s) + ((1 - alpha) * _speed_rad_s);
  }

  double adjust_rad_s = pid(desired_rad_s, _speed_rad_s);
  double adjusted_rad_s =
      desired_rad_s + fmin(fmax(adjust_rad_s, -max_adjust_speed), max_adjust_speed);

  return 0;
}

double PID_Controller::pid(double desired, double measured) {
  double error = desired - measured;

  double proportional = Kp * error;
  double derivative = Kd * (error - last_error.back());

  last_error.push_back(error);
  if (last_error.size() > MAX_ERROR_SIZE) {
    last_error.erase(last_error.begin());
  }

  double sum_of_errors = std::accumulate(last_error.begin(), last_error.end(), 0);
  double integrative = Ki * sum_of_errors;

  return proportional + integrative + derivative;
}

double PID_Controller::pi(double desired, double measured) {
  double error = desired - measured; // erro em rad/s
  
  if(motor_error) {
    errSum = 0;
  }
  
  P = Kp * error;
  I = Ki * errSum;
  D = Kd * (error - derivative_last_error);
  derivative_last_error = error;

  double pwm = P + I + D;

  errSum += error;
  errSum = std::min(errSum, MAX_I);
  errSum = std::max(errSum, -MAX_I);

  return pwm;
}

void PID_Controller::is_motor_running(double desired_rad_s) {
  if ((fabs(desired_rad_s) >= MOTOR_MIN_SPEED_RAD_S) &&
      (fabs(_speed_rad_s) <= MOTOR_STOP_SPEED_RAD_S)) {
    if (!motor_error) // TIMER NOT STARTED
    {
      motor_timer.start();
    } else {
      if (utils::timerRead<chrono::milliseconds>(motor_timer) > 5 * MOTOR_TIMEOUT &&
          motor_error_rst < 5) {
        motor_timer.reset();
        motor_timer.stop();
        motor_error_rst++;
        motor_error = false;
      }
    }
  } else {
    motor_error = false;
    motor_timer.stop();
    motor_timer.reset();
  }

  if (utils::timerRead<chrono::milliseconds>(motor_timer) > MOTOR_TIMEOUT) {
    motor_error = true;
    motor_error_armed_warning = true;
  }
  return;
}

double PID_Controller::get_speed_pwm() {
  _speed_rad_s = enc->getSpeed();
  return utils::rad_s_to_pwm(_speed_rad_s);
}

double PID_Controller::get_speed_rad_s() {
  _speed_rad_s = enc->getSpeed();
  return _speed_rad_s;
}

double PID_Controller::get_desired_speed_rad_s() {
  return _desired_rad_s;
}

double PID_Controller::get_current_pwm() {
  return _pwm;
}

bool PID_Controller::get_motor_error() {
  return motor_error;
}

double PID_Controller::dribbler_get_speed() {
  return enc->getSpeed();
}

void PID_Controller::dribblerConfig() {

  accel_diff_speed = M5_accel_diff_speed;
  max_adjust_speed = M5_max_adjust_speed;

  alpha = M5_alpha;
  Kp = M5_Kp;
  Ki = M5_Ki;
  Kd = M5_Kd;
}

void PID_Controller::setMotorIsLocked(bool robotMoveIsLocked) {
  motor_is_locked = robotMoveIsLocked;
}
