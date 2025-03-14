#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include "mbed.h"
#include "Encoder/Encoder.h"
#include "Encoder/QEI/QEI.h"
#include "Encoder/QEInterruption/QEInterruption.h"
#include "DriverBLDC/DriverBLDC.h"
#include "utils.h"
#include <status.h>
#include <vector>
#include <numeric>

#define MAX_ERROR_SIZE 2 // Number of PID errors accumulation.

enum MotorType { DRIBBLER, DIRECTION };

class PID_Controller {
 public:
  /** Using motor with Timer Encoder
   * @param tsample Encoder sampling time, between speeds calculations
   */
  PID_Controller(PinName pwm,
                 PinName dir,
                 PinName rst,
                 PinName mode,
                 PinName cst,
                 PinName brk,
                 TIM_TypeDef* timer,
                 int pulsesPerRev,
                 int tsample,
                 MotorType M_TYPE = DIRECTION);

  /** Using motor with Interruption Encoder
   * @param tsample Encoder sampling time, between speeds calculations
   */
  PID_Controller(PinName pwm,
                 PinName dir,
                 PinName rst,
                 PinName mode,
                 PinName cst,
                 PinName brk,
                 PinName channelA,
                 PinName channelB,
                 int pulsesPerRev,
                 int tsample,
                 MotorType M_TYPE = DIRECTION);

  void init();
  void stop();
  void reset();
  void runPWM(double desired_pwm);
  void updateDesiredSpeed(double desired_rad_s);
  void setMotorIsLocked(bool robotMoveIsLocked);
  double runPID(double desired_rad_s, MotorType M_TYPE = DIRECTION);
  double runPID_PWM(double desired_pwm);
  int getMotorRst();

  // Calculate the necessary PWM given an desired speed.
  double
  adjust_pwm_to_pwm(double desired_pwm); // Speed in PWM (mapped from no loaded motor direction)
  double adjust_rad_s_to_pwm(double desired_rad_s); // Speed in rad/s
  double dribbler_adjust_rad_s_to_pwm(
      double desired_rad_s); // Speed in PWM (mapped from no loaded motor dribbler)
  double adjust_rad_s_to_pwm_pi(double desired_rad_s); // Speed in PWM (from PI controller)

  double get_speed_pwm();   // Wheel speed in rad/s converted to PWM (mapped from no loaded motor)
  double get_speed_rad_s(); // Wheel speed in rad/s
  double get_desired_speed_rad_s();
  double get_current_pwm();
  double get_motion_rad();
  bool get_motor_error();

  // Motor encoder by interruption
  double dribbler_get_speed(); // current mensured Bar Dribbler speed in rad/s

 private:
  void update();                               // Update Encoder reads and PID control
  void is_motor_running(double desired_rad_s); // Checks if motor is properly running
  double pid(double desired, double measured); // With desired and measured outputs control signal
  double pi(double desired, double measured);  // With desired and measured outputs control signal
  void dribblerConfig();                       // configurate constants for dribbler motor

  Ticker _controlTicker;
  Timer _controlTimeout;
  bool _stopControl = true;
  int _tsample;
  Encoder* enc;
  DriverBLDC motor;
  std::vector<double> last_error;

  PinName pin_pwm;

  double _pwm;
  double _speed_rad_s = 0;
  double _desired_rad_s = 0;

  double alpha = 0.2;

  // For control direction motor
  int accel_diff_speed = 25;
  int max_adjust_speed = 40;

  double P = 0;
  double errSum = 0;
  double I = 0;
  const double MAX_I = STD_MAX_PWM_ALLOWED / Ki; // (170/Ki)
  double derivative_last_error = 0;
  double D = 0;

#ifdef USING_NEW_MOTOR
  double Kp = 1.157;  // PID Control constants
  double Ki = 0.0549; // PID Control constants
  double Kd = 0;      // PID Control constants
#else
  double Kp = 1.391;  // PID Control constants
  double Ki = 0.0296; // PID Control constants
  double Kd = 0;      // PID Control constants
#endif

  // For control dribbler motor
  const int M5_accel_diff_speed = 25;
  const int M5_max_adjust_speed = 150;

  double M5_alpha = 0.25;
  double M5_Kp = 2.0;
  double M5_Ki = 0;
  double M5_Kd = 0.02;

  Timer motor_timer;
  const int MOTOR_STOP_SPEED_RAD_S = 1;
  const float MOTOR_TIMEOUT = 1000.0;
  long motor_error_rst = 0;
  bool motor_error = false;
  bool motor_error_armed_warning = false;
  bool motor_is_locked = false;
};

#endif /* PID_CONTROLLER */
