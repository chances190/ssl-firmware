#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <utils.h>

#define SENSITIVITY 0.025
#define WINDOW_SIZE 100 
#define CURRENT_THRESHOLD 16
#define MAX_SECONDS_ABOVE_THRESHOLD 1

using Voltage = std::pair<double, double>;

class CurrentSensor {
 public:
  class Wiremap {
   public:
    int a0;
    int a1;
    int a2;

    Wiremap(int a0, int a1, int a2) : a0(a0), a1(a1), a2(a2){};
  };
  
  // tsample is in microseconds
  CurrentSensor(int tsample);
  void init();
  double getDribblerCurrent();
  Motors getMotorsCurrent();
  bool isMotorLocked(); 
  
 private:
  DigitalOut A2 = CURRENT_SENSOR_A2;
  DigitalOut A1 = CURRENT_SENSOR_A1;
  DigitalOut A0 = CURRENT_SENSOR_A0;
  AnalogIn vReference = PIN_CURR_REF;
  AnalogIn vOut = PIN_CURR_OUT;
  Wiremap motorOneWiremap{0, 0, 1};
  Wiremap motorTwoWiremap{0, 1, 1};
  Wiremap motorThreeWiremap{1, 1, 1};
  Wiremap motorFourWiremap{1, 0, 0};
  Wiremap dribblerWiremap{0, 1, 0};
  Timer _timer;
  
  int _tsample;
  int maxMeasuresAboveThreshold;
  uint32_t lockedCount = 0;
  uint32_t unlockedCount = 0;
  Ticker checkCurrent;
  void checkCurrentStatus(); // Callback para checar periodicamente se há um motor travado
  bool _isLocked = false;
  
  void setMuxWiremap(Wiremap& wiremap);
  double calculateCurrentFromVoltage(Voltage voltage);
  Voltage readVoltageFromMuxOutput();

  // Buffer circular para envelope
  double updateWindow(double* window, int& index, double newValue);
  double _motor1Window[WINDOW_SIZE] = {0};
  double _motor2Window[WINDOW_SIZE] = {0};
  double _motor3Window[WINDOW_SIZE] = {0};
  double _motor4Window[WINDOW_SIZE] = {0};
  int _motor1Index = 0, _motor2Index = 0, _motor3Index = 0, _motor4Index = 0;
};

#endif // CURRENT_SENSOR_H