#include "CurrentSensor.h"

CurrentSensor::CurrentSensor(int tsample){
  _tsample = tsample;
  maxMeasuresAboveThreshold = (MAX_SECONDS_ABOVE_THRESHOLD * 1000000) / _tsample;
};

void CurrentSensor::setMuxWiremap(Wiremap& wiremap) {
  this->A2.write(wiremap.a2);
  this->A1.write(wiremap.a1);
  this->A0.write(wiremap.a0);
}

double CurrentSensor::calculateCurrentFromVoltage(Voltage voltage) {
  double vref = voltage.first * 3.3;
  double vout = voltage.second * 3.3;

  return (vout - vref) / SENSITIVITY;
}

Voltage CurrentSensor::readVoltageFromMuxOutput() {
  return std::make_pair(vReference.read(), vOut.read());
}

void CurrentSensor::init() {
  checkCurrent.attach(callback(this, &CurrentSensor::checkCurrentStatus), chrono::microseconds(_tsample));
}

Motors CurrentSensor::getMotorsCurrent() {
  Voltage voltage;
  Motors motorCurrent;

  // Motor 1
  this->setMuxWiremap(this->motorOneWiremap);
  voltage = this->readVoltageFromMuxOutput();
  motorCurrent.m1 = this->calculateCurrentFromVoltage(voltage);

  // Motor 2
  this->setMuxWiremap(this->motorTwoWiremap);
  voltage = this->readVoltageFromMuxOutput();
  motorCurrent.m2 = this->calculateCurrentFromVoltage(voltage);

  // Motor 3
  this->setMuxWiremap(this->motorThreeWiremap);
  voltage = this->readVoltageFromMuxOutput();
  motorCurrent.m3 = this->calculateCurrentFromVoltage(voltage);

  // Motor 4
  this->setMuxWiremap(this->motorFourWiremap);
  voltage = this->readVoltageFromMuxOutput();
  motorCurrent.m4 = this->calculateCurrentFromVoltage(voltage);

  return motorCurrent;
}

double CurrentSensor::getDribblerCurrent() {
  // Setup wiremap to dribbler output
  this->setMuxWiremap(this->dribblerWiremap);

  // Read output
  Voltage voltage = this->readVoltageFromMuxOutput();

  // Do not use utils getCurrent, refactor this later on
  return this->calculateCurrentFromVoltage(voltage);
}

bool CurrentSensor::isMotorLocked() {
  return _isLocked;
}

    // Update moving windows and calculate moving max
double CurrentSensor::updateWindow(double* window, int& index, double newValue) {
      window[index] = newValue; // Add the new value to the window
      index = (index + 1) % WINDOW_SIZE; // Move to the next index

      double maxCurrent;
      for (int i = 0; i < WINDOW_SIZE; ++i) {
        maxCurrent = std::max(maxCurrent, window[i]);
      }
      return maxCurrent;
};

void CurrentSensor::checkCurrentStatus() {
  Motors motorsCurrent = getMotorsCurrent();

  double maxM1 = updateWindow(_motor1Window, _motor1Index, std::abs(motorsCurrent.m1));
  double maxM2 = updateWindow(_motor2Window, _motor2Index, std::abs(motorsCurrent.m2));
  double maxM3 = updateWindow(_motor3Window, _motor3Index, std::abs(motorsCurrent.m3));
  double maxM4 = updateWindow(_motor4Window, _motor4Index, std::abs(motorsCurrent.m4));

  // Check if any motor's max current exceeds the lock threshold
  bool aboveThreshold = (maxM1 > CURRENT_THRESHOLD ||
                         maxM2 > CURRENT_THRESHOLD || 
                         maxM3 > CURRENT_THRESHOLD || 
                         maxM4 > CURRENT_THRESHOLD);

  if (aboveThreshold) {
    lockedCount++;
    if (lockedCount > maxMeasuresAboveThreshold) {
      unlockedCount = 0;
      _isLocked = true;
    }
  } else {
    unlockedCount++;
    if (unlockedCount > maxMeasuresAboveThreshold) {
      lockedCount = 0;
      _isLocked = false;
    }
  }
}
