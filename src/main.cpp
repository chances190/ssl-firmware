// RobôCIn SSL - v1.4
#include "defines.h"
#include "utils.h"
#include <Debugger.h>
#include <MotionControl.h>
#include <Navigation.h>
#include <Odometry.h>
#include <kicker.h>
#include <CurrentSensor.h>
#include <nRF24Communication.h>
#include <EthCommunication.h>
#include <status.h>

#define ROBOT_MODE (ExecMode::GAME_ON)

nRF24Communication radio_recv(NRF_R_MOSI,
                              NRF_R_MISO,
                              NRF_R_SCK,
                              NRF_R_CE,
                              NRF_R_CSN,
                              NRF_R_VCC,
                              NetworkType::ssl,
                              RadioFunction::receiver);

nRF24Communication radio_send(NRF_S_MOSI,
                              NRF_S_MISO,
                              NRF_S_SCK,
                              NRF_S_CE,
                              NRF_S_CSN,
                              NRF_S_VCC,
                              NetworkType::ssl,
                              RadioFunction::sender);

EthCommunication eth_communication(ip, netmask, gateway, port);

Kicker kick(PIN_CHARGE, PIN_CAP_LOAD, PIN_FRONT, PIN_CHIP, PIN_IR_LED, PIN_IR_ADC);

KickFlags isKick;
RobotInfo robotInfo;
Vector motionSpeed;
Vector currentSpeed;
Kinematics kinematics;
RobotPosition packetPos;
CurrentSensor currentSensor;

MotionControl motion(&kinematics);
Odometry odometry(&motion, &kinematics, ODOMETRY_TSAMPLE, GYRO_TSAMPLE); // Odm at 5ms, Gyro as 5ms
Navigation navigation(&kinematics, &motion, &odometry);
Debugger debbuger(&motion, &odometry, &kick, &radio_recv, &radio_send, &currentSensor);

static BufferedSerial serial_port(USBTX, USBRX, 230400);
FileHandle* mbed::mbed_override_console(int fd) {
  return &serial_port;
}

Timer msgTimeout;
Timer msgTelemetry;

uint8_t pcktCount = 0;

int main() {
  kick.init();
  utils::initRobot();
  utils::checkBattery();

  motion.init();
  currentSensor.init();

  radio_recv.setup(utils::getRobotId());
  radio_send.setup(utils::getRobotId());
  radio_recv.enable();

  msgType recvType = msgType::NONE;

  ThisThread::sleep_for(100ms);
  Status::clearColors();
  msgTimeout.start();
  msgTelemetry.start();

  debbuger.processRobot(ROBOT_MODE);

  // Init odometry after debugger for avoiding calibration beep
  odometry.init();

  // GAME_ON
  while (1) {
    // Process Gyroscope
    odometry.processGyro();

    motion.setMoveIsLocked(currentSensor.isMotorLocked());

    // New packet?
    bool newPacket = false;

    // Tries to receive radio message!
    if (radio_recv.updatePacket() && radio_recv.getGameState() != refereeCommand::halt) {
      recvType = radio_recv.getPacketType();
      msgTimeout.reset();
      newPacket = true;
    }

    // Find the speed to navigate robot.
    if (recvType == msgType::SSL_SPEED) {
      // Check movement lock
      motion.setMoveIsLocked(radio_recv.robotMoveIsLocked());
      // Update desired speed
      navigation.update(motionSpeed, radio_recv.getVectorSpeed());
    } else if (recvType == msgType::POSITION) {
      if (newPacket) {
        navigation.update(radio_recv.getLastPosition());
      }
      navigation.move(motionSpeed);
    }

    if (utils::timerMillisExpired(msgTimeout, NRF_NO_MSG_TIME)) {
      if (utils::timerMillisExpired(msgTimeout, NRF_NO_MSG_TIME + NRF_HALT_TIME)) {
        // Coast robot after 1s of break.
        motion.stopRobot();
        if (utils::pressedFor(utils::PBT1, 2000) == ButtonState::HELD ||
            utils::pressedFor(utils::PBT2, 2000) == ButtonState::HELD) {
          debbuger.processRobot(ExecMode::TEST);
        }
      } else {
        // Brake robot.
        motion.accelRobot(Vector(0, 0, 0));
      }
      kick.stopCharge();
    } else {
      // Move robot with speed
      motion.accelRobot(motionSpeed);
      // Control Kicker
      radio_recv.getKick(isKick);
      kick.update(isKick,
                  ((radio_recv.getGameState() == refereeCommand::ballPlacementYellow ||
                    radio_recv.getGameState() == refereeCommand::ballPlacementBlue) ?
                       BALL_PLACEMENT_MIN_CHARGE :
                       DEFAULT_MIN_CHARGE),
                  MAX_CHARGE);
      // Control Dribbler
      motion.updateDribbler(isKick);
    }

#ifdef DEFAULT_PCKT
    // Sending Telemetry
    if (utils::timerRead<chrono::milliseconds>(msgTelemetry) > DEFAULT_FEEDBACK_TIME) {
      motion.getMotorsInfo(robotInfo);
      robotInfo.current = currentSensor.getMotorsCurrent();
      kick.getKickerInfo(robotInfo.kickLoad, robotInfo.ball);
      robotInfo.battery = utils::getBattery();
      robotInfo.type = msgType::TELEMETRY;
      robotInfo.count = pcktCount++;
      radio_send.sendTelemetryPacket(robotInfo);
      msgTelemetry.reset();
    }
#endif
#ifdef ODOMETRY_PCKT
    // Sending Odometry
    if (utils::timerRead<chrono::milliseconds>(msgTelemetry) > ODOMETRY_TIME) {
      // Get robot position
      odometry.getCurrentPosition(robotInfo.v);
      // Get motors speed
      motion.getMotorsInfo(robotInfo);
      motion.getRobotSpeed(currentSpeed);
      kick.getKickerInfo(robotInfo.kickLoad, robotInfo.ball);
      robotInfo.battery = utils::getBattery();
      robotInfo.type = msgType::ODOMETRY;
      robotInfo.count = pcktCount++;
      radio_send.sendOdometryPacket(robotInfo);
      msgTelemetry.reset();
    }
#endif
  }
}
