#ifndef _LINKBOT_H_
#define _LINKBOT_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef enum robotJointState_e
{
    ROBOT_NEUTRAL = 0,
    ROBOT_FORWARD,
    ROBOT_BACKWARD,
    ROBOT_HOLD,
    ROBOT_POSITIVE,
    ROBOT_NEGATIVE,
    ROBOT_ACCEL,
} robotJointState_t;

typedef enum mobotFormFactor_e
{
  MOBOTFORM_NULL,
  MOBOTFORM_ORIGINAL,
  MOBOTFORM_I,
  MOBOTFORM_L,
  MOBOTFORM_T,
}mobotFormFactor_t;

class Linkbot {
  public:
    Linkbot(uint16_t zigbee_addr = 0);
    ~Linkbot();

    int checkStatus();
    int driveJointTo(int joint, float angle);
    int driveJointToNB(int joint, float angle);
    int driveTo(float angle1, float angle2, float angle3);
    int driveToNB(float angle1, float angle2, float angle3);
    int getAccelerometerData(float &x, float &y, float &z);
    int getBatteryVoltage(float &volts);
    int getColorRGB(uint8_t &r, uint8_t &g, uint8_t &b);
    int getFormFactor(int &form);
    int getJointAngle(int joint, float &angle);
    int getJointAngles(float &angle1, float &angle2, float &angle3);
    int isMoving();
    int moveJoint(int joint, float angle);
    int moveJointNB(int joint, float angle);
    int moveJointTo(int joint, float angle);
    int moveJointToNB(int joint, float angle);
    int move(float angle1, float angle2, float angle3);
    int moveNB(float angle1, float angle2, float angle3);
    int moveTo(float angle1, float angle2, float angle3);
    int moveToNB(float angle1, float angle2, float angle3);
    int moveWait();
    int reset();
    int resetToZero();
    int setJointSpeed(int joint, float speed);
    int setJointSpeeds(float speed1, float speed2, float speed3);
    int setJointState(int joint, int state);
    int setJointStates(int state1, int state2, int state3);
    int setLEDColor(uint8_t r, uint8_t g, uint8_t b);
    int setMotorPower(int joint, int power);
    int setMotorPowers(int power1, int power2, int power3);
    int stop();

  private:
    uint16_t _zigbee_addr;
    uint8_t _buf[64];
    uint8_t _bufsize;
    void packBufReset();
    void packBufByte(uint8_t byte);
    void packBuf(void* data, int size);
    void packSimpleCmd(uint8_t cmd);
    int transactMessage();
};

#endif
