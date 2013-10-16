
#include <Arduino.h>

extern "C" {
#include "Linkbot.h"
#include "utility/commands.h"
#include "utility/twi.h"
#include <math.h>
}

uint8_t g_recvBytes = 0;
uint8_t g_recvBuf[256];

uint8_t g_twiInitialized = 0;

#define DEG2RAD(x) ((x)*M_PI/180.0)
#define RAD2DEG(x) ((x)*180.0/M_PI)

void onSlaveRX(uint8_t *buf, int len)
{
  memcpy(g_recvBuf, buf, len);
  g_recvBytes = len;
}

Linkbot::Linkbot(uint16_t zigbee_addr)
{
  _zigbee_addr = zigbee_addr;
  if(!g_twiInitialized) {
    twi_init();
    twi_setAddress(0x02);
    twi_attachSlaveRxEvent(onSlaveRX);
  }
}

Linkbot::~Linkbot()
{
}

int Linkbot::checkStatus()
{
  packSimpleCmd(BTCMD(CMD_STATUS));
  return transactMessage();
}

int Linkbot::driveJointTo(int joint, float angle)
{
  driveJointToNB(joint, angle);
  return moveWait();
}

int Linkbot::driveJointToNB(int joint, float angle)
{
  angle = DEG2RAD(angle);
  packBufReset();
  packBufByte(BTCMD(CMD_SETMOTORANGLEPID));
  packBufByte(0x00);
  packBufByte((uint8_t)joint);
  packBuf(&angle, 4);
  packBufByte(0x00);
  return transactMessage();
}

int Linkbot::driveTo(float angle1, float angle2, float angle3)
{
  driveToNB(angle1, angle2, angle3);
  return moveWait();
}

int Linkbot::driveToNB(float angle1, float angle2, float angle3)
{
  angle1 = DEG2RAD(angle1);
  angle2 = DEG2RAD(angle2);
  angle3 = DEG2RAD(angle3);
  packBufReset();
  packBufByte(BTCMD(CMD_SETMOTORANGLESPID));
  packBufByte(0x00);
  packBuf(&angle1, 4);
  packBuf(&angle2, 4);
  packBuf(&angle3, 4);
  packBuf(&angle3, 4);
  packBufByte(0x00);
  return transactMessage();
}

int Linkbot::getAccelerometerData(float &x, float &y, float &z)
{
  packSimpleCmd(BTCMD(CMD_GETACCEL));
  transactMessage();
  memcpy(&x, &_buf[2], 4);
  memcpy(&y, &_buf[6], 4);
  memcpy(&z, &_buf[10], 4);
  return 0;
}

int Linkbot::getBatteryVoltage(float &volts)
{
  packSimpleCmd(BTCMD(CMD_GETBATTERYVOLTAGE));
  transactMessage();
  memcpy(&volts, &_buf[2], 4);
  return 0;
}

int Linkbot::getColorRGB(uint8_t &r, uint8_t &g, uint8_t &b)
{
  return 0;
}

int Linkbot::getFormFactor(int &form)
{
  return 0;
}

int Linkbot::getJointAngle(int joint, float &angle)
{
  return 0;
}

int Linkbot::getJointAngles(float &angle1, float &angle2, float &angle3)
{
  return 0;
}

int Linkbot::isMoving()
{
  return 0;
}

int Linkbot::moveJoint(int joint, float angle)
{
  return 0;
}

int Linkbot::moveJointNB(int joint, float angle)
{
  return 0;
}

int Linkbot::moveJointTo(int joint, float angle)
{
  return 0;
}

int Linkbot::moveJointToNB(int joint, float angle)
{
  return 0;
}

int Linkbot::move(float angle1, float angle2, float angle3)
{
  return 0;
}

int Linkbot::moveNB(float angle1, float angle2, float angle3)
{
  return 0;
}

int Linkbot::moveTo(float angle1, float angle2, float angle3)
{
  return 0;
}

int Linkbot::moveToNB(float angle1, float angle2, float angle3)
{
  return 0;
}

int Linkbot::moveWait()
{
  return 0;
}

int Linkbot::reset()
{
  return 0;
}

int Linkbot::resetToZero()
{
  return 0;
}

int Linkbot::setJointSpeed(int joint, float speed)
{
  return 0;
}

int Linkbot::setJointSpeeds(float speed1, float speed2, float speed3)
{
  return 0;
}

int Linkbot::setJointState(int joint, int state)
{
  return 0;
}

int Linkbot::setJointStates(int state1, int state2, int state3)
{
  return 0;
}

int Linkbot::setLEDColor(uint8_t r, uint8_t g, uint8_t b)
{
  return 0;
}

int Linkbot::setMotorPower(int joint, int power)
{
  return 0;
}

int Linkbot::setMotorPowers(int power1, int power2, int power3)
{
  return 0;
}

int Linkbot::stop()
{
  return 0;
}

void Linkbot::packBufReset()
{
  _bufsize = 0;
}

void Linkbot::packBufByte(uint8_t byte)
{
  _buf[_bufsize] = byte;
  _bufsize++;
}

void Linkbot::packBuf(void* data, int size)
{
  memcpy(&_buf[_bufsize], data, size);
  _bufsize += size;
}

void Linkbot::packSimpleCmd(uint8_t cmd)
{
  _buf[0] = cmd;
  _buf[1] = 3;
  _buf[2] = 0x00;
  _bufsize = 3;
}

int Linkbot::transactMessage()
{
  unsigned long startMillis;
  static uint8_t buf[256];
  /* First, compose the Link-Layer message */
  _buf[1] = _bufsize;
  buf[0] = _buf[0];
  buf[1] = _buf[1] + 6;
  buf[2] = _zigbee_addr >> 8;
  buf[3] = _zigbee_addr & 0x00ff;
  buf[4] = 1;
  memcpy(&buf[5], _buf, _bufsize);
  buf[5+_bufsize] = 0x00;
  /* Wait for ready state */
  while(TWI_READY != twi_state) {
    asm("nop");
  }
  g_recvBytes = 0;
  startMillis = millis();
  twi_writeTo(0x01, buf, _bufsize+6, 1, 1);
  /* Wait for a response or a timeout */
  while(1) {
    if(g_recvBytes != 0) {
      break;
    }
    if((millis() - startMillis) > 500) {
      return -1;
    }
  }
  return 0;
}
