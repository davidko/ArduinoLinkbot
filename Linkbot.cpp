
#include <Arduino.h>

extern "C" {
#include "Linkbot.h"
#include "utility/commands.h"
#include "utility/twi.h"
#include <math.h>

void dprint(const char* buf) {
    Serial.write(buf);
}
} // extern "C"


static void rmemcpy(void *dest, const void *src, size_t n) {
    const uint8_t* _src = (const uint8_t*)src;
    uint8_t* _dest = (uint8_t*) dest;
    for(int i = 0; i < n; i++) {
        _dest[n-1-i] = _src[i];
    }
}

volatile uint8_t g_recvBytes = 0;
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
  memcpy(&x, &g_recvBuf[7], 4);
  memcpy(&y, &g_recvBuf[11], 4);
  memcpy(&z, &g_recvBuf[15], 4);
  return 0;
}

int Linkbot::getBatteryVoltage(float &volts)
{
  packSimpleCmd(BTCMD(CMD_GETBATTERYVOLTAGE));
  transactMessage();
  memcpy(&volts, &g_recvBuf[7], 4);
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
  float angle1, angle2, angle3;
  float angles[3];
  getJointAngles(angle1, angle2, angle3);
  angles[0] = angle1;
  angles[1] = angle2;
  angles[2] = angle3;
  angle = angles[joint-1];
  return 0;
}

int Linkbot::getJointAngles(float &angle1, float &angle2, float &angle3)
{
  packSimpleCmd(BTCMD(CMD_GETMOTORANGLESABS));
  transactMessage();
  memcpy(&angle1, &g_recvBuf[7], 4);
  memcpy(&angle2, &g_recvBuf[11], 4);
  memcpy(&angle3, &g_recvBuf[15], 4);
  char buf[32];
  sprintf(buf, "m1: %d m2: %f, m3: %f, 0x%x%x%x%x\n", (int)(angle1*1000), angle2, angle3,
          g_recvBuf[7],
          g_recvBuf[8],
          g_recvBuf[9],
          g_recvBuf[10]
          );
  Serial.write(buf);
  angle1 = RAD2DEG(angle1);
  angle2 = RAD2DEG(angle2);
  angle3 = RAD2DEG(angle3);
  sprintf(buf, "m1: %f\n", angle1);
  Serial.write(buf);
  return 0;
}

int Linkbot::isMoving()
{
  packSimpleCmd(BTCMD(CMD_IS_MOVING));
  transactMessage();
  return g_recvBuf[7];
}

int Linkbot::moveJoint(int joint, float angle)
{
  float _angle;
  getJointAngle(joint, _angle);
  moveJointToNB(joint, angle+_angle);
  moveWait();
  return 0;
}

int Linkbot::moveJointNB(int joint, float angle)
{
  float _angle;
  getJointAngle(joint, _angle);
  moveJointToNB(joint, angle+_angle);
  return 0;
}

int Linkbot::moveJointTo(int joint, float angle)
{
  moveJointToNB(joint, angle);
  moveWait();
  return 0;
}

int Linkbot::moveJointToNB(int joint, float angle)
{
  angle = DEG2RAD(angle);
  packBufReset();
  packBufByte(BTCMD(CMD_SETMOTORANGLEABS));
  packBufByte(0x00);
  packBufByte(joint);
  packBuf(&angle, 4);
  packBufByte(0x00);
  return transactMessage();
}

int Linkbot::move(float angle1, float angle2, float angle3)
{
  moveNB(angle1, angle2, angle3);
  moveWait();
  return 0;
}

int Linkbot::moveNB(float angle1, float angle2, float angle3)
{
  float a1, a2, a3;
  getJointAngles(a1, a2, a3);
  moveToNB(angle1+a1, angle2+a2, angle3+a3);
  return 0;
}

int Linkbot::moveTo(float angle1, float angle2, float angle3)
{
  moveToNB(angle1, angle2, angle3);
  moveWait();
  return 0;
}

int Linkbot::moveToNB(float angle1, float angle2, float angle3)
{
  angle1 = DEG2RAD(angle1);
  angle2 = DEG2RAD(angle2);
  angle3 = DEG2RAD(angle3);
  packBufReset();
  packBufByte(BTCMD(CMD_SETMOTORANGLESABS));
  packBufByte(0x00);
  packBuf(&angle1, 4);
  packBuf(&angle2, 4);
  packBuf(&angle3, 4);
  packBuf(&angle3, 4);
  packBufByte(0x00);
  return transactMessage();
}

int Linkbot::moveWait()
{
  while(isMoving()) {
    delay(100);
  }
  return 0;
}

int Linkbot::reset()
{
  packSimpleCmd(BTCMD(CMD_RESETABSCOUNTER));
  return transactMessage();
}

int Linkbot::resetToZero()
{
  reset();
  moveTo(0, 0, 0);
  return 0;
}

int Linkbot::setJointSpeed(int joint, float speed)
{
  speed = DEG2RAD(speed);
  packBufReset();
  packBufByte(BTCMD(CMD_SETMOTORSPEED));
  packBufByte(0x00);
  packBufByte(joint);
  packBuf(&speed, 4);
  packBufByte(0x00);
  return transactMessage();
}

int Linkbot::setJointSpeeds(float speed1, float speed2, float speed3)
{
  setJointSpeed(1, speed1);
  setJointSpeed(2, speed2);
  setJointSpeed(3, speed3);
  return 0;
}

int Linkbot::setJointState(int joint, int state)
{
  packBufReset();
  packBufByte(BTCMD(CMD_SETMOTORDIR));
  packBufByte(0x00);
  packBufByte(joint);
  packBufByte(state);
  packBufByte(0x00);
  return transactMessage();
}

int Linkbot::setJointStates(int state1, int state2, int state3, float speed1, float speed2, float speed3)
{
  packBufReset();
  packBufByte(BTCMD(CMD_SETMOTORSTATES));
  packBufByte(0x00);
  packBufByte(state1);
  packBufByte(state2);
  packBufByte(state3);
  packBufByte(0);
  packBuf(&speed1, 4);
  packBuf(&speed2, 4);
  packBuf(&speed3, 4);
  packBuf(&speed1, 4);
  packBufByte(0x00);
  return transactMessage();
}

int Linkbot::setLEDColor(uint8_t r, uint8_t g, uint8_t b)
{
  packBufReset();
  packBufByte(BTCMD(CMD_RGBLED));
  packBufByte(0xff);
  packBufByte(0xff);
  packBufByte(0xff);
  packBufByte(r);
  packBufByte(g);
  packBufByte(b);
  packBufByte(0x00);
  return transactMessage();
}

int Linkbot::setMotorPower(int joint, int power)
{
  uint8_t mask;
  int16_t _power = power;
  int i;
  mask = 1<<joint;
  packBufReset();
  packBufByte(BTCMD(CMD_SETMOTORPOWER));
  packBufByte(0x00);
  packBufByte(mask);
  for(i = 0; i < 3; i++) {
    packBufByte(_power>>8);
    packBufByte(_power&0x00ff);
  }
  packBufByte(0x00);
  return transactMessage();
}

int Linkbot::setMotorPowers(int power1, int power2, int power3)
{
  int16_t powers[3];
  int i;
  powers[0] = power1;
  powers[1] = power2;
  powers[2] = power3;
  packBufReset();
  packBufByte(BTCMD(CMD_SETMOTORPOWER));
  packBufByte(0x00);
  packBufByte(0x07);
  for(i = 0; i < 3; i++) {
    packBufByte(powers[i]>>8);
    packBufByte(powers[i]&0x00ff);
  }
  packBufByte(0x00);
  return transactMessage();
}

int Linkbot::stop()
{
  packSimpleCmd(BTCMD(CMD_STOP));
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
  g_recvBytes = 0;
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
  startMillis = millis();
  Serial.write("!\n");
  twi_writeTo(0x01, buf, _bufsize+6, 1, 1);
  Serial.write(".\n");
  /* Wait for a response or a timeout */
  char sbuf[32];
  while(1) {
    if(g_recvBytes > 0) {
        return 0;
    }
    if((millis() - startMillis) > 500) {
      sprintf(sbuf, "timeout: %lu %lu\n", millis(), startMillis);
      Serial.write(sbuf);
      return -1;
    }
  }
  return 0;
}
