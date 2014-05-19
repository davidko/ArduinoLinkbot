#ifndef _LINKBOT_H_
#define _LINKBOT_H_

/* Web page at http://davidko.github.io/ArduinoLinkbot/ */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/**
 * Possible robot joint states
 * These values represent the possible robot joint states. */
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


/** 
 * The Linkbot Class. 
 * Each instance of a Linkbot class represents a physical Linkbot. The physical
 * Linkbots may be the directly connected Linkbot or remote Linkbots.
 *
 * Blocking and Non-Blocking Functions
 * ===================================
 * 
  The member functions of the Linkbot class which are responsible for moving
  the joints of the Linkbot can be categorized into two general types of
  functions; "blocking" functions and "non-blocking" functions. A blocking
  function is a function that "hangs" until the complete motion is done,
  whereas a "non-blocking" function returns as soon as the motion begins,
  but does not wait until it is done. In the Linkbot class, all functions
  are blocking unless the have the suffix "NB", such as "Linkbot.moveNB()".

  For example, consider the following lines of code::

      linkbot.move(360, 0, 0);
      linkbot.setBuzzerFrequency(440);

  When the above lines of code are executed, the Linkbot will rotate joint 1
  360 degrees. Once the joint has rotated the full revolution, the buzzer will
  sound. Now consider the following code::

      linkbot.moveNB(360, 0, 0);
      linkbot.setBuzzerFrequency(440);

  For these lines of code, joint 1 also moves 360 degrees. The difference is 
  that with these lines of code, the buzzer will begin emitting a tone as soon
  as the joint begins turning, instead of waiting for the motion to finish.
  This is because the non-blocking version of move() was used, which returns
  immediately after the joint begins moving allowing the setBuzzerFrequency() 
  function to execute as the joint begins moving.

  The moveWait() function can be used to
  block until non-blocking motion functions are finished. For instance, the
  following two blocks of code will accomplish the same task::

      linkbot.move(360, 0, 0);
      linkbot.setBuzzerFrequency(440);

      linkbot.moveNB(360, 0, 0);
      linkbot.moveWait();
      linkbot.setBuzzerFrequency(440);
 */
class Linkbot {
  public:
    /** Constructor
     * The constructor can take a single argument. If the argument is a 2-byte
     * non-zero value, it refers to the Zigbee address of a remote Linkbot. If
     * it is zero, it refers to the local Linkbot. */
    Linkbot(uint16_t zigbee_addr = 0);
    ~Linkbot();

    /**
     * Check to see if the Linkbot is responding. Returns 0 on success.
     */
    int checkStatus();

    /**
     * Drive a joint to a certain position using the on-board PID controller.
     * @param joint an integer; the joint to move
     * @param angle the angle to move the joint to in degrees
     */
    int driveJointTo(int joint, float angle);
    int driveJointToNB(int joint, float angle);

    /**
     * Drive all of the joints to specified angles in degrees using the
     * on-board PID controller.
     */
    int driveTo(float angle1, float angle2, float angle3);
    int driveToNB(float angle1, float angle2, float angle3);

    /** 
     * Get the current accelerometer data values. Values of argument variables
     * x, y, and z will be overwritten with values. 
     */
    int getAccelerometerData(float &x, float &y, float &z);

    /** Get the current battery voltage.
     * @param volts the value of this variable will be overwritten with the
     * current battery voltage. */
    int getBatteryVoltage(float &volts);

    /** 
     * The the current RGB LED color values.
     * The parameters r, g, and b will be overwritten with the current rgb values. 
     */
    int getColorRGB(uint8_t &r, uint8_t &g, uint8_t &b);

    int getFormFactor(int &form);

    /**
     * Get the current joint angle of a joint in degrees
     */
    int getJointAngle(int joint, float &angle);

    /**
     * Get the current joint angles of all of the joints in degrees. Values of
     * parameters will be overwritten with joint angle values. */
    int getJointAngles(float &angle1, float &angle2, float &angle3);

    /**
     * Check to see if any of the joints are still moving. Returns 1 if moving,
     * 0 if not moving, or -1 on command failure.  */
    int isMoving();

    /**
     * Move a joint from its current position by some angle in degrees at a
     * constant speed.
     * @param joint the joint to move
     * @param angle the amount of degrees to move the joint. Negative values
     * move the joint backwards.
     */
    int moveJoint(int joint, float angle);
    int moveJointNB(int joint, float angle);

    /**
     * Move a joint to a certain angle in degrees at a constant speed.
     */
    int moveJointTo(int joint, float angle);
    int moveJointToNB(int joint, float angle);

    /**
     * Move all of the joints by a relative number of degrees.
     */
    int move(float angle1, float angle2, float angle3);
    int moveNB(float angle1, float angle2, float angle3);

    /**
     * Move all of the joints to specified angles in degrees
     */
    int moveTo(float angle1, float angle2, float angle3);
    int moveToNB(float angle1, float angle2, float angle3);

    /**
     * Wait for a non-blocking joint motion to finish.
     * This function will block until the robot has completed all of its
     * current motion commands. 
     */
    int moveWait();

    /**
     * Reset multi-rotational joint angle counters on the robot.
     * This function is used to reset the multi-rotational angle counters on
     * the robot. For instance, if a robot's joint is currently at 370 degrees
     * (1 full rotation plus 10 degrees), calling this function will reset the
     * joint angle reading instantly to 10 degrees.
     */
    int reset();
    /**
     * Reset joint rotation counters and move to zero.
     */
    int resetToZero();

    /** Set a joint's speed in degrees/second */
    int setJointSpeed(int joint, float speed);
    int setJointSpeeds(float speed1, float speed2, float speed3);
    /** Set a joint's current state. The state can be any of robotJointState_e */
    int setJointState(int joint, int state);
    int setJointStates(int state1, int state2, int state3, float speed1, float speed2, float speed3);

    /**Set the LED's current color by specifying red, green, and blue values.
     * Each value can range from 0 to 255. */
    int setLEDColor(uint8_t r, uint8_t g, uint8_t b);

    /** Set a motor's power. Power values can be from -255 to 255. */
    int setMotorPower(int joint, int power);
    /** Set the motor power for all of the joints on the robot. */
    int setMotorPowers(int power1, int power2, int power3);

    /** Stop all motors on the robot. */
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
