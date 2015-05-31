#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdint.h>
#include <stddef.h>

#include "config.h"
#include "frame.h"

class Device;
class Receiver;
class Exception;
class UAVNav;


/*
 * Abstract class:
 * Basic functions for all copter-frame types and
 * some pure virtual functions which are specific for certain frames
 */
class AbsAttitude {
private:
  uint_fast32_t m_iAttHTimer;

  // Read from receiver and copy into floats above
  void read_receiver();

protected:
  // Current roll, pitch, throttle and yaw readouts from the receiver module
  float m_fRCRol;
  float m_fRCPit;
  float m_fRCYaw;
  float m_fRCThr;

  // Device module pointers for high level hardware access
  Device*    m_pHalBoard;
  Receiver*  m_pReceiver;
  Exception* m_pExeption;
  UAVNav*    m_pNavigation;

  // Function must be overloaded for basic (and frame dependent) flight control
  virtual void attitude_hold() = 0;  // Calculate here, how the copter can hold attitude automatically
  virtual void altitude_hold() = 0;  // Calculate here, how the copter can hold altitude automatically

  virtual void auto_navigate() = 0;  // Here the basic Auto-GPS navigation should be implemented

  virtual void accel_xy_stab() = 0;  // Correct small deviation along the z axis using the accelerometer
  virtual void accel_z_stab()  = 0;  // Correct small deviations along the x or y axis using the accelerometer

public:
  AbsAttitude(Device *, Receiver *, Exception *, UAVNav *);

  /*
   * Updates all the necessary sensors, reads from receiver
   * and handles exceptions.
   * Function calls:
   * - read_receiver()
   * - calc_attitude_hold()
   * - calc_altitude_hold()
   * - accel_xy_stab()
   * - accel_z_stab()
   * - auto_navigate()
   */
  virtual void run();
};

/*
 * Implementation of an quad-copter with X-configuration
 */
class MultiCopter : public AbsAttitude, Frame4X {
private:
  uint_fast32_t m_iAltHTimer; // Altitude hold timer using barometer/sonar for GPS navigation and altitude hold mode (50 Hz)
  uint_fast32_t m_iAccZTimer; // Altitude hold timer using the accelerometer for every flight mode (if necessary). Aim: Compensation of fast g-changes along the z-axis.

private:
  // Motor compensation terms (if model is tilted or battery voltage drops)
  float m_fBattComp;
  float m_fTiltComp;

  // Calculate and apply the motor compensation terms
  void apply_motor_compens();                                       // This functions applies motor compensation terms (e.g. battery and tilt) to the output of the servos

protected:
  void attitude_hold();
  void altitude_hold();

  void accel_xy_stab();
  void accel_z_stab();

  void auto_navigate();

public:
  MultiCopter(Device *, Receiver *, Exception *, UAVNav *);

  void calc_batt_comp();                                            // battery voltage drop compensation
  void calc_tilt_comp();                                            // motor compensation if model is tilted
  
  /*
   * Updates all the necessary sensors, reads from receiver
   * and handles exceptions.
   * Function calls:
   * - AbsAttitude::run()
   * - servo_out()
   */
  void run();
};

#endif