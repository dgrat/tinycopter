#ifndef DEVICE_h
#define DEVICE_h

#include <stdint.h>
#include <stddef.h>

#include <AP_Math.h>
#include <AC_PID.h>

#include <Filter.h>

#include "containers.h"
#include "absdevice.h"
#include "config.h"

class AP_InertialSensor;
class AP_InertialNav;
class AP_AHRS;
class AP_BattMonitor;
class AP_Baro;
class AP_GPS;
class AP_SerialManager;

class BaroData;
class BattData;
class Compass;
class GPSData;
class LowPassFilterVector3f;
class RangeFinder;


///////////////////////////////////////////////////////////
// This class purpose is to init all the device drivers
// It acts also as a container for the (initialized) device function pointers
///////////////////////////////////////////////////////////
class DeviceInit : public AbsErrorDevice {
protected:
  uint_fast32_t m_t32Inertial;      // For calculating the derivative of the angular changes
  uint_fast32_t m_t32Compass;

  AC_PID *m_pPIDs;

public /*objects*/:
  // Hardware abstraction library interface
  const AP_HAL::HAL *m_pHAL;
  // MPU6050 accel/gyro chip
  AP_InertialSensor *m_pInert;
  // Magnetometer aka compass
  Compass           *m_pComp;
  // Barometer
  AP_Baro           *m_pBaro;
  // GPS
  AP_GPS            *m_pGPS;
  // battery monitor
  AP_BattMonitor    *m_pBat;
  // Sonar
  RangeFinder       *m_pRF;
  // Attitude heading reference system
  AP_AHRS           *m_pAHRS;
  // Serial manager
  AP_SerialManager  *m_pSerMan;

  static const struct AP_Param::GroupInfo var_info[];

public /*functions*/:
  DeviceInit(const AP_HAL::HAL *, AP_SerialManager *, AP_InertialSensor *, Compass *, AP_Baro *, AP_GPS *, AP_BattMonitor *, RangeFinder *, AP_AHRS *, AC_PID *);

  void         init_barometer();
  void         init_compass();
  void         init_inertial();
  void         init_gps();
  void         init_batterymon();
  void         init_rf();
  void         init_inertial_nav();
};

///////////////////////////////////////////////////////////
// Container for sensor data and sensor configuration
///////////////////////////////////////////////////////////
class Device : public DeviceInit {
private /*variables*/:
  // Used with low path filter
  LowPassFilterVector3f m_AccelLPF;
  Vector3f     m_vAccelPG_cmss;         // acceleration readout
  Vector3f     m_vAccelMG_cmss;         // acceleration readout minus G constant (~9.81)

  // User set correction variables
  float        m_fInertPitCor;          // +/-: left to right or right to left
  float        m_fInertRolCor;          // +/-: front to back or back to front

  float        m_fCmpH;                 // Compass heading
  float        m_fGpsH;                 // GPS heading

private /*functions*/:
  void         calc_acceleration(); // Filters (LPF) and calculates the current acceleration with and without 'g'

protected /*variables*/:
  // x = pitch, y = roll, z = yaw
  Vector3f     m_vGyro_deg;
  Vector3f     m_vAtti_deg;
  int_fast16_t m_iAltitude_cm;
  // misc
  BaroData     m_ContBaro;
  GPSData      m_ContGPS;
  BattData     m_ContBat;

public:
  // Accepts pointers to abstract base classes to handle different sensor types
  Device(const AP_HAL::HAL *, AP_SerialManager *, AP_InertialSensor *, Compass *, AP_Baro *, AP_GPS *, AP_BattMonitor *, RangeFinder *, AP_AHRS *, AC_PID *);

  // Setter and getter for inertial adjustments
  void         set_trims(float fRoll_deg, float fPitch_deg);

  void         update_inav();        // Update inertial navigation (accelerometer, barometer, GPS sensor fusion)
  void         update_attitude();

  // Handling of the PIDs
  AC_PID&      get_pid(uint_fast8_t);
  float        get_pid_val(uint_fast8_t);
  void         set_pid(uint_fast8_t, const AC_PID &);
  void         set_pids_dt(const float &dt);
  void         load_pids();
  void         save_pids();
  
  // Updating the sensors
  BaroData     read_baro();
  float        read_comp_deg();
  GPSData      read_gps();
  BattData     read_bat();
  int_fast32_t read_rf_cm();         // Altitude estimate using range finder

  // Return the Vector3f Inertial readouts
  Vector3f     get_atti_cor_deg();   // fused sensor values from accelerometer/gyrometer with m_fInertPitCor/m_fInertRolCor
  Vector3f     get_atti_raw_deg();   // fused sensor values from accelerometer/gyrometer without m_fInertPitCor/m_fInertRolCor

  Vector3f     get_gyro_degps();     // gyrometer sensor readout in degree per second

  Vector3f     get_accel_cor_deg();  // accelerometer sensor readout with m_fInertPitCor/m_fInertRolCor
  Vector3f     get_accel_raw_deg();  // accelerometer sensor readout without m_fInertPitCor/m_fInertRolCor
  
  Vector3f     get_accel_pg_cmss();  // Acceleration with the G-const
  Vector3f     get_accel_mg_cmss();  // Acceleration without the G-const (filtered out)
  
  Vector3f     get_accel_pg_g();     // Acceleration without the G-const (filtered out)
  Vector3f     get_accel_mg_g();     // Acceleration without the G-const (filtered out)
  
  float        get_comp_deg();       // Just return the last estimated compass
  BaroData     get_baro();           // Just return the filtered barometer data
  GPSData      get_gps();            // Just return the last estimated gps data
  BattData     get_bat();            // Just return the last estimated battery data
  int_fast32_t get_rf_cm();          // Altitude estimate using range finder
};

#endif
