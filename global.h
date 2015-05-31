#ifndef GLOB_h
#define GLOB_h

#include "config.h"
#include "scheduler.h"
#include "device.h"
#include "receiver.h"
#include "exceptions.h"
#include "vehicle.h"
#include "navigation.h"
#include "parameters.h"


///////////////////////////////////////////////////////////
// ArduPilot Hardware Abstraction Layer
///////////////////////////////////////////////////////////
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

///////////////////////////////////////////////////////////
// EEPROM: Sets up the parameter table, and sets the default values.
// Must be the first AP_Param variable declared to ensure its
// constructor runs before the constructors of the other AP_Param variables
///////////////////////////////////////////////////////////
AP_Param                       param_loader(var_info);

// serial manager for GPS serial communication
AP_SerialManager               _SER_MANAGER;

///////////////////////////////////////////////////////////
// Container for all the PIDs of the copter
// PID indices are defined in "config.h"
// see: AC_PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt)
///////////////////////////////////////////////////////////
AC_PID                         _PIDS[NR_OF_PIDS] = { AC_PID(0.65, 0.35, 0.015, 50,  AC_PID_FILT_HZ_DEFAULT, 0.01),    // _PIT_RATE
                                                     AC_PID(0.65, 0.35, 0.015, 50,  AC_PID_FILT_HZ_DEFAULT, 0.01),    // _ROL_RATE
                                                     AC_PID(0.75, 0.50, 0.f,   50,  AC_PID_FILT_HZ_DEFAULT, 0.01),    // _YAW_RATE
                                                     AC_PID(0.25, 0.50, 0.f,   100, AC_PID_FILT_HZ_DEFAULT, 0.01),    // _THR_RATE
                                                     AC_PID(0.50, 0.10, 0.f,   100, AC_PID_FILT_HZ_DEFAULT, 0.01),    // _ACC_RATE
                                                     AC_PID(4.25, 0.f,  0.f,   0,   AC_PID_FILT_HZ_DEFAULT, 0.01),    // _PIT_STAB
                                                     AC_PID(4.25, 0.f,  0.f,   0,   AC_PID_FILT_HZ_DEFAULT, 0.01),    // _ROL_STAB
                                                     AC_PID(4.25, 0.f,  0.f,   0,   AC_PID_FILT_HZ_DEFAULT, 0.01),    // _YAW_STAB
                                                     AC_PID(5.50, 0.f,  0.f,   0,   AC_PID_FILT_HZ_DEFAULT, 0.01),    // _THR_STAB
                                                     AC_PID(4.25, 0.f,  0.f,   0,   AC_PID_FILT_HZ_DEFAULT, 0.01) };  // _ACC_STAB

///////////////////////////////////////////////////////////
// Board specific sensors
///////////////////////////////////////////////////////////
Compass                        _COMP;
AP_Baro                        _BARO;
AP_InertialSensor              _INERT;
AP_BattMonitor                 _BAT;                                                              // battery monitor
RangeFinder                    _SON_RF;
AP_GPS                         _GPS;                                                              // GPS
#if AP_AHRS_NavEKF                                                                                // Excluded: APM1, APM2
AP_AHRS_NavEKF                 _AHRS        (_INERT, _BARO, _GPS);
#else
AP_AHRS_DCM                    _AHRS        (_INERT, _BARO, _GPS);
#endif

///////////////////////////////////////////////////////////
// Abstracted hardware abstraction classes :D
// Take any sensor if derived from ArduPilot library!
///////////////////////////////////////////////////////////
Scheduler                      _SCHED_NAV   (&hal); // Scheduler for navigation system
Scheduler                      _SCHED_OUT   (&hal); // Scheduler for network output

Device                         _HAL_BOARD   (&hal, &_SER_MANAGER, &_INERT, &_COMP, &_BARO, &_GPS, &_BAT, &_SON_RF, &_AHRS, &_PIDS[0]);
Receiver                       _RECVR       (&_HAL_BOARD, &_SCHED_OUT);
Exception                      _EXCP        (&_HAL_BOARD, &_RECVR);

// Currently just a quad-copter with X-frame is implemented
UAVNav                         _UAV         (&_HAL_BOARD, &_RECVR, &_EXCP);
MultiCopter                    _MODEL       (&_HAL_BOARD, &_RECVR, &_EXCP, &_UAV);

///////////////////////////////////////////////////////////
// EEPROM
// var_info table
///////////////////////////////////////////////////////////
const AP_Param::Info var_info[] PROGMEM = {
  GSCALAR(format_version, "SYSID_SW_MREV", 0),

  // PIDs
  GOBJECTN(_PIDS[PID_PIT_RATE], PIT_RATE, "PID_PIT_RATE", AC_PID),
  GOBJECTN(_PIDS[PID_ROL_RATE], ROL_RATE, "PID_ROL_RATE", AC_PID),
  GOBJECTN(_PIDS[PID_YAW_RATE], YAW_RATE, "PID_YAW_RATE", AC_PID),
  GOBJECTN(_PIDS[PID_THR_RATE], THR_RATE, "PID_THR_RATE", AC_PID),
  GOBJECTN(_PIDS[PID_ACC_RATE], ACC_RATE, "PID_ACC_RATE", AC_PID),
  GOBJECTN(_PIDS[PID_PIT_STAB], PIT_STAB, "PID_PIT_STAB", AC_PID),
  GOBJECTN(_PIDS[PID_ROL_STAB], ROL_STAB, "PID_ROL_STAB", AC_PID),
  GOBJECTN(_PIDS[PID_YAW_STAB], YAW_STAB, "PID_YAW_STAB", AC_PID),
  GOBJECTN(_PIDS[PID_ACC_STAB], ACC_STAB, "PID_ACC_STAB", AC_PID),
  GOBJECTN(_PIDS[PID_THR_STAB], THR_STAB, "PID_THR_STAB", AC_PID),

  // Device objects
  GOBJECT(_COMP,        "COMPASS_",   Compass),
  GOBJECT(_INERT,       "INS_",       AP_InertialSensor),
  GOBJECT(_AHRS,        "AHRS_",      AP_AHRS),
  GOBJECT(_BAT,         "BATT_",      AP_BattMonitor),
  GOBJECT(_BARO,        "GND_",       AP_Baro),
  GOBJECT(_GPS,         "GPS_",       AP_GPS),
  GOBJECT(_SON_RF,      "RNGFND_",     RangeFinder),

  AP_VAREND
};

#endif

