#include <AP_AHRS.h>
#include <AP_Compass.h>

#include "frame.h"
#include "vehicle.h"
#include "device.h"
#include "receiver.h"
#include "exceptions.h"
#include "navigation.h"
#include "arithmetics.h"


////////////////////////////////////////////////////////////////////////
// Abstract class implementation
////////////////////////////////////////////////////////////////////////
AbsAttitude::AbsAttitude(Device *pDev, Receiver *pRecv, Exception *pExcp, UAVNav* pUAV) {
  // Module pointers
  m_pHalBoard   = pDev;
  m_pReceiver   = pRecv;
  m_pExeption   = pExcp;
  m_pNavigation = pUAV;
  // Float initialization
  m_fRCRol      = 0.f;
  m_fRCPit      = 0.f;
  m_fRCYaw      = 0.f;
  m_fRCThr      = 0.f;

  m_iAttHTimer = pDev->m_pHAL->scheduler->millis();
}

void AbsAttitude::read_receiver() {
  m_fRCRol = static_cast<float>(m_pReceiver->get_channel(RC_ROL) );
  m_fRCPit = static_cast<float>(m_pReceiver->get_channel(RC_PIT) );
  m_fRCThr = static_cast<float>(m_pReceiver->get_channel(RC_THR) > RC_THR_80P ? RC_THR_80P : m_pReceiver->get_channel(2) );
  m_fRCYaw = static_cast<float>(m_pReceiver->get_channel(RC_YAW) );
}

void AbsAttitude::run() {
  // .. and update inertial information
  m_pHalBoard->update_attitude();

  // Handle all defined problems (time-outs, broken gyrometer, GPS signal ..)
  m_pExeption->handle();

  // Read from receiver module
  read_receiver();

  // Calculate dt for the PID regulator
  uint_fast32_t iCurTime_ms = m_pHalBoard->m_pHAL->scheduler->millis();
  float fDeltaTime_s = (float)(iCurTime_ms - m_iAttHTimer) / 1000.f;
  m_iAttHTimer = iCurTime_ms;
  m_pHalBoard->set_pids_dt(fDeltaTime_s);

  // Must get called before altitude hold calculation
  attitude_hold();
  accel_xy_stab();
  accel_z_stab();
  // Activate these features only if using advanced hardware
  #if AP_AHRS_NAVEKF_AVAILABLE
  altitude_hold();
  auto_navigate();
  #endif

  // Give the compass the throttle information ..
  // .. after the motor input was post-processed by the functions above
  if(m_pHalBoard->m_pComp->healthy() ) {
    float fPercThr = (float)(m_fRCThr - RC_THR_OFF) / (float)(RC_THR_80P - RC_THR_OFF);
    m_pHalBoard->m_pComp->set_throttle(fPercThr);
  }
}

////////////////////////////////////////////////////////////////////////
// Class implementation
////////////////////////////////////////////////////////////////////////
MultiCopter::MultiCopter(Device *pDev, Receiver *pRecv, Exception *pExcp, UAVNav* pUAV) 
  : AbsAttitude(pDev, pRecv, pExcp, pUAV), 
    Frame4X(pDev->m_pHAL->rcout)
{
  m_fBattComp = 0.f;
  m_fTiltComp = 0.f;

  m_iAccZTimer = m_iAltHTimer = pDev->m_pHAL->scheduler->millis();
}

void MultiCopter::calc_tilt_comp() {
  // For safety, always reset the correction term
  m_fTiltComp = 1.f;

  if( tiny::in_range(RC_ROL_MIN, RC_ROL_MAX, m_fRCRol) &&
      tiny::in_range(RC_PIT_MIN, RC_PIT_MAX, m_fRCPit) )
  {
    m_fTiltComp = 1.f / (cos(ToRad(m_fRCRol) ) * cos(ToRad(m_fRCPit) ) );
  }
}

void MultiCopter::calc_batt_comp() {
  // For safety, always reset the correction term
  m_fBattComp = 1.f;

  float fCurVoltage = m_pHalBoard->get_bat().voltage_V;
  if( m_pHalBoard->get_bat().refVoltage_V > 0.f &&
      tiny::in_range(BATT_MIN_VOLTAGE, BATT_MAX_VOLTAGE, fCurVoltage) )
  {
    m_fBattComp = m_pHalBoard->get_bat().refVoltage_V / fCurVoltage;
  }
}

void MultiCopter::apply_motor_compens() {
  calc_tilt_comp();
  //calc_batt_comp(); // Do it in the scheduler, because there the battery monitor is running

  float fCurThr = m_fRCThr - RC_THR_ACRO;
  // Calculate new throttle output (tilt and battery compensated)
  float fCompThr = fCurThr * (m_fTiltComp * m_fBattComp) + RC_THR_ACRO;
  m_fRCThr = fCompThr <= RC_THR_80P ? fCompThr : RC_THR_80P;
}

////////////////////////////////////////////////////////////////////////
// Implementations
// This function is only needed for autonomous flight mode with:
// * GPS auto-navigation
// * Overrides the remote control for controlling the quad
////////////////////////////////////////////////////////////////////////
void MultiCopter::auto_navigate() {
  // Break this function if there was not the proper UAV-mode set
  if(!tiny::chk_fset(m_pReceiver->get_waypoint()->mode, GPSPosition::GPS_NAVIGATN_F) ) {
    return;
  }
  // Break this function if the compass is not configured
  if(!m_pHalBoard->m_pComp->configured() ) {
    return;
  }

  // Set yaw in remote control
  m_pReceiver->set_channel(RC_YAW, m_pNavigation->calc_yaw() );
}

////////////////////////////////////////////////////////////////////////
// Implementations
// This function is only needed for (semi-)autonomous flight mode like:
// * Hold altitude
// * GPS auto-navigation
////////////////////////////////////////////////////////////////////////
void MultiCopter::altitude_hold() {
  // Do only change thrust if copter is armed!
  #if !BENCH_OUT
  if(m_fRCThr < RC_THR_ACRO) {
    return;
  }
  #endif
  int_fast16_t iAltZOutput = 0; // Barometer & Sonar

  // Calc current g-force
  Vector3f fAccel_g = m_pHalBoard->get_accel_mg_g();

  // Limit altitude control if in remote control mode
  bool bUseAltHold = m_pReceiver->get_waypoint()->mode == GPSPosition::NOTHING_F ? false : true;
  // Limit this loop to 50 Hz
  uint_fast32_t iHldAltTime_ms = m_pHalBoard->m_pHAL->scheduler->millis() - m_iAltHTimer;
  if(iHldAltTime_ms >= HLD_ALTITUDE_TIMER) {
    m_iAltHTimer = m_pHalBoard->m_pHAL->scheduler->millis();
  } else {
    bUseAltHold = false;
  }

  // Height regulation using the barometer and/or sonar
  // This function is necessary for navigation :D
  if(bUseAltHold) {
    float fTargAlti_cm  = static_cast<float>(m_pReceiver->get_waypoint()->altitude_cm);

    // Get the current position
    Location curPos;
    if(!m_pHalBoard->m_pAHRS->get_position(curPos) ) {
      return;
    }
    float fCurrAlti_cm = curPos.alt;

    // NED: North, East, Down
    Vector3f v3fCurSpeed_ms(0.f, 0.f, 0.f);
    if(!m_pHalBoard->m_pAHRS->get_velocity_NED(v3fCurSpeed_ms) ) {
      return;
    }
    float fClmbRate_cms = v3fCurSpeed_ms.z * 100.f;

    // Calculate the motor speed changes by the error from the height estimate and the current climb rates
    // If the quadro is going down, because of an device error, then this code is not used
    if(m_pReceiver->get_waypoint()->mode != GPSPosition::CONTRLD_DOWN_F) {
	  m_pHalBoard->get_pid(PID_THR_STAB).set_input_filter_all(fTargAlti_cm - fCurrAlti_cm);
      float fAltZStabOut = constrain_float(m_pHalBoard->get_pid_val(PID_THR_STAB), -250, 250);

	  m_pHalBoard->get_pid(PID_THR_RATE).set_input_filter_all(fAltZStabOut - fClmbRate_cms);
      iAltZOutput        = static_cast<int_fast16_t>(constrain_float(m_pHalBoard->get_pid_val(PID_THR_RATE), -500, 500) );
    }

    if(m_pReceiver->get_channel(RC_ROL) > RC_THR_OFF) {
      // If the quad-copter is going down too fast, fAcceleration_g becomes greater
      if(m_pReceiver->get_waypoint()->mode == GPSPosition::CONTRLD_DOWN_F && fAccel_g.z > HLD_ALTITUDE_ZGBIAS) {
        m_pExeption->pause_take_down();
      }

      // else: the fAcceleration_g becomes smaller
      if(m_pReceiver->get_waypoint()->mode == GPSPosition::CONTRLD_DOWN_F && fAccel_g.z <= HLD_ALTITUDE_ZGBIAS) {
        m_pExeption->cont_take_down();
      }
    }
  }

  // Modify the speed of the motors to hold the altitude
  add_thr_gain(iAltZOutput);
}

void MultiCopter::accel_xy_stab() {
}

void MultiCopter::accel_z_stab() {
  // Do only change thrust if copter is armed!
  #if !BENCH_OUT
  if(m_fRCThr < RC_THR_ACRO) {
    return;
  }
  #endif
  const float fScaleF_g2cmss = 100.f * INERT_G_CONST;
  int_fast16_t iAccZOutput = 0; // Accelerometer

  // Calc current g-force
  Vector3f fAccel_g = m_pHalBoard->get_accel_mg_g(); // Get the acceleration in g

  // Small & fast stabilization using the accelerometer
  static short iLAccSign = 0;
  if(fabs(fAccel_g.z) >= HLD_ALTITUDE_ZGBIAS) {
    if(iLAccSign == 0) {
      iLAccSign = tiny::sign_f(fAccel_g.z);
    }

    // The g-force must act for a minimum time interval before the PID can be used
    uint_fast32_t iAccZTime = m_pHalBoard->m_pHAL->scheduler->millis() - m_iAccZTimer;
    if(iAccZTime < HLD_ALTITUDE_ZTBIAS) {
       return;
    }

    // Check whether the direction of acceleration changed suddenly
    // If so: reset the timer
    short iCAccSign = tiny::sign_f(fAccel_g.z);
    if(iCAccSign != iLAccSign) {
      // Reset the switch if acceleration becomes normal again
      m_iAccZTimer = m_pHalBoard->m_pHAL->scheduler->millis();
      // Reset the PID integrator
      m_pHalBoard->get_pid(PID_ACC_RATE).reset_I();
      // Save last sign
      iLAccSign = iCAccSign;
      return;
    }

    // Feed the current acceleration into the PID regulator
    float fAccZ_cmss = tiny::sign_f(fAccel_g.z) * (fabs(fAccel_g.z) - HLD_ALTITUDE_ZGBIAS) * fScaleF_g2cmss;
    m_pHalBoard->get_pid(PID_ACC_RATE).set_input_filter_all(-fAccZ_cmss);
    iAccZOutput = static_cast<int_fast16_t>(constrain_float(m_pHalBoard->get_pid_val(PID_ACC_RATE), -500, 500) );

    #if DEBUG_OUT
    m_pHalBoard->m_pHAL->console->printf("altitude_hold() - iAccZOutput: %d\n", iAccZOutput);
    #endif
  } else {
    // Reset the switch if acceleration becomes normal again
    m_iAccZTimer = m_pHalBoard->m_pHAL->scheduler->millis();
    // Reset the PID integrator
    m_pHalBoard->get_pid(PID_ACC_RATE).reset_I();
  }

  // Modify the speed of the motors to hold the altitude
  add_thr_gain(iAccZOutput);
}

/*
 * Fast and time critical loop for:
 * - controlling the quadrocopter
 * - fetching RC signals
 * - filtering and processing sensor data necessary for flight
 */
void MultiCopter::attitude_hold() {
  // additional filter or rc variables
  static float targ_yaw = 0.f; // Yaw target from RC

  Vector3f vAtti = m_pHalBoard->get_atti_cor_deg(); // returns the fused sensor value (gyrometer and accelerometer)
  Vector3f vGyro = m_pHalBoard->get_gyro_degps();   // returns the sensor value from the gyrometer

#if !BENCH_OUT
  // Throttle raised, turn on stabilisation.
  if(m_fRCThr >= RC_THR_ACRO) {
#endif

    // Stabilise PIDS
    m_pHalBoard->get_pid(PID_PIT_STAB).set_input_filter_all(m_fRCPit - vAtti.x);
    m_pHalBoard->get_pid(PID_ROL_STAB).set_input_filter_all(m_fRCRol - vAtti.y);
    m_pHalBoard->get_pid(PID_YAW_STAB).set_input_filter_all(tiny::wrap180_f(targ_yaw - vAtti.z) );

    float pit_stab_output = constrain_float(m_pHalBoard->get_pid_val(PID_PIT_STAB), -250, 250);
    float rol_stab_output = constrain_float(m_pHalBoard->get_pid_val(PID_ROL_STAB), -250, 250);
    float yaw_stab_output = constrain_float(m_pHalBoard->get_pid_val(PID_YAW_STAB), -360, 360);

    // Is pilot asking for yaw change? - If so, feed directly to rate PID (overwriting yaw stab output)
    if(fabs(m_fRCYaw ) > 5.f) {
      yaw_stab_output = m_fRCYaw;
      targ_yaw = vAtti.z; // remember this yaw for when pilot stops
    }

    // Rate PIDS
    m_pHalBoard->get_pid(PID_PIT_RATE).set_input_filter_all(pit_stab_output - vGyro.x);
    m_pHalBoard->get_pid(PID_ROL_RATE).set_input_filter_all(rol_stab_output - vGyro.y);
    m_pHalBoard->get_pid(PID_YAW_RATE).set_input_filter_all(yaw_stab_output - vGyro.z);

    set_pit_gain(static_cast<int_fast16_t>(constrain_float(m_pHalBoard->get_pid_val(PID_PIT_RATE), -500, 500) ) );
    set_rol_gain(static_cast<int_fast16_t>(constrain_float(m_pHalBoard->get_pid_val(PID_ROL_RATE), -500, 500) ) );
    set_yaw_gain(static_cast<int_fast16_t>(constrain_float(m_pHalBoard->get_pid_val(PID_YAW_RATE), -500, 500) ) );

    // Apply: tilt- and battery-compensation algorithms
    apply_motor_compens();  // calc new m_fRCThr
    set_thr_gain(m_fRCThr); // apply new m_fRCThr

#if !BENCH_OUT
  } else {
    // Clear motor output
    set_motor_gains(0, 0, 0, RC_THR_OFF);
    // reset yaw target so we maintain this on take-off
    targ_yaw = vAtti.z;
    // reset PID integrals whilst on the ground
    for(uint_fast8_t i = 0; i < NR_OF_PIDS; i++) {
      m_pHalBoard->get_pid(i).reset_I();
    }
  }
#endif
}

void MultiCopter::run() {
  // Do the basic attitude calculations
  AbsAttitude::run();
  // Calculate the motor output and send it to the servos
  Frame4X::calc_servo_out();
  Frame4X::servo_out();
}
