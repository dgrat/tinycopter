#include <float.h>

#include <AP_AHRS.h>
#include <AP_GPS.h>
#include <AP_Baro.h>

#include "exceptions.h"
#include "receiver.h"
#include "device.h"
#include "absdevice.h"


/*
 * Calculate time necessary to reduce the throttle until the value "THR_TAKE_OFF"
 * The higher the quadcopter is the longer it will take, because the take-down speed should be constant (MAX_FALL_SPEED_MS: ~0.833 m/s)
 */
inline float go_down_t(float altitude_m, float fThr) {
  // Calc save time (in ms) to touch the ground
  float fTime2Ground_ms = altitude_m / MAX_FALL_SPEED_MS * 1000.f;
  // Delta of the current throttle and the throttle needed to take-off
  float fdThr = fThr > THR_TAKE_OFF ? (fThr - THR_TAKE_OFF) / THR_MOD_STEP_S : fThr / THR_MOD_STEP_S;

  // Time constant (in ms) which determines the velocity to reduce the throttle
  return fTime2Ground_ms / fdThr;
}

///////////////////////////////////////////////////////////////////////////////////////
// ExeptionDevice
///////////////////////////////////////////////////////////////////////////////////////
bool ExeptionDevice::read_recvr() {
  if(m_bRcvrLock == true) {
    return m_bRcvrLock;
  }
  memcpy(m_rgChannelsRC, m_pReceiver->get_channels(), sizeof(m_rgChannelsRC) );
  m_bRcvrLock = true;
  return m_bRcvrLock;
}

bool ExeptionDevice::write_recvr() {
  if(m_bRcvrLock != true)  {
    return m_bRcvrLock;
  }
  m_bRcvrLock = false;
  m_pReceiver->set_errors(AbsErrorDevice::NOTHING_F);
  memset(m_rgChannelsRC, 0, sizeof(m_rgChannelsRC) );
  return m_bRcvrLock;
}

ExeptionDevice::ExeptionDevice(Device *pDevice, Receiver *pReceiver) {
  m_pHalBoard    = pDevice;
  m_pReceiver    = pReceiver;
  m_bRcvrLock    = false;
  memcpy(m_rgChannelsRC, m_pReceiver->get_channels(), sizeof(m_rgChannelsRC) );
}

void ExeptionDevice::dsbl_althld_recvr() {
  if(m_pReceiver->get_waypoint()->mode == GPSPosition::HLD_ALTITUDE_F) {
    m_pReceiver->get_waypoint()->mode = GPSPosition::NOTHING_F;
  }
}

void ExeptionDevice::dsbl_gpsnav_recvr() {
  if(m_pReceiver->get_waypoint()->mode == GPSPosition::GPS_NAVIGATN_F) {
    m_pReceiver->get_waypoint()->mode = GPSPosition::NOTHING_F;
  }
}

///////////////////////////////////////////////////////////////////////////////////////
// Exception
///////////////////////////////////////////////////////////////////////////////////////
Exception::Exception(Device *pDevice, Receiver *pReceiver) : ExeptionDevice(pDevice, pReceiver) {
  m_bPauseTD     = false;
  m_iPauseTDTime = 0;

  m_t32Pause = m_t32Altitude = m_t32Device = m_pHalBoard->m_pHAL->scheduler->millis();
}

void Exception::dev_take_down() {
  static bool bInertTimer = false;

  // If motors do not spin: reset & return
  if(m_pReceiver->get_channel(RC_THR) == RC_THR_OFF) {
    bInertTimer = false;
    m_pHalBoard->set_errors(AbsErrorDevice::NOTHING_F);
    m_pReceiver->get_waypoint()->mode = GPSPosition::NOTHING_F;
    write_recvr();
    return;
  } else {
    // Set the flag that copter is in controlled take down mode
    m_pReceiver->get_waypoint()->mode = GPSPosition::CONTRLD_DOWN_F;
  }

  // Don't reduce speed of the motors if pause set
  if(m_bPauseTD == true) {
    return;
  }

  // Set timer one time, to calculate how much the motors should be reduced
  if(bInertTimer == false) {
    m_t32Device = m_pHalBoard->m_pHAL->scheduler->millis();
    bInertTimer = true;
  }
  // Override the receiver, no matter what happens
  if(read_recvr() ) {
    reduce_thr(m_pHalBoard->m_pHAL->scheduler->millis() - m_t32Device - m_iPauseTDTime);
  }
}

void Exception::rcvr_take_down() {
  // Get time to calculate how much the motors should be reduced
  uint_fast32_t packet_t = m_pReceiver->last_parse_t32(); // Measure time elapsed since last successful package from WiFi or radio
  // If motors do not spin or a new packet arrived: reset & return
  if(m_pReceiver->get_channel(RC_THR) == RC_THR_OFF || packet_t <= COM_PKT_TIMEOUT ) {
    m_pReceiver->set_errors(AbsErrorDevice::NOTHING_F);
    m_pReceiver->get_waypoint()->mode = GPSPosition::NOTHING_F;
    write_recvr();
    return;
  } else {
    // Set the flag that copter is in controlled take down mode
    m_pReceiver->get_waypoint()->mode = GPSPosition::CONTRLD_DOWN_F;
  }

  // Don't reduce speed of the motors if pause set
  if(m_bPauseTD == true) {
    return;
  }
  // Remove the override if there was a new package within the interval
  if(read_recvr() ) {
    reduce_thr(packet_t - COM_PKT_TIMEOUT - m_iPauseTDTime);
  }
}

bool Exception::handle() {
  //////////////////////////////////////////////////////////////////////////////////////////
  // Device handler
  //////////////////////////////////////////////////////////////////////////////////////////
  if(m_pHalBoard->get_errors() & AbsErrorDevice::GYROMETER_F) {     // Gyrometer was not healthy: Go down straight
    #if DEBUG_OUT and !BENCH_OUT
    m_pHalBoard->m_pHAL->console->printf("Inertial exception - Taking model down\n");
    #endif

    dev_take_down();
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::ACCELEROMETR_F) {  // Accelerometer was not healthy: Go down straight
    #if DEBUG_OUT and !BENCH_OUT
    m_pHalBoard->m_pHAL->console->printf("Inertial exception - Taking model down\n");
    #endif

    dev_take_down();
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::BAROMETER_F) {     // If barometer is not working, the height calculation would be likely unreliable (GPS probably not stable)
    #if DEBUG_OUT and !BENCH_OUT
    m_pHalBoard->m_pHAL->console->printf("barometer exception - disable hold altitude\n");
    #endif

    dsbl_althld_recvr();
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::COMPASS_F) {       // If Compass not working, the GPS navigation shouldn't be used
    dsbl_gpsnav_recvr();
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::GPS_F) {           // And if GPS not working, the GPS navigation shouldn't be used at all :D
    dsbl_gpsnav_recvr();
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::VOLTAGE_HIGH_F) {
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::VOLTAGE_LOW_F) {
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::CURRENT_HIGH_F) {
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::CURRENT_LOW_F) {   // Battery is at the end: Go down straight
    #if DEBUG_OUT and !BENCH_OUT
    m_pHalBoard->m_pHAL->console->printf("current exception - Taking model down\n");
    #endif

    dev_take_down();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Remote control handler
  //////////////////////////////////////////////////////////////////////////////////////////
  if(m_pReceiver->get_errors() & AbsErrorDevice::UART_TIMEOUT_F) {
    #if DEBUG_OUT
    //m_pHalBoard->m_pHAL->console->printf("No signal from receiver for more than %d ms\n", COM_PKT_TIMEOUT);
    #endif

    rcvr_take_down();
    return true;
  }

  return false;
}

void Exception::reduce_thr(float fTime) {
  static float fStepC = 15.f;   // Default step size

  // The speed of decreasing the throttle is dependent on the height
  uint_fast32_t iAltitudeTime = m_pHalBoard->m_pHAL->scheduler->millis() - m_t32Altitude;
  if(iAltitudeTime > INAV_T_MS) {
    bool bOK = false;
	// Get the current position
    Location curPos;
    bOK = m_pHalBoard->m_pAHRS->get_position(curPos);
    float fAlti_m = curPos.alt / 100.f;

    if(bOK == true) {
      fStepC = go_down_t(fAlti_m, m_rgChannelsRC[RC_THR]);
      fStepC = fStepC < THR_MIN_STEP_S ? THR_MIN_STEP_S : fStepC;
    }
    // Save some variables and set timer
    m_t32Altitude   = m_pHalBoard->m_pHAL->scheduler->millis();
  }

  // Calculate how much to reduce throttle
  float fTConst = (THR_MOD_STEP_S * (fTime / fStepC) );
  int_fast16_t fThr = m_rgChannelsRC[RC_THR] - static_cast<int_fast16_t>(fTConst);

  #if DEBUG_OUT
  m_pHalBoard->m_pHAL->console->printf("Reduce throttle - fStepC: %f, fThr: %d\n", fStepC, fThr);
  #endif

  // reduce throttle..
  m_pReceiver->set_channel(RC_THR, fThr >= RC_THR_ACRO ? fThr : RC_THR_OFF); // throttle
  // reset yaw, pitch and roll
  m_pReceiver->set_channel(RC_YAW, 0);                                       // yaw
  m_pReceiver->set_channel(RC_PIT, 0);                                       // pitch
  m_pReceiver->set_channel(RC_ROL, 0);                                       // roll
}

void Exception::pause_take_down() {
  m_bPauseTD = true;
  m_t32Pause = m_pHalBoard->m_pHAL->scheduler->millis();
}

void Exception::cont_take_down() {
  if(m_bPauseTD == false) {
    return;
  }

  m_bPauseTD = false;
  m_iPauseTDTime += m_pHalBoard->m_pHAL->scheduler->millis() - m_t32Pause;
}
