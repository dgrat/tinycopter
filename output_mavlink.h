#ifndef OUT_M_h
#define OUT_M_h

#include <float.h>

#include "mavlink.h"
#include "global.h"
#include "containers.h"
#include "scheduler.h"


void mav_send_heartbeat_preflight(int uartX) {
  mavlink_system_t mavlink_system;
  mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
  mavlink_system.compid = MAV_COMP_ID_SYSTEM_CONTROL;     ///< The component sending the message is the IMU, it could be also a Linux process

  // Define the system type, in this case an airplane
  uint8_t system_type = MAV_TYPE_VTOL_QUADROTOR;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t mavLen = mavlink_msg_to_send_buffer(buf, &msg);

  AP_HAL::UARTDriver *pOut = uartX == UART_C ? hal.uartC : hal.uartA;
  uint16_t outLen = pOut->printf("%s", msg);

  if(mavLen != outLen) {
    pOut->printf("mav_send_heartbeat(): Packet size mismatch");
  }
}

///////////////////////////////////////////////////////////
// AHRS
///////////////////////////////////////////////////////////
void mav_send_ahrs(int uartX) {
  send_ahrs(uartX, *(_HAL_BOARD.m_pAHRS) );
}

///////////////////////////////////////////////////////////
// GPS raw
///////////////////////////////////////////////////////////
void mav_send_gps(int uartX) {
  send_gps_raw(uartX, *(_HAL_BOARD.m_pGPS) );
}

///////////////////////////////////////////////////////////
// GPS system time
///////////////////////////////////////////////////////////
void mav_send_gps(int uartX) {
  send_system_time(uartX, *(_HAL_BOARD.m_pGPS) );
}

///////////////////////////////////////////////////////////
// Inertial & compass
///////////////////////////////////////////////////////////
void mav_send_imu(int uartX) {
  send_raw_imu(uartX, *(_HAL_BOARD.m_pInert), *(_HAL_BOARD.m_pComp));
}

///////////////////////////////////////////////////////////
// barometer
///////////////////////////////////////////////////////////
void mav_send_baro(int uartX) {
  send_scaled_pressure(uartX, *(_HAL_BOARD.m_pBaro) );
}

///////////////////////////////////////////////////////////
// Intertial & compass & barometer sensor offsets
///////////////////////////////////////////////////////////
void mav_send_imu(int uartX) {
  send_sensor_offsets(uartX, *(_HAL_BOARD.m_pInert), *(_HAL_BOARD.m_pComp), *(_HAL_BOARD.m_pBaro) );
}

///////////////////////////////////////////////////////////
// battery monitor
///////////////////////////////////////////////////////////
void mav_send_bat(int uartX) {
  send_battery(uartX, *(_HAL_BOARD.m_pBat) );
}

///////////////////////////////////////////////////////////
// attitude in degrees
///////////////////////////////////////////////////////////
void mav_send_atti(int uartX) {
  static uint_fast32_t iTimer = 0;

  static float fAHRSX = FLT_MAX;
  static float fAHRSY = FLT_MAX;
  static float fAHRSZ = FLT_MAX;

  static float fGYRX = FLT_MAX;
  static float fGYRY = FLT_MAX;
  static float fGYRZ = FLT_MAX;

  float fAHRS_P = _HAL_BOARD.get_atti_cor_deg().x;
  float fAHRS_R = _HAL_BOARD.get_atti_cor_deg().y;
  float fAHRS_Y = _HAL_BOARD.get_atti_cor_deg().z;

  float fGYR_P = _HAL_BOARD.get_gyro_degps().x;
  float fGYR_R = _HAL_BOARD.get_gyro_degps().y;
  float fGYR_Y = _HAL_BOARD.get_gyro_degps().z;

  // Only use this function if the difference is significant
  // We should save some CPU time if possible
  uint_fast32_t iBCurTime = _HAL_BOARD.m_pHAL->scheduler->millis();
  if(iBCurTime - iTimer < 500) {
    if( fabs(fLX - fCX) < 1.f && fabs(fLY - fCY) < 1.f && fabs(fLZ - fCZ) < 2.5f ) {
      return;
    }
  } else {
    iTimer = iBCurTime;
  }

  fAHRSX = fAHRS_P;
  fAHRSY = fAHRS_R;
  fAHRSZ = fAHRS_Y;

  fGYRX = fGYR_P;
  fGYRY = fGYR_R;
  fGYRZ = fGYR_Y;

  mavlink_msg_attitude_send( uartX, iBCurTime,
    fAHRSX, fAHRSY, fAHRSZ,
    fGYRX, fGYRY, fGYRZ );
}

///////////////////////////////////////////////////////////
// GPS position
///////////////////////////////////////////////////////////
void mav_send_position(int uartX) {
  // Has fix?
  if(!_HAL_BOARD.m_pGPS->status() > 1) {
    return;
  }

  uint_fast32_t iBCurTime = _HAL_BOARD.m_pHAL->scheduler->millis();
  uint_fast32_t iCompass  = _HAL_BOARD.m_pAHRS->yaw_sensor;
  Location homeLoc        = _HAL_BOARD.m_pAHRS->get_home();
//  const Vector3f &v3fVel  = _HAL_BOARD.m_pInertNav->get_velocity();
  GPSData gps             = _HAL_BOARD.get_gps();

  mavlink_msg_global_position_int_send(
      uartX, iBCurTime,
      gps.latitude,
      gps.longitude,
      (homeLoc.alt + gps.altitude_cm),// cm above sea level
      gps.altitude_cm,                // cm above ground
      v3fVel.x,                       // X speed cm/s (+ve North)
      v3fVel.y,                       // Y speed cm/s (+ve East)
      v3fVel.z,                       // Z speed cm/s (+ve up)
      iCompass );                     // compass heading in 1/100 degree
}

///////////////////////////////////////////////////////////
// PID configuration
///////////////////////////////////////////////////////////
void mav_send_pids_attitude(int uartX) {
  // Capture values
  float pit_rkp   = _HAL_BOARD.get_pid(PID_PIT_RATE).kP();
  float pit_rki   = _HAL_BOARD.get_pid(PID_PIT_RATE).kI();
  float pit_rkd   = _HAL_BOARD.get_pid(PID_PIT_RATE).kD();
  float pit_rimax = _HAL_BOARD.get_pid(PID_PIT_RATE).imax();

  float rol_rkp   = _HAL_BOARD.get_pid(PID_ROL_RATE).kP();
  float rol_rki   = _HAL_BOARD.get_pid(PID_ROL_RATE).kI();
  float rol_rkd   = _HAL_BOARD.get_pid(PID_ROL_RATE).kD();
  float rol_rimax = _HAL_BOARD.get_pid(PID_ROL_RATE).imax();

  float yaw_rkp   = _HAL_BOARD.get_pid(PID_YAW_RATE).kP();
  float yaw_rki   = _HAL_BOARD.get_pid(PID_YAW_RATE).kI();
  float yaw_rkd   = _HAL_BOARD.get_pid(PID_YAW_RATE).kD();
  float yaw_rimax = _HAL_BOARD.get_pid(PID_YAW_RATE).imax();

  float pit_skp   = _HAL_BOARD.get_pid(PID_PIT_STAB).kP();
  float rol_skp   = _HAL_BOARD.get_pid(PID_ROL_STAB).kP();
  float yaw_skp   = _HAL_BOARD.get_pid(PID_YAW_STAB).kP();

  AP_HAL::UARTDriver *pOut = uartX == UART_C ? hal.uartC : hal.uartA;
  pOut->printf( "{\"t\":\"pid_cnf\","
                "\"p_rkp\":%.2f,\"p_rki\":%.2f,\"p_rkd\":%.4f,\"p_rimax\":%.2f,"
                "\"r_rkp\":%.2f,\"r_rki\":%.2f,\"r_rkd\":%.4f,\"r_rimax\":%.2f,"
                "\"y_rkp\":%.2f,\"y_rki\":%.2f,\"y_rkd\":%.4f,\"y_rimax\":%.2f,"
                "\"p_skp\":%.2f,\"r_skp\":%.2f,\"y_skp\":%.4f}\n",
                static_cast<double>(pit_rkp), static_cast<double>(pit_rki), static_cast<double>(pit_rkd), static_cast<double>(pit_rimax),
                static_cast<double>(rol_rkp), static_cast<double>(rol_rki), static_cast<double>(rol_rkd), static_cast<double>(rol_rimax),
                static_cast<double>(yaw_rkp), static_cast<double>(yaw_rki), static_cast<double>(yaw_rkd), static_cast<double>(yaw_rimax),
                static_cast<double>(pit_skp), static_cast<double>(rol_skp), static_cast<double>(yaw_skp) );
}

void mav_send_pids_altitude(int uartX) {
  // Capture values
  float thr_rkp   = _HAL_BOARD.get_pid(PID_THR_RATE).kP();
  float thr_rki   = _HAL_BOARD.get_pid(PID_THR_RATE).kI();
  float thr_rkd   = _HAL_BOARD.get_pid(PID_THR_RATE).kD();
  float thr_rimax = _HAL_BOARD.get_pid(PID_THR_RATE).imax();

  float acc_rkp   = _HAL_BOARD.get_pid(PID_ACC_RATE).kP();
  float acc_rki   = _HAL_BOARD.get_pid(PID_ACC_RATE).kI();
  float acc_rkd   = _HAL_BOARD.get_pid(PID_ACC_RATE).kD();
  float acc_rimax = _HAL_BOARD.get_pid(PID_ACC_RATE).imax();

  float thr_skp   = _HAL_BOARD.get_pid(PID_THR_STAB).kP();
  float acc_skp   = _HAL_BOARD.get_pid(PID_ACC_STAB).kP();

  AP_HAL::UARTDriver *pOut = uartX == UART_C ? hal.uartC : hal.uartA;
  pOut->printf( "{\"t\":\"pid_cnf\","
                "\"t_rkp\":%.2f,\"t_rki\":%.2f,\"t_rkd\":%.4f,\"t_rimax\":%.2f,"
                "\"a_rkp\":%.2f,\"a_rki\":%.2f,\"a_rkd\":%.4f,\"a_rimax\":%.2f,"
                "\"t_skp\":%.2f,\"a_skp\":%.2f}\n",
                static_cast<double>(thr_rkp), static_cast<double>(thr_rki), static_cast<double>(thr_rkd), static_cast<double>(thr_rimax),
                static_cast<double>(acc_rkp), static_cast<double>(acc_rki), static_cast<double>(acc_rkd), static_cast<double>(acc_rimax),
                static_cast<double>(thr_skp), static_cast<double>(acc_skp) );
}

#endif

