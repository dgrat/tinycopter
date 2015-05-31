#include <AP_InertialSensor.h> // for user interactant
#include <AP_AHRS.h>
#include <AP_Param.h>
#include <RC_Channel.h>     // RC Channel Library

#include <float.h>

#include "receiver.h"
#include "scheduler.h"
#include "device.h"
#include "arithmetics.h"


///////////////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////////////
namespace tiny {
inline char *strncpy ( char *destination, const char *source, size_t num ) {
  destination[num] = '\0';                  // Null termination is important
  return (char *)memcpy(destination, source, num);  // Probably most efficient method to copy a cstring
}

inline int jsoneq(const char *json, jsmntok_t *tok, const char *cstr) {
  if (tok->type == JSMN_STRING && (int) strlen(cstr) == tok->end - tok->start &&
      strncmp(json + tok->start, cstr, tok->end - tok->start) == 0) {
    return 0;
  }
  return -1;
}

// JSON: type: message ?accepted?, ID of the parser type: ctrl/pid, .., return of the function (true, false)
void send_reply_msg(AP_HAL::UARTDriver *pUART, const char *cstr, bool bState) {
  #ifdef __AVR__
  pUART->printf_P(PSTR("{\"t\":\"acpt\",\"id\":\"%s\",\"ret\":%d}\n"), cstr, bState);
  #else
  pUART->printf("{\"t\":\"acpt\",\"id\":\"%s\",\"ret\":%d}\n", cstr, bState);
  #endif
}

} /*namespace tiny*/

inline void run_calibration(Device *pHalBoard, Scheduler *m_pOutSched, AP_HAL::UARTDriver *pUART) {
  while(pUART->available() ) {
    pUART->read();
  }

  #if !defined( __AVR_ATmega1280__ )
  // Stop the sensor output
  m_pOutSched->stop();

  // Do the calibration
  float roll_trim, pitch_trim;
  AP_InertialSensor_UserInteractStream interact(pUART);
  pHalBoard->m_pInert->calibrate_accel(&interact, roll_trim, pitch_trim);

  // Adjust AHRS
  pHalBoard->m_pAHRS->set_trim(Vector3f(roll_trim, pitch_trim, 0) );

  // After all, resume the sensor output
  m_pOutSched->resume();
  #else
  
  #ifdef __AVR__
  pUART->println_P(PSTR("calibrate_accel not available on 1280") );
  #else
  pUART->println("calibrate_accel not available on 1280");
  #endif
#endif
}

inline bool check_input(int_fast16_t iRol, int_fast16_t iPit, int_fast16_t iThr, int_fast16_t iYaw) {
  if(!tiny::in_range(RC_PIT_MIN, RC_PIT_MAX, iPit) ) {
    return false;
  }
  if(!tiny::in_range(RC_ROL_MIN, RC_ROL_MAX, iRol) ) {
    return false;
  }
  if(!tiny::in_range(RC_THR_OFF, RC_THR_MAX, iThr) ) {
    return false;
  }
  if(!tiny::in_range(RC_YAW_MIN, RC_YAW_MAX, iYaw) ) {
    return false;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////
// Receiver
///////////////////////////////////////////////////////////////////////////////////////
Receiver::Receiver(Device *pHalBoard, Scheduler *pTMUart) {
  m_pHalBoard = pHalBoard;
  m_pTMUartOut = pTMUart;
  m_pCurUART = m_pHalBoard->m_pHAL->uartA;

  memset(m_cBuffer, 0, sizeof(m_cBuffer) );
  memset(m_rgChannelsRC, 0, sizeof(m_rgChannelsRC) );

  jsmn_init(&m_JSONParser);
  m_JSONNumToken = 0;

  m_iPPMTimer = m_iSParseTimer_A = m_iSParseTimer_C = m_iSParseTimer = m_pHalBoard->m_pHAL->scheduler->millis();
  m_iPPMTime  = m_iSParseTime_A  = m_iSParseTime_C  = m_iSParseTime  = 0;
  m_eErrors   = NOTHING_F;

  m_pRCRol = new RC_Channel(RC_ROL);
  m_pRCPit = new RC_Channel(RC_PIT);
  m_pRCThr = new RC_Channel(RC_THR);
  m_pRCYaw = new RC_Channel(RC_YAW);

  // setup radio
  if (m_pRCThr->radio_min == 0) {
    // cope with AP_Param not being loaded
    m_pRCThr->radio_min = RC_THR_OFF;
  }
  if (m_pRCThr->radio_max == 0) {
    // cope with AP_Param not being loaded
    m_pRCThr->radio_max = RC_THR_MAX;
  }

  // set rc channel ranges
  m_pRCRol->set_angle(RC_ROL_MAX*100);
  m_pRCPit->set_angle(RC_PIT_MAX*100);
  m_pRCThr->set_range(RC_THR_ACRO-RC_THR_OFF, RC_THR_MAX-RC_THR_OFF);
  m_pRCYaw->set_angle(RC_YAW_MAX*100);
}

void Receiver::set_channel(uint_fast8_t index, int_fast32_t value) {
  if(index >= APM_IOCHAN_CNT) {
    m_rgChannelsRC[APM_IOCHAN_CNT-1] = value;
  }
  m_rgChannelsRC[index] = value;
}

int_fast32_t Receiver::get_channel(uint_fast8_t index) const {
  if(index >= APM_IOCHAN_CNT) {
    return m_rgChannelsRC[APM_IOCHAN_CNT-1];
  }
  return m_rgChannelsRC[index];
}

int_fast32_t *Receiver::get_channels() {
  return &m_rgChannelsRC[0];
}

GPSPosition *Receiver::get_waypoint() {
  return &m_Waypoint;
}

uint_fast32_t Receiver::last_parse_t32() {
  m_iSParseTime = m_pHalBoard->m_pHAL->scheduler->millis() - m_iSParseTimer;

  if(m_iSParseTime > COM_PKT_TIMEOUT) {
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(tiny::add_flag(m_eErrors, UART_TIMEOUT_F) );
  }

  return m_iSParseTime;
}

uint_fast32_t Receiver::last_parse_uartA_t32() {
  m_iSParseTime_A = m_pHalBoard->m_pHAL->scheduler->millis() - m_iSParseTimer_A;
  return m_iSParseTime_A;
}

uint_fast32_t Receiver::last_parse_uartC_t32() {
  m_iSParseTime_C = m_pHalBoard->m_pHAL->scheduler->millis() - m_iSParseTimer_C;
  return m_iSParseTime_C;
}

uint_fast32_t Receiver::last_rcin_t32() {
  m_iPPMTime = m_pHalBoard->m_pHAL->scheduler->millis() - m_iPPMTimer;
  return m_iPPMTime;
}

bool Receiver::parse_set_par(int i) {
  #if USE_CRC16
  unsigned short iCRC = 0;
  #endif
  char AP_ParamPayl[sizeof(m_cJSONKey)] = { 0 };
  AP_Param *pParam = NULL;
  enum ap_var_type p_type;

  ///////////////////////////////////////////////////////////////////////////////////
  // JSON parser section
  ///////////////////////////////////////////////////////////////////////////////////
  for(; i < m_JSONNumToken; i++) {
    uint_fast16_t iStrLen = m_JSONToken[i+1].end - m_JSONToken[i+1].start;
    if(iStrLen > sizeof(m_cJSONKey) ) {
      return false;
    }
    const char *cStr = m_cBuffer + m_JSONToken[i+1].start;
    char *cType = tiny::strncpy(m_cJSONKey, cStr, iStrLen);

    if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "name") == 0) {
      pParam = AP_Param::find(cType, &p_type);
      if(!pParam) {
        #if DBGRC_OUT
          #ifdef __AVR__
          m_pCurUART->printf_P(PSTR("parse_set_param() - parameter not found\n") );
          #else
          m_pCurUART->printf("parse_set_param() - parameter not found\n");
          #endif
        #endif
        return false;
      }
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "value") == 0) {
      tiny::strncpy(AP_ParamPayl, cType, strlen(cType) );
      i++;
    }
    #if USE_CRC16
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "crc") == 0) {
      iCRC = strtol(cType, NULL, 16);
      i++;
    }
    #endif
    else {
      #if DBGRC_OUT
        #ifdef __AVR__
        m_pCurUART->printf_P(PSTR("parse_set_param() - unexpected key: %.*s\n"), m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #else
        m_pCurUART->printf("parse_set_param() - unexpected key: %.*s\n", m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #endif
      #endif
      return false;
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////
  // Check points are here
  ///////////////////////////////////////////////////////////////////////////////////
  #if USE_CRC16
  if(iCRC != tiny::crc16(m_cBuffer, strlen(m_cBuffer) ) ) {
    #if DBGRC_OUT
      #ifdef __AVR__
      m_pCurUART->printf_P(PSTR("parse_ctrl_com(): failed CRC16 check\n") );
      #else
      m_pCurUART->printf("parse_ctrl_com(): failed CRC16 check\n");
      #endif
    #endif
    return false;
  }
  #endif

  // Check whether AP_Parm was set correctly
  if(!pParam) {
    #if DBGRC_OUT
      #ifdef __AVR__
      m_pCurUART->printf_P(PSTR("parse_set_param() - parameter not set\n") );
      #else
      m_pCurUART->printf("parse_set_param() - parameter not set\n");
      #endif
    #endif
    return false;
  }

  // Check whether a payload was set or the array is untouched
  int iSum = 0;
  for(unsigned int k = 0; k < sizeof(m_cJSONKey); k++) {
    iSum += AP_ParamPayl[k];
  }
  if(!iSum) {
    #if DBGRC_OUT
      #ifdef __AVR__
      m_pCurUART->printf_P(PSTR("parse_set_param() - the parameter payload was not defined\n") );
      #else
      m_pCurUART->printf("parse_set_param() - the parameter payload was not defined\n");
      #endif
    #endif
    return false;
  }

  ///////////////////////////////////////////////////////////////////////////////////
  // Here we switch through the parameter types
  ///////////////////////////////////////////////////////////////////////////////////
  int32_t value_int32 = 0;
  float   value_flt32 = 0.f;
  switch(p_type) {
    case AP_PARAM_INT8:
      value_int32 = strtol(AP_ParamPayl, NULL, 10);
      if(!tiny::in_range(-127, 127, value_int32) ) {
        #if DBGRC_OUT
          #ifdef __AVR__
          m_pCurUART->printf_P(PSTR("parse_set_param() - the parameter payload is out of the range (type is INT8), payload is %d\n"), value_int32);
          #else
          m_pCurUART->printf("parse_set_param() - the parameter payload is out of the range (type is INT8), payload is %d\n", value_int32);
          #endif
        #endif
        return false;
      }
      ((AP_Int8*)pParam)->set_and_save((int8_t)value_int32);
      break;

    case AP_PARAM_INT16:
      value_int32 = strtol(AP_ParamPayl, NULL, 10);
      if(!tiny::in_range(-32767, 32767, value_int32) ) {
        #if DBGRC_OUT
          #ifdef __AVR__
          m_pCurUART->printf_P(PSTR("parse_set_param() - the parameter payload is out of the range (type is INT16), payload is %d\n"), value_int32);
          #else
          m_pCurUART->printf("parse_set_param() - the parameter payload is out of the range (type is INT16), payload is %d\n", value_int32);
          #endif
        #endif
        return false;
      }
      ((AP_Int16*)pParam)->set_and_save((int16_t)value_int32);
      break;

    case AP_PARAM_INT32:
      value_int32 = strtol(AP_ParamPayl, NULL, 10);
      ((AP_Int32*)pParam)->set_and_save(value_int32);
      break;

    case AP_PARAM_FLOAT:
      value_flt32 = atof(AP_ParamPayl);
      ((AP_Float*)pParam)->set_and_save(value_flt32);
      break;

    default:
      #if DBGRC_OUT
        #ifdef __AVR__
        m_pCurUART->printf_P(PSTR("parse_set_param() - cannot set parameter of type %d\n"), p_type);
        #else
        m_pCurUART->printf("parse_set_param() - cannot set parameter of type %d\n", p_type);
        #endif
      #endif
      return false;
  }
    
  return true;  
}

// Remote control:
// Change the input only if everything seems fine :)
bool Receiver::parse_ctrl_com(int i) {
  // Save old remote input
  float fPit = m_rgChannelsRC[RC_PIT];
  float fRol = m_rgChannelsRC[RC_ROL];
  float fThr = m_rgChannelsRC[RC_THR];
  float fYaw = m_rgChannelsRC[RC_YAW];
  #if USE_CRC16
  unsigned short iCRC = 0;
  #endif
  for(; i < m_JSONNumToken; i++) {
    uint_fast16_t iStrLen = m_JSONToken[i+1].end - m_JSONToken[i+1].start;
    if(iStrLen > sizeof(m_cJSONKey) ) {
      return false;
    }
    const char *cStr = m_cBuffer + m_JSONToken[i+1].start;
    char *cType = tiny::strncpy(m_cJSONKey, cStr, iStrLen);

    if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "r") == 0) {
      fRol = strtol(cType, NULL, 10);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "p") == 0) {
      fPit = strtol(cType, NULL, 10);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "f") == 0) {
      fThr = strtol(cType, NULL, 10);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "y") == 0) {
      fYaw = strtol(cType, NULL, 10);
      i++;
    }
    #if USE_CRC16
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "crc") == 0) {
      iCRC = strtol(cType, NULL, 16);
      i++;
    }
    #endif
    else {
      #if DBGRC_OUT
        #ifdef __AVR__
        m_pCurUART->printf_P(PSTR("parse_ctrl_com() - unexpected key: %.*s\n"), m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #else
        m_pCurUART->printf("parse_ctrl_com() - unexpected key: %.*s\n", m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #endif
      #endif
      return false;
    }
  }
  
  #if USE_CRC16
  if(iCRC != tiny::crc16(m_cBuffer, strlen(m_cBuffer) ) ) {
    #if DBGRC_OUT
      #ifdef __AVR__
      m_pCurUART->printf_P(PSTR("parse_ctrl_com(): failed CRC16 check\n") );
      #else
      m_pCurUART->printf("parse_ctrl_com(): failed CRC16 check\n");
      #endif
    #endif
    return false;
  }
  #endif
  
  // Validity check: second ratings
  if(!check_input(fRol, fPit, fThr, fYaw) ) {
    return false;
  }
  
  // Apply new input
  m_rgChannelsRC[RC_ROL] = fRol;
  m_rgChannelsRC[RC_PIT] = fPit;
  m_rgChannelsRC[RC_THR] = fThr;
  m_rgChannelsRC[RC_YAW] = fYaw;
  
  m_iSParseTimer = m_pHalBoard->m_pHAL->scheduler->millis(); // update last valid packet
  
  return true;
}

// drift compensation
// maximum value is between -10 and 10 degrees
bool Receiver::parse_gyr_cor(int i) {
  float fRol = FLT_MAX;
  float fPit = FLT_MAX;
  #if USE_CRC16
  unsigned short iCRC = 0;
  #endif
  for(; i < m_JSONNumToken; i++) {
    uint_fast16_t iStrLen = m_JSONToken[i+1].end - m_JSONToken[i+1].start;
    if(iStrLen > sizeof(m_cJSONKey) ) {
      return false;
    }
    const char *cStr = m_cBuffer + m_JSONToken[i+1].start;
    char *cType = tiny::strncpy(m_cJSONKey, cStr, iStrLen);

    if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "r") == 0) {
      fRol = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "p") == 0) {
      fPit = atof(cType);
      i++;
    }
    #if USE_CRC16
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "crc") == 0) {
      iCRC = strtol(cType, NULL, 16);
      i++;
    }
    #endif
    else {
      #if DBGRC_OUT
        #ifdef __AVR__
        m_pCurUART->printf_P(PSTR("parse_ctrl_com() - unexpected key: %.*s\n"), m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #else
        m_pCurUART->printf("parse_ctrl_com() - unexpected key: %.*s\n", m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #endif
      #endif
      return false;
    }
  }

  #if USE_CRC16
  if(iCRC != tiny::crc16(m_cBuffer, strlen(m_cBuffer) ) ) {
    #if DBGRC_OUT
      #ifdef __AVR__
      m_pCurUART->printf_P(PSTR("parse_ctrl_com(): failed CRC16 check\n") );
      #else
      m_pCurUART->printf("parse_ctrl_com(): failed CRC16 check\n");
      #endif
    #endif
    return false;
  }
  #endif
  
  // sanity check
  if(!tiny::in_range(-10.f, 10.f, fRol) || !tiny::in_range(-10.f, 10.f, fPit) ) {
    return false;
  }
  else {
    m_pHalBoard->set_trims(fRol, fPit);
  }
  return true;
}

bool Receiver::parse_waypoint(int i) {
  bool bRet = false;
  int_fast32_t lat           = 0;
  int_fast32_t lon           = 0;
  int_fast32_t alt_cm        = 0;
  #if USE_CRC16
  unsigned short iCRC = 0;
  #endif
  GPSPosition::UAV_TYPE flag = GPSPosition::NOTHING_F;

  for(; i < m_JSONNumToken; i++) {
    uint_fast16_t iStrLen = m_JSONToken[i+1].end - m_JSONToken[i+1].start;
    if(iStrLen > sizeof(m_cJSONKey) ) {
      return false;
    }
    const char *cStr = m_cBuffer + m_JSONToken[i+1].start;
    char *cType = tiny::strncpy(m_cJSONKey, cStr, iStrLen);

    if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "lat_d") == 0) {
      lat = strtol(cType, NULL, 10);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "lon_d") == 0) {
      lon = strtol(cType, NULL, 10);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "alt_m") == 0) {
      alt_cm = strtol(cType, NULL, 10);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "flag_t") == 0) {
      // Parse the type flag
      flag = static_cast<GPSPosition::UAV_TYPE>(strtol(cType, NULL, 10) );
      // Override the height if the flag is HLD_ALTITUDE_F
      if(flag == GPSPosition::HLD_ALTITUDE_F) {
        // Measure the current height
        Location curPos;
        bRet = m_pHalBoard->m_pAHRS->get_position(curPos);
        alt_cm = curPos.alt;
        // If height measurement failed, then break it
        if(!bRet) {
          flag = GPSPosition::NOTHING_F;
        }
        bRet = true;
      }
      i++;
    }
    #if USE_CRC16
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "crc") == 0) {
      iCRC = strtol(cType, NULL, 16);
      i++;
    }
    #endif
    else {
      #if DBGRC_OUT
        #ifdef __AVR__
        m_pCurUART->printf_P(PSTR("parse_waypoint() - unexpected key: %.*s\n"), m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #else
        m_pCurUART->printf("parse_waypoint() - unexpected key: %.*s\n", m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #endif
      #endif
      return false;
    }
  }

  #if USE_CRC16
  if(iCRC != tiny::crc16(m_cBuffer, strlen(m_cBuffer) ) ) {
    #if DBGRC_OUT
      #ifdef __AVR__
      m_pCurUART->printf_P(PSTR("parse_waypoint(): failed CRC16 check\n") ;
      #else
      m_pCurUART->printf("parse_waypoint(): failed CRC16 check\n");
      #endif
    #endif
    return false;
  }
  #endif
  
  // Set new waypoint only if the everything worked out like expected
  if(bRet == true) {
    m_Waypoint = GPSPosition(lat, lon, alt_cm, flag);
  }

  return bRet;
}

bool Receiver::parse_gyr_cal(int i) {
  if(m_rgChannelsRC[RC_THR] > RC_THR_ACRO) {
    return false;
  }
  #if USE_CRC16
  unsigned short iCRC = 0;
  #endif
  bool bcalib = false;

  for(; i < m_JSONNumToken; i++) {
    uint_fast16_t iStrLen = m_JSONToken[i+1].end - m_JSONToken[i+1].start;
    if(iStrLen > sizeof(m_cJSONKey) ) {
      return false;
    }
    const char *cStr = m_cBuffer + m_JSONToken[i+1].start;
    char *cType = tiny::strncpy(m_cJSONKey, cStr, iStrLen);

    if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "cal") == 0) {
      bcalib = strtol(cType, NULL, 10);
      i++;
    }
    #if USE_CRC16
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "crc") == 0) {
      iCRC = strtol(cType, NULL, 16);
      i++;
    }
    #endif
    else {
      #if DBGRC_OUT
        #ifdef __AVR__
        m_pCurUART->printf_P(PSTR("parse_gyr_cal() - unexpected key: %.*s\n"), m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #else
        m_pCurUART->printf("parse_gyr_cal() - unexpected key: %.*s\n", m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #endif
      #endif
      return false;
    }
  }

  #if USE_CRC16
  if(iCRC != tiny::crc16(m_cBuffer, strlen(m_cBuffer) ) ) {
    #if DBGRC_OUT
      #ifdef __AVR__
      m_pCurUART->printf_P(PSTR("parse_gyr_cal(): failed CRC16 check\n") );
      #else
      m_pCurUART->printf("parse_gyr_cal(): failed CRC16 check\n");
      #endif
    #endif
    return false;
  }
  #endif  
 
 if(bcalib) {
    run_calibration(m_pHalBoard, m_pTMUartOut, m_pCurUART);
  }
 
  return true;
}

bool Receiver::parse_pid_conf(int i) {
  if(m_rgChannelsRC[RC_THR] > RC_THR_ACRO) {             // If motors run: Do nothing!
    return false;
  }
  #if USE_CRC16
  unsigned short iCRC = 0;
  #endif

  // Init the variables storing the PIDs
  float p_rkp = m_pHalBoard->get_pid(PID_PIT_RATE).kP();
  float p_rki = m_pHalBoard->get_pid(PID_PIT_RATE).kI();
  float p_rkd = m_pHalBoard->get_pid(PID_PIT_RATE).kD();
  float p_rimax = m_pHalBoard->get_pid(PID_PIT_RATE).imax();

  float r_rkp = m_pHalBoard->get_pid(PID_ROL_RATE).kP();
  float r_rki = m_pHalBoard->get_pid(PID_ROL_RATE).kI();
  float r_rkd = m_pHalBoard->get_pid(PID_ROL_RATE).kD();
  float r_rimax = m_pHalBoard->get_pid(PID_ROL_RATE).imax();

  float y_rkp = m_pHalBoard->get_pid(PID_YAW_RATE).kP();
  float y_rki = m_pHalBoard->get_pid(PID_YAW_RATE).kI();
  float y_rkd = m_pHalBoard->get_pid(PID_YAW_RATE).kD();
  float y_rimax = m_pHalBoard->get_pid(PID_YAW_RATE).imax();

  float t_rkp = m_pHalBoard->get_pid(PID_THR_RATE).kP();
  float t_rki = m_pHalBoard->get_pid(PID_THR_RATE).kI();
  float t_rkd = m_pHalBoard->get_pid(PID_THR_RATE).kD();
  float t_rimax = m_pHalBoard->get_pid(PID_THR_RATE).imax();

  float a_rkp = m_pHalBoard->get_pid(PID_ACC_RATE).kP();
  float a_rki = m_pHalBoard->get_pid(PID_ACC_RATE).kI();
  float a_rkd = m_pHalBoard->get_pid(PID_ACC_RATE).kD();
  float a_rimax = m_pHalBoard->get_pid(PID_ACC_RATE).imax();

  float p_skp = m_pHalBoard->get_pid(PID_PIT_STAB).kP();
  float r_skp = m_pHalBoard->get_pid(PID_ROL_STAB).kP();
  float y_skp = m_pHalBoard->get_pid(PID_YAW_STAB).kP();
  float t_skp = m_pHalBoard->get_pid(PID_THR_STAB).kP();
  float a_skp = m_pHalBoard->get_pid(PID_ACC_STAB).kP();

  // Parse the new PIDs
  for(; i < m_JSONNumToken; i++) {
    uint_fast16_t iStrLen = m_JSONToken[i+1].end - m_JSONToken[i+1].start;
    if(iStrLen > sizeof(m_cJSONKey) ) {
      return false;
    }
    const char *cStr = m_cBuffer + m_JSONToken[i+1].start;
    char *cType = tiny::strncpy(m_cJSONKey, cStr, iStrLen);

    // PITCH
    if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "p_rkp") == 0) {
      p_rkp = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "p_rki") == 0) {
      p_rki = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "p_rkd") == 0) {
      p_rkd = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "p_rimax") == 0) {
      p_rimax = atof(cType);
      i++;
    }
    // ROLL
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "r_rkp") == 0) {
      r_rkp = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "r_rki") == 0) {
      r_rki = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "r_rkd") == 0) {
      r_rkd = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "r_rimax") == 0) {
      r_rimax = atof(cType);
      i++;
    }
    // YAW
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "y_rkp") == 0) {
      y_rkp = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "y_rki") == 0) {
      y_rki = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "y_rkd") == 0) {
      y_rkd = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "y_rimax") == 0) {
      y_rimax = atof(cType);
      i++;
    }
    // THR
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "t_rkp") == 0) {
      t_rkp = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "t_rki") == 0) {
      t_rki = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "t_rkd") == 0) {
      t_rkd = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "t_rimax") == 0) {
      t_rimax = atof(cType);
      i++;
    }
    // ACCEL
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "a_rkp") == 0) {
      a_rkp = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "a_rki") == 0) {
      a_rki = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "a_rkd") == 0) {
      a_rkd = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "a_rimax") == 0) {
      a_rimax = atof(cType);
      i++;
    }
    // STABS
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "p_skp") == 0) {
      p_skp = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "r_skp") == 0) {
      r_skp = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "y_skp") == 0) {
      y_skp = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "t_skp") == 0) {
      t_skp = atof(cType);
      i++;
    }
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "a_skp") == 0) {
      a_skp = atof(cType);
      i++;
    }
    #if USE_CRC16
    else if (tiny::jsoneq(m_cBuffer, &m_JSONToken[i], "crc") == 0) {
      iCRC = strtol(cType, NULL, 16);
      i++;
    }
    #endif
    else {
      #if DBGRC_OUT
        #ifdef __AVR__
        m_pCurUART->printf_P(PSTR("parse_pid_conf() - unexpected key: %.*s\n"), m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #else
        m_pCurUART->printf("parse_pid_conf() - unexpected key: %.*s\n", m_JSONToken[i].end-m_JSONToken[i].start, m_cBuffer + m_JSONToken[i].start);
        #endif
      #endif
      return false;
    }
  }
  
  #if USE_CRC16
  if(iCRC != tiny::crc16(m_cBuffer, strlen(m_cBuffer) ) ) {
    #if DBGRC_OUT
      #ifdef __AVR__
      m_pCurUART->printf_P(PSTR("parse_pid_conf(): failed CRC16 check\n") );
      #else
      m_pCurUART->printf("parse_pid_conf(): failed CRC16 check\n");
      #endif
    #endif
    return false;
  }
  #endif

  // Save the new, parsed PIDs
  m_pHalBoard->get_pid(PID_PIT_RATE).kP(p_rkp);
  m_pHalBoard->get_pid(PID_PIT_RATE).kI(p_rki);
  m_pHalBoard->get_pid(PID_PIT_RATE).kD(p_rkd);
  m_pHalBoard->get_pid(PID_PIT_RATE).imax(p_rimax);

  m_pHalBoard->get_pid(PID_ROL_RATE).kP(r_rkp);
  m_pHalBoard->get_pid(PID_ROL_RATE).kI(r_rki);
  m_pHalBoard->get_pid(PID_ROL_RATE).kD(r_rkd);
  m_pHalBoard->get_pid(PID_ROL_RATE).imax(r_rimax);

  m_pHalBoard->get_pid(PID_YAW_RATE).kP(y_rkp);
  m_pHalBoard->get_pid(PID_YAW_RATE).kI(y_rki);
  m_pHalBoard->get_pid(PID_YAW_RATE).kD(y_rkd);
  m_pHalBoard->get_pid(PID_YAW_RATE).imax(y_rimax);

  m_pHalBoard->get_pid(PID_THR_RATE).kP(t_rkp);
  m_pHalBoard->get_pid(PID_THR_RATE).kI(t_rki);
  m_pHalBoard->get_pid(PID_THR_RATE).kD(t_rkd);
  m_pHalBoard->get_pid(PID_THR_RATE).imax(t_rimax);

  m_pHalBoard->get_pid(PID_ACC_RATE).kP(a_rkp);
  m_pHalBoard->get_pid(PID_ACC_RATE).kI(a_rki);
  m_pHalBoard->get_pid(PID_ACC_RATE).kD(a_rkd);
  m_pHalBoard->get_pid(PID_ACC_RATE).imax(a_rimax);

  m_pHalBoard->get_pid(PID_PIT_STAB).kP(p_skp);
  m_pHalBoard->get_pid(PID_ROL_STAB).kP(r_skp);
  m_pHalBoard->get_pid(PID_YAW_STAB).kP(y_skp);
  m_pHalBoard->get_pid(PID_THR_STAB).kP(t_skp);
  m_pHalBoard->get_pid(PID_ACC_STAB).kP(a_skp);

  m_pHalBoard->save_pids();
  return true;
}

/*
 * Compact remote control packet system for the radio on Uart2,
 * Everything fits into 7 bytes
 */
bool Receiver::parse_radio() {
  int_fast16_t thr = 1000 + (static_cast<uint_fast16_t>(m_cBuffer[0]) * 100) + (uint_fast16_t)m_cBuffer[1];    // 1000 - 1900
  int_fast16_t pit = static_cast<int_fast16_t>(m_cBuffer[2]);                                                  // -45° - 45°
  int_fast16_t rol = static_cast<int_fast16_t>(m_cBuffer[3]);                                                  // -45° - 45°
  int_fast16_t yaw = static_cast<uint_fast8_t>(m_cBuffer[5]) * static_cast<int_fast16_t>(m_cBuffer[4]);        // -180° - 180°
  uint_fast8_t chk = static_cast<uint_fast8_t>(m_cBuffer[6]);                                                  // checksum byte

  // Validity check: first checksum (crc8)
  const uint_fast8_t iLnMsg = 6;
  if(tiny::crc8(m_cBuffer, iLnMsg) != chk) {
    return false;
  }
  // Validity check: second ratings
  else if(!check_input(rol, pit, thr, yaw) ) {
    return false;
  }
  
  // Set values if everything seems fine
  m_rgChannelsRC[RC_ROL] = rol;
  m_rgChannelsRC[RC_PIT] = pit;
  m_rgChannelsRC[RC_THR] = thr;
  m_rgChannelsRC[RC_YAW] = yaw;

  m_iSParseTimer = m_pHalBoard->m_pHAL->scheduler->millis();                                                   // update last valid packet
  return true;
}

bool Receiver::parse_cstr() {
  const int iFirstKey = 1; // ID of the first key in the JSON string
  const int iSeconKey = 3; // ID of the second key in the JSON string
  // Minimum number of tokens in the message (if the number of tokens is lower, the message is certainly broken)
  #if USE_CRC16
  const int iMinNrTok = 7;
  #else 
  const int iMinNrTok = 5;
  #endif
  
  // Reset the JSON parser
  memset(&m_cJSONKey[0], 0, sizeof(m_cJSONKey) );
  jsmn_init(&m_JSONParser);
  m_JSONNumToken = jsmn_parse( &m_JSONParser, m_cBuffer, strlen(m_cBuffer), m_JSONToken,
                               sizeof(m_JSONToken)/sizeof(m_JSONToken[0]) );

  #if DBGRC_OUT
    #ifdef __AVR__
    m_pCurUART->printf_P(PSTR("parse_cstr(): %s\n"), m_cBuffer);
    #else
    m_pCurUART->printf("parse_cstr(): %s\n", m_cBuffer);
    #endif
  #endif

	if (m_JSONNumToken < 0) {
    #if DBGRC_OUT
      #ifdef __AVR__
      m_pCurUART->printf_P(PSTR("parse_cstr(): failed to parse JSON: %d\n"), m_JSONNumToken);
      #else
      m_pCurUART->printf("parse_cstr(): failed to parse JSON: %d\n", m_JSONNumToken);
      #endif
    #endif
		return false;
	}

	// Assume the top-level element is an object ..
  // .. we should have at least 5 objects in every message (type_id = 2, key_1 = 2, toplevel = 1) == 5  
	if (m_JSONNumToken < iMinNrTok || m_JSONToken[0].type != JSMN_OBJECT) {
    #if DBGRC_OUT
      #ifdef __AVR__
      m_pCurUART->printf_P(PSTR("parse_cstr(): more objects expected\n") );
      m_pCurUART->printf_P(PSTR("I found just %d objects\n"), m_JSONNumToken);
      #else
      m_pCurUART->printf("parse_cstr(): more objects expected\n");
      m_pCurUART->printf("I found just %d objects\n", m_JSONNumToken);
      #endif
    #endif
		return false;
	}
  
  // JSON has the type identifier 't'
  if (tiny::jsoneq(m_cBuffer, &m_JSONToken[iFirstKey], "t") == 0) {
    // Get the type of the JSON
    uint_fast16_t iStrLen = m_JSONToken[iFirstKey+1].end - m_JSONToken[iFirstKey+1].start;
    if(iStrLen > sizeof(m_cJSONKey) ) {
      return false;
    }
    const char *cStr = m_cBuffer + m_JSONToken[iFirstKey+1].start;
    char *cType = tiny::strncpy(m_cJSONKey, cStr, iStrLen);

    // Remote control command
    if(strcmp(cType, "rc") == 0) {
      return parse_ctrl_com(iSeconKey);
    }
    // attitude offset for small imbalances
    else if(strcmp(cType, "cmp") == 0) {
      bool bRet = parse_gyr_cor(iSeconKey);
      tiny::send_reply_msg(m_pCurUART, "cmp", bRet);
      return bRet;
    }
    // Waypoints for UAV mode
    else if(strcmp(cType, "usv") == 0) {
      bool bRet = parse_waypoint(iSeconKey);
      tiny::send_reply_msg(m_pCurUART, "uav", bRet);
      return bRet;
    }
    // Gyrometer calibration
    else if(strcmp(cType, "gyr") == 0) {
      bool bRet = parse_gyr_cal(iSeconKey);
      tiny::send_reply_msg(m_pCurUART, "gyr", bRet);
      return bRet;
    }
    // PID regulator constants
    else if(strcmp(cType, "pid") == 0) {
      bool bRet = parse_pid_conf(iSeconKey);
      tiny::send_reply_msg(m_pCurUART, "pid", bRet);
      return bRet;
    }
    // set a parameter registered at AP_Param to a certain value
    else if(strcmp(cType, "par") == 0) {
      bool bRet = parse_set_par(iSeconKey);
      tiny::send_reply_msg(m_pCurUART, "par", bRet);
      return bRet;
    }
    // tinycopter doesn't know what to do with this message :(
    else {
      tiny::send_reply_msg(m_pCurUART, "what?", false);
    }
  }

  return false;
}

bool Receiver::read_radio(AP_HAL::UARTDriver *pIn, uint_fast16_t &offset) {
  bool bRet = false;
  uint_fast16_t bytesAvail = pIn->available();
  for(; bytesAvail > 0; bytesAvail--) {
    char c = static_cast<char>(pIn->read() );                 // read next byte

    // New radio message found
    if(c == static_cast<char>(254) ) {                        // this control char is not used for any other symbol
      m_cBuffer[offset] = '\0';                               // null terminator at 8th position
      if(offset != RADIO_MSG_LENGTH) {                        // theoretically a broken message can still be shorter than it should be
        memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0; //so break here if something was wrong
        return false;
      } else {                                                // message has perfect length and stop byte
        bRet = parse_radio();
        if(bRet) {
          m_iSParseTimer_C = m_iSParseTimer;
        }
        memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0; //so break here if something was wrong
      }
    }
    // For the radio, the command string has to be 7 bytes long
    else if(offset < RADIO_MSG_LENGTH) {
      m_cBuffer[offset++] = c;                                // store in buffer and continue until newline
    }
    // If it is longer, but there is no stop byte,
    // it is likely it is a different type of command string
    else {
      return false;
    }
  }

  return bRet;
}

bool Receiver::read_cstring(AP_HAL::UARTDriver *pIn, uint_fast16_t &offset) {
  bool bRet = false;

  uint_fast16_t bytesAvail = pIn->available();
  for(; bytesAvail > 0; bytesAvail--) {
    char c = static_cast<char>(pIn->read() );           // read next byte

    // error check if for a broken radio string, ..
    if(c == static_cast<char>(254) ) {
      memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0;
      return false;
    }

    // .. otherwise it should be regular string
    if(c == '\n' || c == '\r') {                        // new line sign found
      m_cBuffer[offset] = '\0';                         // null terminator
      bRet = parse_cstr();
      if(bRet) {
        m_iSParseTimer_A = m_iSParseTimer;
      }
      memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0;
    }
    // End of regular message reached
    else if(offset < (sizeof(m_cBuffer)-1) ) {
      m_cBuffer[offset++] = c;                          // store in buffer and continue until newline
    }
    // Message seems to be broken, so reset buffer and return
    else {
      memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0;
      return false;
    }
  }

  return bRet;
}

bool Receiver::read_uartX(AP_HAL::UARTDriver *pIn, bool bUseJSON) {
  bool bRet = false;
  m_pCurUART = pIn;
  static uint_fast16_t offset = 0;

  // Try to read very brief command string from radio first, ..
  if(!bUseJSON) {
    bRet = read_radio(pIn, offset);
  }
  // .. otherwise treat it as JSON
  else if(!bRet) {
    bRet = read_cstring(pIn, offset);
  }

  return bRet;
}

// In addition to the attitude control loop,
// reading from the radio or other input sources is the main performance sink.
// This function has the aim to be optimized as much as possible
bool Receiver::try_any() {
  bool bRet = false;

  // Try rcin (PPM radio)
  #if USE_RCIN
  bRet = read_rcin();
  #endif

  // Try WiFi over uartA
  #if USE_UART_A
  if(last_rcin_t32() > RCIN_TIMEOUT && !bRet) {
    bRet = read_uartX(m_pHalBoard->m_pHAL->uartA, true);
    // Disable sensor data output for uartC and enable it for uartA
    if(bRet) {
      m_pTMUartOut->set_arguments(UART_A);
    }
  }
  #endif

  // Try radio (433 or 900 MHz) over uartC
  #if USE_UART_C
  if(last_parse_uartA_t32() > UART_A_TIMEOUT && !bRet) {
    // If currently in other modes, radio could be still helpful
    if(!tiny::chk_fset(m_Waypoint.mode, GPSPosition::GPS_NAVIGATN_F) ) {
      // Disable sensor data output for uartA and enable it for uartC
      #if !DEBUG_OUT // only if the debug output is disabled
      m_pTMUartOut->set_arguments(UART_C);
      #endif
    }
    bRet = read_uartX(m_pHalBoard->m_pHAL->uartC, false);
  }
  #endif

  // Update the time for the last successful parse of a control string
  last_parse_t32();
  return bRet;
}

bool Receiver::read_rcin() {
  if(!m_pHalBoard->m_pHAL->rcin->new_input() ) {
    return false;
  }

  m_pRCPit->set_pwm(m_pHalBoard->m_pHAL->rcin->read(RC_PIT) );
  m_pRCRol->set_pwm(m_pHalBoard->m_pHAL->rcin->read(RC_ROL) );
  m_pRCThr->set_pwm(m_pHalBoard->m_pHAL->rcin->read(RC_THR) );
  m_pRCYaw->set_pwm(m_pHalBoard->m_pHAL->rcin->read(RC_YAW) );

  int_fast16_t pit = m_pRCPit->control_in / 100;
  int_fast16_t rol = m_pRCRol->control_in / 100;
  int_fast16_t thr = m_pRCThr->control_in;
  int_fast16_t yaw = m_pRCYaw->control_in / 100;

  // Small validity check
  if(!check_input(rol, pit, thr, yaw) ) {
    return false;
  }

  // If check was successful we feed the input into or rc array
  m_rgChannelsRC[RC_THR] = thr;
  // dezi degree to degree
  m_rgChannelsRC[RC_PIT] = pit;
  m_rgChannelsRC[RC_ROL] = rol;
  m_rgChannelsRC[RC_YAW] = yaw;

  // Update timers
  m_iSParseTimer = m_iPPMTimer = m_pHalBoard->m_pHAL->scheduler->millis();           // update last valid packet
  return true;
}