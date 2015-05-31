#include "frame.h"


////////////////////////////////////////////////////////////////////////
// Abstract class implementation
////////////////////////////////////////////////////////////////////////
AbsFrame::AbsFrame(AP_HAL::RCOutput *pOut) {
  rol_output = 0;
  pit_output = 0;
  yaw_output = 0;
  thr_output = 0;
  
  m_pRCOut = pOut;
}

void AbsFrame::set_motor_gains(const int_fast16_t iRol, const int_fast16_t iPit, const int_fast16_t iYaw, const int_fast16_t iThr) {
  rol_output = iRol;
  pit_output = iPit;
  yaw_output = iYaw;
  thr_output = iThr;
}

void AbsFrame::set_rol_gain(const int_fast16_t iRol) {
  rol_output = iRol;
}

void AbsFrame::set_pit_gain(const int_fast16_t iPit) {
  pit_output = iPit;
}

void AbsFrame::set_yaw_gain(const int_fast16_t iYaw) {
  yaw_output = iYaw;
}

void AbsFrame::set_thr_gain(const int_fast16_t iThr) {
  thr_output = iThr;
}

void AbsFrame::add_rol_gain(const int_fast16_t iRol) {
  rol_output += iRol;
}

void AbsFrame::add_pit_gain(const int_fast16_t iPit) {
  pit_output += iPit;
}

void AbsFrame::add_yaw_gain(const int_fast16_t iYaw) {
  yaw_output += iYaw;
}

void AbsFrame::add_thr_gain(const int_fast16_t iThr) {
  thr_output += iThr;
}

////////////////////////////////////////////////////////////////////////
// Abstract class implementation
////////////////////////////////////////////////////////////////////////
Frame4X::Frame4X(AP_HAL::RCOutput *pOut) 
  : AbsFrame(pOut) 
{
  m_iFL = 0;
  m_iBL = 0;
  m_iFR = 0;
  m_iBR = 0;
}

void Frame4X::calc_servo_out() {
  m_iFL = thr_output + rol_output + pit_output - yaw_output;
  m_iBL = thr_output + rol_output - pit_output + yaw_output;
  m_iFR = thr_output - rol_output + pit_output + yaw_output;
  m_iBR = thr_output - rol_output - pit_output - yaw_output;
}

void Frame4X::servo_out() {
  m_pRCOut->write(MOTOR_FL, m_iFL);
  m_pRCOut->write(MOTOR_BL, m_iBL);
  m_pRCOut->write(MOTOR_FR, m_iFR);
  m_pRCOut->write(MOTOR_BR, m_iBR);
}
