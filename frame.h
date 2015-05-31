#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>
#include <stddef.h>

#include "config.h"

/*
 * Abstract class:
 * Basic functions for all? copter-frames
 */
class AbsFrame { 
protected:
  int_fast16_t rol_output;
  int_fast16_t pit_output;
  int_fast16_t yaw_output;
  int_fast16_t thr_output;
  
  AP_HAL::RCOutput* m_pRCOut;
  
public:
  AbsFrame(AP_HAL::RCOutput *);
  
  void set_motor_gains(const int_fast16_t rol, const int_fast16_t pit, const int_fast16_t yaw, const int_fast16_t thr);
  void set_rol_gain(const int_fast16_t);
  void set_pit_gain(const int_fast16_t);
  void set_yaw_gain(const int_fast16_t);
  void set_thr_gain(const int_fast16_t);
  
  void add_rol_gain(const int_fast16_t);
  void add_pit_gain(const int_fast16_t);
  void add_yaw_gain(const int_fast16_t);
  void add_thr_gain(const int_fast16_t);
  
  virtual void calc_servo_out() = 0;
  virtual void servo_out() = 0;
};

/*
 * This class implements the motor/servo output for 4X quadcopter frames
 */
class Frame4X : public AbsFrame {
private:
  int_fast16_t m_iFL;
  int_fast16_t m_iBL;
  int_fast16_t m_iFR;
  int_fast16_t m_iBR;
  
public:
  Frame4X(AP_HAL::RCOutput *);
  
  virtual void calc_servo_out();
  virtual void servo_out();
};

#endif