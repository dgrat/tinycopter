#include <AP_Math.h>
#include <AP_InertialNav.h>
#include <math.h>

#include "navigation.h"
#include "device.h"
#include "receiver.h"
#include "exceptions.h"
#include "arithmetics.h"

#define M_2PI               (2.f * M_PI)
#define M_PERIMETER_EARTH_M (40074000.f)
#define M_RADIUS_EARTH_M    (M_PERIMETER_EARTH_M / M_2PI)
#define M_AP_INT2FLOAT_DEG  (10000000.f)


UAVNav::UAVNav(Device *pDev, Receiver *pRecv, Exception *pExcp) {
  m_pHalBoard      = pDev;
  m_pReceiver      = pRecv;
  m_pExeption      = pExcp;

  m_t32YawTimer    = m_pHalBoard->m_pHAL->scheduler->millis();

  m_fTargetYaw_deg = 0.f;
  m_fTargetPit_deg = 0.f;
}

/*
 * Geographic calculations
 */
float UAVNav::width_of_merid_m(const float fLat_deg) const {
  return (M_2PI * M_RADIUS_EARTH_M * cos(ToRad(fLat_deg) ) ) / 360.f;
}

float UAVNav::dist_2_equat_m(const float fLat_deg) const {
  return (M_PERIMETER_EARTH_M / 360.f) * fLat_deg;
}

float UAVNav::dist_2_greenw_m(const float fLat_deg, const float fLon_deg) const {
  return width_of_merid_m(fLat_deg) * fLon_deg;
}

float UAVNav::calc_error_deg() {
  Location curLocation;
  bool bLocOK = m_pHalBoard->m_pAHRS->get_position(curLocation);
  float fLatHome_deg = static_cast<float>(curLocation.lat) / M_AP_INT2FLOAT_DEG;
  float fLonHome_deg = static_cast<float>(curLocation.lng) / M_AP_INT2FLOAT_DEG;

  float fLatTarg_deg = static_cast<float>(m_pReceiver->get_waypoint()->latitude) / M_AP_INT2FLOAT_DEG;
  float fLonTarg_deg = static_cast<float>(m_pReceiver->get_waypoint()->longitude) / M_AP_INT2FLOAT_DEG;

  float fXHome = dist_2_greenw_m(fLatHome_deg, fLonHome_deg);
  float fYHome = dist_2_equat_m(fLatHome_deg);

  float fXTarg = dist_2_greenw_m(fLatTarg_deg, fLonTarg_deg);
  float fYTarg = dist_2_equat_m(fLatTarg_deg);

  return tiny::delta180_f(atan2(fXTarg - fXHome, fYTarg - fYHome) , m_pHalBoard->get_comp_deg() );
}

int_fast16_t UAVNav::calc_yaw() {
  // TODO Implement me


  // Anneal to correct yaw
  return static_cast<int_fast16_t>(m_fTargetYaw_deg);
}

int_fast16_t UAVNav::calc_pitch() {
  // TODO Implement me

  return m_fTargetPit_deg;
}
