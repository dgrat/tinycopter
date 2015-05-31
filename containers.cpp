#include "containers.h"


BaroData::BaroData() {
  pressure_pa      = 0;
  altitude_cm      = 0;
  temperature_deg  = 0;
  climb_rate_cms   = 0;
  pressure_samples = 0;
}

GPSData::GPSData() {
  satelites   = 0;
  status      = 0;

  latitude    = 0;
  longitude   = 0;
  altitude_cm = 0;

  gspeed_cms  = 0;

  gcourse_cd  = 0;
  time_week   = 0;
  time_week_s = 0.f;
}

GPSPosition::GPSPosition() {
  latitude    = 0;
  longitude   = 0;
  altitude_cm = 0;

  mode        = GPSPosition::NOTHING_F;
}

GPSPosition::GPSPosition(int_fast32_t lat, int_fast32_t lon, int_fast32_t alt, GPSPosition::UAV_TYPE flag) {
  latitude    = lat;
  longitude   = lon;
  altitude_cm = alt;

  mode        = flag;
}

BattData::BattData() {
  refVoltage_V = -1.f;
  voltage_V    = 0.f;
  current_A    = 0.f;
  consumpt_mAh = 0.f;
  power_W      = 0.f;
}
