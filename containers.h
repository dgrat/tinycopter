#ifndef CONTAINER_h
#define CONTAINER_h

#include <stdint.h>
#include <stddef.h>


// barometer data container
struct BaroData {
  float         pressure_pa;
  int_fast32_t  altitude_cm;
  float         temperature_deg;
  int_fast16_t  climb_rate_cms;
  uint_fast8_t  pressure_samples;

  BaroData();
};

// gps data container
struct GPSData {
  uint_fast8_t  satelites;
  int_fast16_t  status;

  int_fast32_t  latitude;     // in degrees * 10,000,000
  int_fast32_t  longitude;    // in degrees * 10,000,000
  int_fast32_t  altitude_cm;  // altitude in cm

  uint_fast32_t gspeed_cms;   // ground speed in cm/sec

  int_fast32_t  gcourse_cd;   // ground course in degree
  uint_fast16_t time_week;
  float  time_week_s;

  GPSData();
};

struct GPSPosition {
  enum UAV_TYPE {
    NOTHING_F = 1 << 0,
    HLD_ALTITUDE_F = 1 << 1,
    GPS_NAVIGATN_F = 1 << 2,
    CONTRLD_DOWN_F = 1 << 3
  };

  int_fast32_t latitude;     // in degrees * 10,000,000
  int_fast32_t longitude;    // in degrees * 10,000,000
  int_fast32_t altitude_cm;  // altitude in cm
  UAV_TYPE     mode;

  GPSPosition();
  GPSPosition(int_fast32_t, int_fast32_t, int_fast32_t, GPSPosition::UAV_TYPE);
};

// battery monitor
struct BattData {
  // Reference voltage of the battery (e.g. on startup)
  float refVoltage_V;
  float voltage_V;
  float current_A;
  float power_W;
  float consumpt_mAh;

  BattData();
};

#endif
