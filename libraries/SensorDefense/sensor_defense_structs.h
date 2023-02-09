#ifndef SENSOR_DEFENSE_STRUCTS_H
#define SENSOR_DEFENSE_STRUCTS_H

#include "../AP_AHRS/AP_AHRS.h"
#include "../AP_Baro/AP_Baro.h"
#include "../AP_Common/Location.h"
#include "../AP_Compass/AP_Compass.h"
#include "../AP_InertialSensor/AP_InertialSensor.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_OpticalFlow/AP_OpticalFlow.h"
#include "../AP_RangeFinder/AP_RangeFinder.h"
#include "AP_Math/matrix3.h"
#include <cstdint>

// Consts
const float RMS = 1.73;         // RMS constant useful for tri-axis errors
const float GPS_RATE = 200;     // ms, GPS Update rate
const Vector3f GRAVITY_NED = Vector3f{0, 0, 9.80665F}; // m/s/s, 3D Vector of gravity
//    IMPORTANT: Change below values for your implementation, i.e., observed max disagreement in flight or simulation
//               current values are relevant to UAV used for data collection and testing for the USENIX submission.
//               If running simulation consider running a simulated flight for error margins and plugging in here.
//                NOTE: Live testing showed the OF behaving significantly differently than simulated
const float BENIGN_VEL = 7.348; //Max disagreement in velocity found in Benign Flight 1 - 4
const float BENIGN_ANG = 14; //Max disagreement found in yaw in Benign Flight 1- 4
const float GPS_MIN_SPD = 1.5; //Minimum GPS airspeed before we use ground course, empirically derived

// Typedef for clarity
typedef Vector3f NED;
typedef Vector3f BF;

// Helper functions
inline Vector3f lpf(const Vector3f &reading, const Vector3f &value, float alpha = 0.5) {
  return(value + ((reading - value) * alpha));
};
inline Vector2f lpf(const Vector2f &reading, const Vector2f &value, float alpha = 0.5) {
  return(value + ((reading - value) * alpha));
};

// Structs containing data
struct Accel {
  BF Raw; // m/s/s, Unfiltered accelerometer readings, Scaled and Rotated into BF
  Matrix3f rot;       // Rotation matrix used by AHRS
  NED Readings;       // m/s/s, Accelerometer readings rotated to NED
  NED Velocity;       // m/s, Velocity in NED frame
  uint32_t Timestamp; // us

  bool update(const AP_InertialSensor *frontend) {
    uint32_t newTimestamp = frontend->get_last_update_usec(); // us

    if ((newTimestamp - Timestamp) > 0) {
      uint8_t primary_accel = frontend->get_primary_accel();
      Raw = frontend->get_accel_raw(primary_accel);
      rot = AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned();
      NED newReading = (rot * frontend->get_accel(primary_accel)) + GRAVITY_NED;

      // Trapezoidal Integration
      Velocity +=
          (((newReading + Readings) / 2.0F) * (newTimestamp - Timestamp)) /
          1000000;
      Readings = newReading;
      Timestamp = newTimestamp;
      return true;
    }
    return false;
  }

  void reset(float newTs = 0) {
    rot.zero();
    Raw.zero();
    Readings.zero();
    Velocity.zero();
    Timestamp = newTs;
  }

  Accel &operator=(const Accel &rhs) {
    rot = rhs.rot;
    Raw = rhs.Raw;
    Readings = rhs.Readings;
    Velocity = rhs.Velocity;
    Timestamp = rhs.Timestamp;
    return *this;
  }
};

struct Gyro {
  BF Raw;      // x = Roll, y = Pitch, z = Yaw
  BF Readings; // x = Roll, y = Pitch, z = Yaw
  BF TrapInt;  // Trapezoid Integration of each axis, delta degrees
  uint32_t Timestamp; // us
  uint32_t TimeContained;

  bool update(const AP_InertialSensor *frontend) {
    uint32_t newTimestamp = frontend->get_last_update_usec(); // us

    if ((newTimestamp - Timestamp) > 0) {
      uint8_t primary_gyro = frontend->get_primary_gyro();
      Raw = frontend->get_gyro_raw(primary_gyro);
      BF newReadings = frontend->get_gyro(primary_gyro);
      TrapInt = (newReadings + Readings) * ((float(newTimestamp) - float(Timestamp)) / 2000.0F); 
      Readings = newReadings;
      TimeContained += (newTimestamp - Timestamp);
      Timestamp = newTimestamp;
      return true;
    }
    return false;
  }

  void reset(float newTs = 0) {
    Raw.zero();
    Readings.zero();
    Timestamp = newTs;
    TimeContained = 0;
  }

  Gyro &operator=(const Gyro &rhs) {
    Raw = rhs.Raw;
    Readings = rhs.Readings;
    Timestamp = rhs.Timestamp;
    TimeContained = rhs.TimeContained;
    return *this;
  }
};

struct Mag {
  uint32_t Timestamp; // us, timestamp of last magnetometer reading
  Vector3f Readings;  // milligauss

  void update() {
    Compass *mag = Compass::get_singleton();
    uint32_t newTimestamp = mag->last_update_usec();

    if ((newTimestamp - Timestamp) > 0) {
      Readings = mag->get_reading();
      Timestamp = newTimestamp;
    }
  }

  void reset(float newTs = 0) {
    Timestamp = newTs;
    Readings.zero();
  }
};

struct GPS {
  int32_t Latitude;  // Degrees 1E-7
  int32_t Longitude; // Degrees 1E-7
  int32_t Altitude;
  NED Airspeed; // m/s
  float Sacc;   // m/s GPS 3D RMS Speed Accuracy
  float Hacc;   // m GPS 3D RMS Horizontal Position Accuracy
  float Vacc;   // m GPS 3D RMS Vertical Position Accuracy
  float Yaw;    // Degrees, North is considered 0 degrees moving Clock-Wise
  float Yaw_Error;
  uint32_t Yaw_TimeMs;
  uint32_t Timestamp;     // ms
  uint32_t lastTimestamp; // ms

  void update_location(const Location &rhs) {
    Latitude = rhs.lat;
    Longitude = rhs.lng;
    Altitude = rhs.alt;
  }

  bool update(const AP_GPS *frontend, GPS &prevGps) {
    uint32_t newTimestamp = frontend->last_fix_time_ms();

    if ((newTimestamp - Timestamp) > 0) {
      // Save current GPS data to previous GPS data
      prevGps = *this;

      // Location, Velocity, Heading
      update_location(frontend->location());
      Airspeed = frontend->velocity();
      if (!frontend->gps_yaw_deg(Yaw, Yaw_Error, Yaw_TimeMs)) {
        Yaw = 1000;
        Yaw_Error = -1;
      }

      // Timestamps
      lastTimestamp = Timestamp;
      Timestamp = newTimestamp;

      // Accuracy Measurements
      if (!frontend->speed_accuracy(Sacc)) {
        Sacc = -1;
      }
      if (!frontend->vertical_accuracy(Vacc)) {
        Vacc = -1;
      }
      if (!frontend->horizontal_accuracy(Hacc)) {
        Hacc = -1;
      }
      return true;
    }
    return false;
  }

  void reset(float newTs = 0) {
    Latitude = 0;
    Longitude = 0;
    Altitude = 0;
    Airspeed.zero();
    Yaw = -1;
    Yaw_Error = -1;
    Yaw_TimeMs = 0;
    Timestamp = newTs;
  }
};

struct RF {
  float Timestamp; // ms, Timestamp of last healthy RF reading
  LowPassFilterFloat rf_filt;  // cm, filtered tilt compensated rangefinder readings
  float rf_raw; // cm, raw rf reading

  void update() {
    // Update Timestamp (ms)
    const uint32_t &newTimestamp = RangeFinder::get_singleton()->last_reading_ms(ROTATION_PITCH_270);
    if ((newTimestamp - Timestamp) > 0) {
      const uint16_t &newReading = RangeFinder::get_singleton()->distance_cm_orient(ROTATION_PITCH_270);
      // Assuming we want to use tilt compensation on rangefinder
      rf_raw = newReading;
      rf_filt.apply(rf_raw, 0.05f);
      Timestamp = newTimestamp;
    }
  }

  void reset(float newTs = 0) {
    rf_raw = 0;
    Timestamp = newTs;
    rf_filt.reset();
    rf_filt.set_cutoff_frequency(0.5f);
  }
};

struct OF {
  Vector2f flow_filtered; // Low Pass Filter for flowrate
  Vector2f FlowRate; // rad/s, this is the angular rate calculated by the
                     // camera, i.e., HereFlow
  Vector2f BodyRate; // rad/s, in SITL this is the same as the gyroscope. Pitch Roll
  uint32_t Timestamp; // ms, Timestamp of last OF Update
  Vector2f VelNE;     // m/s, Velocity in EN frame (North/East)

  bool update(const OpticalFlow *frontend, const RF &currRF) {
    uint32_t newTimestamp = frontend->last_update(); // ms

    if ((newTimestamp - Timestamp) > 0) {
      FlowRate = frontend->flowRate(); // FlowRate is scaled and yawed according
                                       // to parameters
      BodyRate = frontend->bodyRate();
      Vector2f RawRate = FlowRate - BodyRate;
      flow_filtered = lpf(RawRate, flow_filtered, 0.2);
      Timestamp = newTimestamp;

// SITL has a weird scaling factor based on the rotation matrix, refer to
// AP_OpticalFlow_SITL::update() rotmat
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
      float scaling_factor = AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned()[2][2];
#else
      float scaling_factor = 1;
#endif

      Vector2f VelBF = flow_filtered* (currRF.rf_filt.get() / 100.0F) / scaling_factor;
      VelNE = AP_AHRS::get_singleton()->body_to_earth2D(VelBF); // Rotated BF to NED
      return true;
    }
    return false;
  }

  void reset(float newTs = 0) {
    flow_filtered.zero();
    FlowRate.zero();
    BodyRate.zero();
    Timestamp = newTs;
    VelNE.zero();
  }

  OF &operator=(const OF &rhs) {
    flow_filtered = rhs.flow_filtered;
    FlowRate = rhs.FlowRate;
    BodyRate = rhs.BodyRate;
    Timestamp = rhs.Timestamp;
    VelNE = rhs.VelNE;
    return *this;
  }
};

struct Baro {
  uint32_t Timestamp;   // ms, Timestamp of last healthy RF reading
  float Altitude;    // meters, Relative to barometer calibration
  float Pressure;    // Pascal
  float Temperature; // Celsius

  void update() {
    const AP_Baro *baro = AP_Baro::get_singleton();
    const uint32_t &newTimestamp = baro->get_last_update(); // ms

    if ((newTimestamp - Timestamp) > 0) {
      Pressure = baro->get_pressure();
      Temperature = baro->get_temperature();
      Altitude = baro->get_altitude();
      Timestamp = newTimestamp;
    }
  }

  void reset(float newTs = 0) {
    Timestamp = newTs;
    Altitude = 0;
    Pressure = 0;
    Temperature = 0;
  }
};

#endif