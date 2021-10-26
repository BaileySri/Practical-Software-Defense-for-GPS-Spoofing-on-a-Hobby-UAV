#pragma once

#include <cmath>
#include <math.h>
#include <vector>
#include <numeric>

//Consts
const float ACC_ERR = 0.0015;     // (m/s/s)/(sqrt(Hz)) Accelerometer noise (LSM303D) Assuming 100Hz Bandwidth
const float GYRO_ERR = 0.00019;   // (rad/s)/(sqrt(Hz)) Gyro Noise (L3GD20H) Assuming 50Hz Bandwidth
const float FLOW_ERR = 0.05;      // rad/s, In SITL there is a defined FLOW Noise that should be accounted for
const float OF_GYRO_ERR = 0.0007; // (rad/s) Gyro Noise (ICM-20602) Assuming 100Hz Bandwidth
const float LT5_RF_ERR = 0.01;    // meters, Error in rangefinder when less than 5 meters distance
const float GT5_RF_ERR = 0.025;   // meters, Error in rangefinder when greater than or equal to 5 meters distance
const float RMS = 1.73;           // RMS constant useful for tri-axis errors
const float GPS_RATE = 200;       // us, GPS Update rate

//Added to support platform-independent confirmation
#include <cstdint>
#include "../AP_Common/Location.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_InertialSensor/AP_InertialSensor.h"
#include "../AP_AHRS/AP_AHRS.h"
#include "../AP_OpticalFlow/AP_OpticalFlow.h"
#include "../AP_Logger/AP_Logger.h"
#include "../AP_RangeFinder/AP_RangeFinder.h"

class SensorConfirmation{
    public:
        SensorConfirmation()
        {}
        ~SensorConfirmation()
        {}
        //Setup delays
        uint32_t DELAY = 45000000U;       // Start-Up delay to allow sensors to settle
        uint32_t BIAS_DELAY = 30000000U;  // Delay after Start-Up to allow biasing (30 seconds)
        
        //----typedef----//
        typedef Vector3f NED;
        typedef Vector3f BF;

        //----   Helper Functions  ----//
        static bool confirm(const float a, const float a_err, const float b, const float b_err, bool);
        static Vector3f abs(const Vector3f &x);

        //----   Structs  ----//
        struct Accel
        {
            NED Readings;       //m/s/s, Accelerometer readings rotated to NED
            NED Velocity;       //m/s
            float Error;        //m/s
            uint32_t Timestamp; //us

            bool update(const AP_InertialSensor *frontend, NED bias)
            {
                uint32_t dT = frontend->get_last_update_usec() - Timestamp; //us
                if (dT > 0)
                {
                    NED newReading = (AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned() * frontend->get_accel()) - bias;
                    //Trapezoidal Integration
                    Velocity += (((newReading + Readings) / 2.0F) * dT) / 1000000;
                    Readings = newReading;
                    Error += (ACC_ERR * RMS * dT) / 1000000; // Error in acceleration, dT is in us and multiply by time for velocity error
                    Timestamp += dT;
                    return true;
                }
                return false;
            }

            void reset()
            {
                Readings.zero();
                Velocity.zero();
                Error = 0;
                //Important note, after every reset if time matters assign the old timestamp to the reset sensor
                Timestamp = 0;
            }

            Accel &operator=(const Accel &rhs)
            {
                Readings = rhs.Readings;
                Velocity = rhs.Velocity;
                Timestamp = rhs.Timestamp;
                Error = rhs.Error;
                return *this;
            }
        };
        struct Gyro
        {
            BF Readings; //x = Roll, y = Pitch, z = Yaw
            float Error;
            uint32_t Timestamp;
            uint32_t TimeContained;

            bool update(const AP_InertialSensor *frontend)
            {
                uint32_t dT = frontend->get_last_update_usec() - Timestamp; //us
                if (dT > 0)
                {
                    Readings = frontend->get_gyro();
                    Error += GYRO_ERR;
                    Timestamp += dT;
                    TimeContained += dT;
                    return true;
                }
                return false;
            }

            void reset()
            {
                Readings.zero();
                Error = 0;
                //Important note, after every reset if time matters assign the old timestamp to the reset sensor
                Timestamp = 0;
                TimeContained = 0;
            }

            Gyro &operator=(const Gyro &rhs)
            {
                Readings = rhs.Readings;
                Timestamp = rhs.Timestamp;
                Error = rhs.Error;
                TimeContained = rhs.TimeContained;
                return *this;
            }
        };
        struct Mag
        {
            Vector3f Readings; //milligauss

            void reset()
            {
                Readings.zero();
            }
        };
        struct GPS
        {
            int32_t Latitude;  //Degrees 1E-7
            int32_t Longitude; //Degrees 1E-7
            int32_t Altitude;
            NED Airspeed; //m/s
            float Sacc;   //m/s GPS 3D RMS Speed Accuracy
            float Hacc;   //m GPS 3D RMS Horizontal Position Accuracy
            float Vacc;   //m GPS 3D RMS Vertical Position Accuracy
            float Yaw;    //Degrees, North is considered 0 degrees moving Clock-Wise
            float Yaw_Error;
            uint32_t Yaw_TimeMs;
            uint32_t Timestamp;     //ms
            uint32_t lastTimestamp; //ms

            void update_location(const Location &rhs)
            {
                Latitude = rhs.lat;
                Longitude = rhs.lng;
                Altitude = rhs.alt;
            }

            bool update(const AP_GPS *frontend, GPS &prevGps)
            {
                uint32_t dT = frontend->last_fix_time_ms() - Timestamp;
                if (dT > 0)
                {
                    //Save current GPS data to previous GPS data
                    prevGps = *this;

                    //Location, Velocity, Heading
                    update_location(frontend->location());
                    Airspeed = frontend->velocity();
                    if (!frontend->gps_yaw_deg(Yaw, Yaw_Error, Yaw_TimeMs))
                    {
                        Yaw = 1000;
                        Yaw_Error = -1;
                    }

                    //Timestamps
                    lastTimestamp = Timestamp;
                    Timestamp += dT;

                    //Accuracy Measurements
                    if (!frontend->speed_accuracy(Sacc))
                    {
                        Sacc = -1;
                    }
                    if (!frontend->vertical_accuracy(Vacc))
                    {
                        Vacc = -1;
                    }
                    if (!frontend->horizontal_accuracy(Hacc))
                    {
                        Hacc = -1;
                    }
                    return true;
                }
                return false;
            }

            void reset()
            {
                Latitude = 0;
                Longitude = 0;
                Altitude = 0;
                Airspeed.zero();
                Yaw = -1;
                Yaw_Error = -1;
                Yaw_TimeMs = 0;
                //Important note, after every reset if time matters assign the old timestamp to the reset sensor
                Timestamp = 0;
            }
        };
        struct RF
        {
            float tcReading;  //meters, Tilt compensated but not filtered readings
            float Timestamp;  //ms, Timestamp of last healthy RF reading
            float RF_Vel;     //m/s, Velocity from rangefinder measurements
            float RF_Vel_Err; //m/s, Possible error in velocity
            float RangeErr;   //meters, Error in RF reading
            LowPassFilterFloat rf_filt;

            void update()
            {
                const uint32_t &newTimestamp = RangeFinder::get_singleton()->last_reading_ms(ROTATION_PITCH_270);
                const uint16_t &newReading = RangeFinder::get_singleton()->distance_cm_orient(ROTATION_PITCH_270);
                float dT = newTimestamp - Timestamp;
                if (dT > 0)
                {
                    // Assuming we want to use tilt compensation on rangefinder
                    const float tilt_correction = MAX(0.707f, AP_AHRS::get_singleton()->get_rotation_body_to_ned().c.z);
                    float prevReading = rf_filt.get();
                    tcReading = tilt_correction * newReading;
                    rf_filt.apply(tcReading, 0.05f);
                    float curReading = rf_filt.get();
                    RF_Vel = (curReading - prevReading) / (dT / 1000);
                    RF_Vel_Err = ((curReading >= 500 ? GT5_RF_ERR : LT5_RF_ERR) + RangeErr) / (dT / 1000);
                    RangeErr = curReading >= 500 ? GT5_RF_ERR : LT5_RF_ERR;
                    Timestamp += dT;
                }
            }

            void reset()
            {
                tcReading = 0;
                RF_Vel = 0;
                RF_Vel_Err = 0;
                //Important note, after every reset if time matters assign the old timestamp to the reset sensor
                Timestamp = 0;
                RangeErr = 0;
                rf_filt.reset();
                rf_filt.set_cutoff_frequency(0.5f);
            }
        };
        struct OF
        {
            LowPassFilterVector2f flow_filter; // LowPassFilter for flowrate
            Vector2f FlowRate;  // rad/s, this is the angular rate calculated by the camera, i.e., HereFlow
            Vector2f BodyRate;  // rad/s, in SITL this is the same as the gyroscope. Pitch Roll
            uint32_t Timestamp; // ms, Timestamp of last OF Update
            float RateErr;      // rad/s, Error in angular rate. Depends on sensor but HereFlow has a separate Gyroscope
            BF VelBF;           //cm/s, Velocity in Body Frame (OF, RF, Barometer)
            NED VelNED;         //cm/s, velocity in NED frame (Mag, OF, RF)
            NED Err;            //cm/s. Error in velocity in NED frame

            bool update(const OpticalFlow *frontend, const RF &currRF)
            {
                uint32_t dT = frontend->last_update() - Timestamp; //ms
                if (dT > 0)
                {
                    FlowRate = frontend->flowRate();
                    BodyRate = frontend->bodyRate();
                    flow_filter.apply(FlowRate - BodyRate);
                    RateErr += OF_GYRO_ERR;
                    Timestamp += dT;
                    VelBF = Vector3f{tanf(flow_filter.get()[1]) * (currRF.rf_filt.get()), //m/s, Front
                                    tanf(flow_filter.get()[0]) * (currRF.rf_filt.get()), //m/s, Right
                                    currRF.RF_Vel};                                               //m/s, Down
                    VelNED = (AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned() * VelBF);   //Rotated BF to NED

                    BF ErrBF = Vector3f{tanf((flow_filter.get()[1]) + RateErr + FLOW_ERR) * ((currRF.rf_filt.get()) + currRF.RangeErr),
                                        tanf((flow_filter.get()[0]) + RateErr + FLOW_ERR) * ((currRF.rf_filt.get()) + currRF.RangeErr),
                                        currRF.RF_Vel + currRF.RF_Vel_Err};
                    Err = (AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned() * abs(ErrBF - VelBF));
                    return true;
                }
                return false;
            }

            void reset(void)
            {
                flow_filter.reset();
                // Copter and Plane mainloop operate at 400Hz, assume that is standard here
                // Refer to mode_flowhold.cpp for further information
                flow_filter.set_cutoff_frequency(400, 5);
                FlowRate.zero();
                BodyRate.zero();
                //Important note, after every reset if time matters assign the old timestamp to the reset sensor
                Timestamp = 0;
                RateErr = 0;
                VelBF.zero();
                VelNED.zero();
                Err.zero();
            }

            OF &operator=(const OF &rhs)
            {
                flow_filter.reset(rhs.flow_filter.get());
                flow_filter.set_cutoff_frequency(rhs.flow_filter.get_cutoff_freq());
                FlowRate = rhs.FlowRate;
                BodyRate = rhs.BodyRate;
                Timestamp = rhs.Timestamp;
                RateErr = rhs.RateErr;
                VelBF = rhs.VelBF;
                VelNED = rhs.VelNED;
                Err = rhs.Err;
                return *this;
            }
        };

        //Sensor Data
        struct
        {
            Accel currAccel; //Acceleration data within GPS update frame
            Accel nextAccel; //Acceleration data that happens after expected GPS update
            Gyro currGyro;   //Roll,Pitch,Yaw (Rates in radians/sec)
            Gyro nextGyro;   //Roll,Pitch,Yaw (Rates in radians/sec)
            GPS currGps;     //Most recent GPS reading
            GPS prevGps;     //Previous GPS reading
            OF prevOF;       //Previous Optical Flow Data
            OF currOF;       //Current Optical Flow Data
            OF nextOF;       //Optical Flow Data that occurs after expected GPS update
            RF rangefinder;  //Rangefinder Data

            //Data for Accelerometer confirming to Optical Flow
            Accel pOFAccel;
            Accel cOFAccel;
            Accel nOFAccel;
            OF pOF;
            OF cOF;
        } sensors;

        //Confirmation Data
        struct
        {
            bool init = false; //Initialized Structs
            bool gpsAvail;     //GPS Data has been updated
            bool ofAvail;      // OF Data has been updated
        } framework;

        //Bias Data
        struct
        {
            bool biased = false; //True when bias has been determined
            uint32_t Timestamp;  //Used to determine when readings change for bias
            uint32_t count = 0;  //Number of readings used in bias
            NED AccBias;         //Bias for NED acceleration
        } bias;

        //----General Functions----//
        void initialize();
        void update();
        bool run();
        void alert();
        void debug();
        void confirmation();

        //----Confirmation Functions----//
        // Confirm change in velocity of GPS and Accelerometer
        bool AccGPS();
        // Confirm change in velocity of Accelerometer and OF
        bool AccOF();
        // Confirm velocity of GPS and Optical Flow Sensor
        bool GpsOF();
        // Confirm ground course based on GPS movement and Magnetometer heading
        bool GpsMagGC();
        // Confirm ground course based on GPS movement and Optical Flow movement with Magnetometer for rotation
        bool GpsOFGC();
        // Confirm ground course based on GPS movement and Accelerometer movement with Magnetometer for rotation
        bool AccGpsGC();
        // Confirm ground course based on Accelerometer and Optical Flow movement with Magnetometer for rotation
        bool AccOFGC();
    private:
};