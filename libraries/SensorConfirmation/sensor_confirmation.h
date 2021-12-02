#pragma once

//Libraries
#include <cstdint>
#include "../AP_Common/Location.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_InertialSensor/AP_InertialSensor.h"
#include "../AP_AHRS/AP_AHRS.h"
#include "../AP_OpticalFlow/AP_OpticalFlow.h"
#include "../AP_Logger/AP_Logger.h"
#include "../AP_RangeFinder/AP_RangeFinder.h"

//Consts
const float ACC_ERR = (150) * (9.80665) * 4 / (1E6); // (m/s/s)/sqrt(Hz), noise density * m/s/s / g * nsamples.
                                                     // still need to multiply by sqrt(sampling frequency)
                                                     // For non-SITL builds the nsamples will likely need to be adjusted
const float GYRO_ERR = (0.011) * (M_PI / 180.0F) * 4;// (rad/s)/(sqrt(Hz)), noise density * pi / 180 * nsamples
                                                     // still need to multiply by sqrt(sampling frequency)
                                                     // For non-SITL builds the nsamples will likely need to be adjusted
const float FLOW_ERR = 0.05;       // rad/s, In SITL there is a defined FLOW Noise that should be accounted for
const float OF_GYRO_ERR = (0.004) * (M_PI / 180.0F) * (sqrtf(10.0F)); // (rad/s)
const float LT5_RF_ERR = 0.01;     // meters, Error in rangefinder when less than 5 meters distance
const float GT5_RF_ERR = 0.025;    // meters, Error in rangefinder when greater than or equal to 5 meters distance
const float RMS = 1.73;            // RMS constant useful for tri-axis errors
const float GPS_RATE = 200;        // ms, GPS Update rate
const Vector3f GRAVITY_NED = Vector3f{0,0, 9.80665F}; // m/s/s, 3D Vector of gravity

class SensorConfirmation{
    public:
        SensorConfirmation()
        {}
        ~SensorConfirmation()
        {}
        
        //----typedef----//
        typedef Vector3f NED;
        typedef Vector3f BF;

        //----   Helper Functions  ----//
        static bool confirm(const float a, const float a_err, const float b, const float b_err, bool);
        static Vector3f abs(const Vector3f &x);
        static Vector2f abs(const Vector2f &x);

        //----   Structs  ----//
        struct Accel
        {
            NED Readings;       //m/s/s, Accelerometer readings rotated to NED
            NED Velocity;       //m/s, Velocity in NED frame
            float Error;        //m/s, Single axis error, Multiply by RMS for Net
            uint32_t Timestamp; //us

            bool update(const AP_InertialSensor *frontend)
            {
                uint32_t dT = frontend->get_last_update_usec() - Timestamp; //us
                if (dT > 0)
                {
                    uint8_t primary_accel = frontend->get_primary_accel();
                    NED newReading = (AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned() * frontend->get_accel(primary_accel)) + GRAVITY_NED;
                    //Trapezoidal Integration
                    Velocity += (((newReading + Readings) / 2.0F) * dT) / 1000000;
                    Readings = newReading;
                    //Pull accelerometer sampling rate for error calculation
                    uint16_t sample_rate = frontend->get_accel_rate_hz(primary_accel);
                    Error += (ACC_ERR * sqrtf(sample_rate) * dT) / (float)1000000; // Error in acceleration, dT is in us and multiply by time for velocity error
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
                    uint8_t primary_gyro = frontend->get_primary_gyro();
                    Readings = frontend->get_accel(primary_gyro);
                    Error += GYRO_ERR * sqrtf(frontend->get_gyro_rate_hz(primary_gyro));
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
            float RangeErr;   //meters, Error in RF reading
            LowPassFilterFloat rf_filt; //cm, filtered tilt compensated rangefinder readings

            void update()
            {
                const uint32_t &newTimestamp = RangeFinder::get_singleton()->last_reading_ms(ROTATION_PITCH_270);
                const uint16_t &newReading = RangeFinder::get_singleton()->distance_cm_orient(ROTATION_PITCH_270);
                float dT = newTimestamp - Timestamp;
                if (dT > 0)
                {
                    // Assuming we want to use tilt compensation on rangefinder
                    const float tilt_correction = MAX(0.707f, AP_AHRS::get_singleton()->get_rotation_body_to_ned().c.z);
                    tcReading = tilt_correction * newReading;
                    rf_filt.apply(tcReading, 0.05f);                
                    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                        RangeErr = 0.05; //SITL error isn't set to LLv3 documentation 
                    #else
                        float curReading = rf_filt.get();
                        RangeErr = curReading >= 500 ? GT5_RF_ERR : LT5_RF_ERR;
                    #endif
                    Timestamp += dT;
                }
            }

            void reset()
            {
                tcReading = 0;
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
            Vector2f VelNE;     // m/s, Velocity in EN frame (North/East)
            Vector2f Err;     // m/s. Error in velocity in NE frame

            bool update(const OpticalFlow *frontend, const RF &currRF)
            {
                uint32_t dT = frontend->last_update() - Timestamp; //ms
                if (dT > 0)
                {
                    FlowRate = frontend->flowRate();
                    BodyRate = frontend->bodyRate();
                    Vector2f RawRate = FlowRate-BodyRate;
                    flow_filter.apply(Vector2f{RawRate[1],RawRate[0]}); //Reverse order to be in NE frame
                    Timestamp += dT;
                    // SITL has a weird scaling factor based on the rotation matrix, refer to AP_OpticalFlow_SITL::update() rotmat
                    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                        float scaling_factor = AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned()[2][2];
                    #else
                        float scaling_factor = 1;
                    #endif

                    VelNE = AP_AHRS::get_singleton()->body_to_earth2D(flow_filter.get() * (currRF.rf_filt.get()/100.0F) / scaling_factor);   //Rotated BF to NED
                    // Error Propagation of Rangefinder, Scaling Factor, and Optical Flow
                    Vector2f ErrFR = Vector2f{sqrtf(sq(currRF.rf_filt.get()/100.0F / scaling_factor) * sq(FLOW_ERR + OF_GYRO_ERR) + 
                                                    sq(flow_filter.get()[0] / scaling_factor) * sq(currRF.RangeErr) + 
                                                    sq(flow_filter.get()[0] * currRF.rf_filt.get()/100.0F) * sq(0.05)),
                                              sqrtf(sq(currRF.rf_filt.get()/100.0F / scaling_factor) * sq(FLOW_ERR + OF_GYRO_ERR) + 
                                                    sq(flow_filter.get()[1] / scaling_factor) * sq(currRF.RangeErr) + 
                                                    sq(flow_filter.get()[1] * currRF.rf_filt.get()/100.0F / sq(scaling_factor)) * sq(0.05))};
                    Err = AP_AHRS::get_singleton()->body_to_earth2D(ErrFR);
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
                VelNE.zero();
                Err.zero();
            }

            OF &operator=(const OF &rhs)
            {
                flow_filter.reset(rhs.flow_filter.get());
                flow_filter.set_cutoff_frequency(rhs.flow_filter.get_cutoff_freq());
                FlowRate = rhs.FlowRate;
                BodyRate = rhs.BodyRate;
                Timestamp = rhs.Timestamp;
                VelNE = rhs.VelNE;
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

        //----General Functions----//
        void initialize();
        void update();
        bool run();
        void alert();
        void debug();
        void confirmation();
        // The max velocity allowed before confirmation fails
        float NetGpsLimit() const;
        // The max flowrate with bodyrate allowed before confirmation fails
        float NetOFLimit() const;
        // Depending on selected mode return the limited GPS value
        //  1: Acc
        //  2: OF
        //  3: Mag 
        //  4: Acc/OF
        //  5: Acc/Mag
        //  6: OF/Mag
        float GCGpsLimit(const int mode) const;

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