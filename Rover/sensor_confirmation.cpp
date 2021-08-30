//PADLOCK
//Sensor Confirmation source code
//7/12/2021

#include "Rover.h"
#include <cmath>
#include <math.h>
#include <vector>
#include <numeric>

//Constant declarations
static uint32_t DELAY = 45000000U;       // Start-Up delay to allow sensors to settle
static uint32_t BIAS_DELAY = 90000000U;  // Delay after Start-Up to allow biasing
static const float ACC_ERR = 0.0015;     // (m/s/s)/(sqrt(Hz)) Accelerometer noise (LSM303D) Assuming 100Hz Bandwidth
static const float GYRO_ERR = 0.00019;   // (rad/s)/(sqrt(Hz)) Gyro Noise (L3GD20H) Assuming 50Hz Bandwidth
static const float FLOW_ERR = 0.05;      // rad/s, In SITL there is a defined FLOW Noise that should be accounted for
static const float OF_GYRO_ERR = 0.0007; // (rad/s) Gyro Noise (ICM-20602) Assuming 100Hz Bandwidth
static const float LT5_RF_ERR = 0.01;    // meters, Error in rangefinder when less than 5 meters distance
static const float GT5_RF_ERR = 0.025;   // meters, Error in rangefinder when greater than or equal to 5 meters distance
static const float RMS = 1.73;           // RMS constant useful for tri-axis errors
static const uint32_t GPS_RATE = 200;    // us, GPS Update rate

//----Function delcarations----//
void initialize();
void update(float Rangems, float Range);
bool run();
void recover();
//----   Helper Functions  ----//
void debug();
bool confirm(const float a, const float a_err, const float b, const float b_err, bool wrap = false)
{
    if (!wrap)
    {
        if (a >= b)
        {
            return (((b + b_err) - (a - a_err)) >= 0);
        }
        else
        {
            return (((a + a_err) - (b - b_err)) >= 0);
        }
    }
    else
    {
        float lower = 0;
        float upper = 0;
        if (a < b)
        {
            lower = a + a_err;
            upper = b - b_err;
        }
        else
        {
            lower = b + b_err;
            upper = a - a_err;
        }
        if ((lower >= 360) || (upper <= 0))
        {
            // If wrapping occurs then confirm
            return true;
        }
        else if (lower - upper >= 0)
        {
            // This implies the lower + error is greater than upper - error
            return true;
        }
        //Swapping directions to see if moving away from each other causes wrapping
        if (a >= b)
        {
            upper = a + a_err;
            lower = b - b_err;
        }
        else
        {
            upper = b + b_err;
            lower = a - a_err;
        }
        if ((lower <= 0) && (upper >= 360))
        {
            // If both wrap its an easy confirmation
            return true;
        }
        else if (lower <= 0)
        {
            // If the lower value wraps around, confirm if it becomes less than the upper
            lower = fmod(lower, 360);
            if (upper - lower >= 0)
                return true;
        }
        else if (upper >= 360)
        {
            // If the upper value wraps around, confirm if it becomes greater than the lower
            upper = fmod(upper, 360);
            if (upper - lower >= 0)
                return true;
        }
        return false;
    }
}
Vector3f abs(const Vector3f &x)
{
    return Vector3f{abs(x.x), abs(x.y), abs(x.z)};
}

//----Specific Confirmations----//
bool AccGPS();
bool AccOF();
bool GpsOF();
bool AccGpsGC();
bool AccOFGC();
bool GpsMagGC();
bool GpsOFGC();

typedef Vector3f NED;
typedef Vector3f BF;

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
            if (!frontend->gps_yaw_deg(Yaw, Yaw_Error))
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
        //Important note, after every reset if time matters assign the old timestamp to the reset sensor
        Timestamp = 0;
    }
};

struct RF
{
    float Range;     //meters, Rangefinder reading after being LPF
    float Timestamp; //ms, Timestamp of last healthy RF reading
    float RangeErr;  //meters, Error in RF reading

    void update(float Rangems, float newRange)
    {
        uint32_t dT = Rangems - Timestamp;
        if (dT > 0)
        {
            Range = newRange / 100;
            RangeErr = Range >= 5 ? GT5_RF_ERR : LT5_RF_ERR;
            Timestamp += dT;
        }
    }

    void reset()
    {
        Range = 0;
        //Important note, after every reset if time matters assign the old timestamp to the reset sensor
        Timestamp = 0;
        RangeErr = 0;
    }
};

struct OF
{
    Vector2f FlowRate;  // rad/s, this is the angular rate calculated by the camera, i.e., HereFlow
    Vector2f BodyRate;  // rad/s, in SITL this is the same as the gyroscope. Pitch Roll
    uint32_t Timestamp; // ms, Timestamp of last OF Update
    float RateErr;      // rad/s, Error in angular rate. Depends on sensor but HereFlow has a separate Gyroscope
    float Range;        // cm, Rangefinder reading at update time
    //I'm letting the OF store the velocity data as the Rangefinder has a typical operating
    //rate of 270Hz meaning it should have more relevant information on OF update
    BF VelBF;   //cm/s, Velocity in Body Frame (OF, RF, Barometer)
    NED VelNED; //cm/s, velocity in NED frame (Mag, OF, RF)
    NED Err;    //cm/s. Error in velocity in NED frame

    bool update(const OpticalFlow *frontend, const RF &currRF)
    {
        uint32_t dT = frontend->last_update() - Timestamp; //ms
        if (dT > 0)
        {
            FlowRate = frontend->flowRate();
            BodyRate = frontend->bodyRate();
            RateErr += OF_GYRO_ERR;
            Timestamp += dT;
            VelBF = Vector3f{tan(FlowRate[1] - BodyRate[1]) * (currRF.Range),            //m/s, Front
                             tan(FlowRate[0] - BodyRate[0]) * (currRF.Range),            //m/s, Right
                             ((currRF.Range - Range) / (dT / 1000.0F))};                 //m/s, Down
            VelNED = (AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned() * VelBF); //Rotated BF to NED

            BF ErrBF = Vector3f{tan((FlowRate[1] - BodyRate[1]) + RateErr + FLOW_ERR) * ((currRF.Range) + currRF.RangeErr),
                                tan((FlowRate[0] - BodyRate[0]) + RateErr + FLOW_ERR) * ((currRF.Range) + currRF.RangeErr),
                                ((currRF.Range + currRF.RangeErr - Range) / (dT / 1000.0F))};
            Err = (AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned() * abs(ErrBF - VelBF));
            Range = currRF.Range;
            return true;
        }
        return false;
    }

    void reset(void)
    {
        FlowRate.zero();
        BodyRate.zero();
        //Important note, after every reset if time matters assign the old timestamp to the reset sensor
        Timestamp = 0;
        RateErr = 0;
        //Important Note, Optical Flow needs to caarry over previous Range for proper velocity calculations
        Range = 0;
        VelBF.zero();
        VelNED.zero();
    }

    OF &operator=(const OF &rhs)
    {
        FlowRate = rhs.FlowRate;
        BodyRate = rhs.BodyRate;
        Timestamp = rhs.Timestamp;
        RateErr = rhs.RateErr;
        Range = rhs.Range;
        VelBF = rhs.VelBF;
        VelNED = rhs.VelNED;
        Err = rhs.Err;
        return *this;
    }
};

//Sensor Data
static struct
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
static struct
{
    bool init = false; //Initialized Structs
    bool gpsAvail;     //GPS Data has been updated
    bool ofAvail;      // OF Data has been updated
} framework;

//Bias Data
static struct
{
    bool biased = false; //True when bias has been determined
    uint32_t Timestamp;  //Used to determine when readings change for bias
    uint32_t count = 0;  //Number of readings used in bias
    NED AccBias;         //Bias for NED acceleration
} bias;

void Rover::sensor_confirmation(float Rangems, float range)
{
    const uint32_t time = AP_HAL::micros64();
    //Initialize struct data
    if (!framework.init)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "PDLK: Initializing...");
        initialize();
        DELAY += time;
        BIAS_DELAY += time;
    }
    if (time >= DELAY)
    {
        if (!bias.biased && time < BIAS_DELAY)
        {
            //Can't bias rangefinder on the ground, nonlinearity when below 1m reading
            gcs().send_text(MAV_SEVERITY_INFO, "PDLK: Biasing...");
            const AP_InertialSensor *IMU = AP_InertialSensor::get_singleton();
            bias.AccBias += IMU->get_accel() * AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned();
            bias.count++;
        }
        else if (!bias.biased)
        {
            bias.AccBias /= bias.count;
            bias.biased = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Sensors have been biased.");
            gcs().send_text(MAV_SEVERITY_INFO, "Acc (F/R/D): (%f/%f/%f)", bias.AccBias[0], bias.AccBias[1], bias.AccBias[2]);
            sensors.currAccel.Timestamp = AP_HAL::micros64();
        }
        else
        {
            //Update sensor values
            update(Rangems, range);

            //If run returns False, Recover
            //If run returns True, Nothing
            if (!run())
            {
                recover();
            }
        }
    }
}

void initialize()
{
    // Reset sensors
    sensors.currAccel.reset();
    sensors.nextAccel.reset();
    sensors.currGyro.reset();
    sensors.nextGyro.reset();
    sensors.currGps.reset();
    sensors.prevGps.reset();
    sensors.currOF.reset();
    sensors.nextOF.reset();
    sensors.rangefinder.reset();

    //Zero framework readings
    framework.gpsAvail = false;
    framework.init = true;

    //Set bias
    bias.AccBias.zero();
    bias.biased = false;
    bias.count = 0;
    bias.Timestamp = AP_HAL::micros64();
}

void update(float Rangems, float Range)
{
    const AP_InertialSensor *IMU = AP_InertialSensor::get_singleton();
    const OpticalFlow *OF = AP::opticalflow();

    //---OF---//
    // OF specific confirmation for accelerometer
    //RF
    sensors.rangefinder.update(Rangems, Range);

    //Update the ACCOF sensors
    framework.ofAvail = sensors.cOF.update(OF, sensors.rangefinder);
    if(framework.ofAvail)
    {
        sensors.nOFAccel.Timestamp = sensors.cOFAccel.Timestamp;
        sensors.nOFAccel.update(IMU, bias.AccBias);
    } else
    {
        sensors.cOFAccel.update(IMU, bias.AccBias);
    }
    
    //---GPS---//
    // GPS specific confirmation updates below
    // The GPS will be lagging behind the accelerometer in practice so we want to
    // be sure to update nextAccel in the same moment that the GPS updates
    framework.gpsAvail = sensors.currGps.update(AP::gps().get_singleton(), sensors.prevGps);
    
    //IMU Sensors
    //Updating Current and Previous based on GPS Update rate

    if (((IMU->get_last_update_usec() / 1000) - sensors.currGps.Timestamp) >= GPS_RATE)
    {
        sensors.nextAccel.update(IMU, bias.AccBias);
        sensors.nextGyro.update(IMU);
    }
    else
    {
        sensors.currAccel.update(IMU, bias.AccBias);
        sensors.currGyro.update(IMU);
    }

    if ((sensors.currOF.Timestamp - sensors.currGps.Timestamp) >= (GPS_RATE / 1000))
    {
        sensors.nextOF.update(OF, sensors.rangefinder);
    }
    else
    {
        sensors.currOF.update(OF, sensors.rangefinder);
    }
}

bool run()
{
    if (framework.gpsAvail)
    {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && 0
        debug();
#endif
        AP_Logger::get_singleton()->Write_CNFR(sensors.prevOF.VelNED,
                                               sensors.prevOF.Err,
                                               sensors.currOF.VelNED,
                                               sensors.currOF.Err,
                                               sensors.prevGps.Airspeed,
                                               sensors.prevGps.Sacc,
                                               sensors.currGps.Airspeed,
                                               sensors.currGps.Sacc,
                                               sensors.currAccel.Velocity,
                                               sensors.currAccel.Error,
                                               sensors.nextAccel.Velocity,
                                               sensors.nextAccel.Error);
        /*
        if (false && !AccGPS())
        {
            gcs().send_text(MAV_SEVERITY_WARNING, "AccGPS Failed.");
        }
        if (false && !GpsMagGC())
        {
            gcs().send_text(MAV_SEVERITY_WARNING, "GpsMag Failed.");
        }
        if (!GpsOFGC())
        {
            gcs().send_text(MAV_SEVERITY_WARNING, "GpsOFGC Failed.");
        }
        // if (!confirmAlt())
        // {
        //     gcs().send_text(MAV_SEVERITY_WARNING, "Altitude outside of error ranges.");
        // }
        */

        //Switch to next IMU data
        sensors.currAccel = sensors.nextAccel;
        sensors.nextAccel.reset();
        sensors.nextAccel.Timestamp = sensors.currAccel.Timestamp;
        sensors.currGyro = sensors.nextGyro;
        sensors.nextGyro.reset();
        sensors.nextGyro.Timestamp = sensors.currGyro.Timestamp;

        //Switch to next OF data
        sensors.prevOF = sensors.currOF;
        sensors.currOF = sensors.nextOF;
        sensors.nextOF.reset();
        sensors.nextOF.Timestamp = sensors.currOF.Timestamp;
        sensors.nextOF.Range = sensors.currOF.Range;

        framework.gpsAvail = false;
    }
    if(framework.ofAvail)
    {
        AP_Logger::get_singleton()->Write_CNFR2(sensors.pOF.VelNED,
                                               sensors.pOF.Err,
                                               sensors.cOF.VelNED,
                                               sensors.cOF.Err,
                                               sensors.cOFAccel.Velocity,
                                               sensors.cOFAccel.Error);
        // if (!AccOF())
        // {
        //     gcs().send_text(MAV_SEVERITY_WARNING, "AccOF Failed.");
        // }
        //Save current accelerometer data and move new data into current sensor
        sensors.pOFAccel = sensors.cOFAccel;
        sensors.cOFAccel = sensors.nOFAccel;
        sensors.nOFAccel.reset();

        //Save current OF data and reset current
        sensors.pOF = sensors.cOF;
        sensors.cOF.reset();
        sensors.cOF.Timestamp = sensors.pOF.Timestamp;
        sensors.cOF.Range = sensors.pOF.Range;

        framework.ofAvail = false;
    }
    return true;
}

void recover()
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Recovery");
}

void debug()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    gcs().send_text(MAV_SEVERITY_INFO, "CURRENT [%llu] currAcc: %lu | nextAcc: %lu | GPS: %lu",
                    AP_HAL::micros64(),
                    sensors.currAccel.Timestamp,
                    sensors.nextAccel.Timestamp,
                    sensors.gps.Timestamp);
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "CURRENT [%lu] currAcc: %u | nextAcc: %u | GPS: %u",
                    AP_HAL::micros64(),
                    sensors.currAccel.Timestamp,
                    sensors.nextAccel.Timestamp,
                    sensors.currGps.Timestamp);
#endif
}

// Confirm change in velocity of GPS and Accelerometer
bool AccGPS()
{
    float dAccVel = sensors.currAccel.Velocity.length();
    Vector3f dGpsVel = sensors.currGps.Airspeed - sensors.prevGps.Airspeed;
    return (confirm(dAccVel, sensors.currAccel.Error, dGpsVel.length(), sensors.currGps.Sacc + sensors.prevGps.Sacc));
}

// Confirm change in velocity of Accelerometer and OF
bool AccOF()
{
    float dAccVel = sensors.cOFAccel.Velocity.length();
    Vector3f dOFVel = sensors.cOF.VelNED - sensors.pOF.VelNED;
    return (confirm(dAccVel, sensors.currAccel.Error, dOFVel.length(), sensors.cOF.Err.length() + sensors.pOF.Err.length()));
}

// Confirm velocity of GPS and Optical Flow Sensor
bool GpsOF()
{
    return (confirm(sensors.currOF.VelNED.length(), sensors.currOF.Err.length(), sensors.currGps.Airspeed.length(), sensors.currGps.Sacc));
}

// Confirm ground course based on GPS movement and Magnetometer heading
bool GpsMagGC()
{
    if (abs(sensors.currGps.Airspeed[0]) < 0.05 && abs(sensors.currGps.Airspeed[1]) < 0.05)
    {
        // Speed is below potential error, GpsMag is unusable
        return true;
    }
    std::vector<float> north{1, 0};

    //GPS and GPS Error
    std::vector<float> gps{sensors.currGps.Airspeed[0], sensors.currGps.Airspeed[1]};
    float dot = gps[0];  // N * 1 + E * 0
    float det = -gps[1]; // N * 0 - E * 1
    float GpsGC = atan2(det, dot);
    std::vector<float> ErrGps{abs(sensors.currGps.Airspeed[0]) - abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000)),
                              abs(sensors.currGps.Airspeed[1]) + abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000))};
    gps[0] = abs(gps[0]);
    gps[1] = abs(gps[1]);
    dot = gps[0] * ErrGps[0] + gps[1] * ErrGps[1]; // N * N + E * E
    det = gps[0] * ErrGps[1] - gps[1] * ErrGps[0]; // N * E - E * N
    float ErrGpsGC = ToDeg(atan2(det, dot));

    //Magnetometer with Gyro Error
    const Matrix3f dcm = AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned();
    float MagGC = ToDeg(atan2f(dcm.b[0], dcm.a[0]));
    if (MagGC < 0)
        MagGC += 360;

    return (confirm(GpsGC, ErrGpsGC, MagGC, ToDeg(sensors.currGyro.Error * (sensors.currGyro.TimeContained / (float)1000000)), true));
}

// Confirm ground course based on GPS movement and Optical Flow movement with Magnetometer for rotation
bool GpsOFGC()
{
    if (abs(sensors.currGps.Airspeed[0]) < 0.05 && abs(sensors.currGps.Airspeed[1]) < 0.05)
    {
        // Speed is below potential error, GpsOF is unusable
        return true;
    }
    std::vector<float> north{1, 0};

    //GPS and GPS Error
    std::vector<float> gps{sensors.currGps.Airspeed[0], sensors.currGps.Airspeed[1]};
    float dot = gps[0];  // N * 1 + E * 0
    float det = -gps[1]; // N * 0 - E * 1
    float GpsGC = atan2(det, dot);
    std::vector<float> ErrGps{abs(sensors.currGps.Airspeed[0]) - abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000)),
                              abs(sensors.currGps.Airspeed[1]) + abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000))};
    gps[0] = abs(gps[0]);
    gps[1] = abs(gps[1]);
    dot = gps[0] * ErrGps[0] + gps[1] * ErrGps[1]; // N * N + E * E
    det = gps[0] * ErrGps[1] - gps[1] * ErrGps[0]; // N * E - E * N
    float ErrGpsGC = ToDeg(atan2(det, dot));

    //Optical Flow and Optical Flow Error
    std::vector<float> OF{sensors.currOF.VelNED[0], sensors.currOF.VelNED[1]};
    dot = OF[0];
    det = -OF[1];
    float OFGC = atan2(det, dot);
    OF[0] = abs(OF[0]);
    OF[1] = abs(OF[1]);
    dot = OF[0] * abs(sensors.currOF.Err[0]) + OF[1] * abs(sensors.currOF.Err[1]); // N * N + E * E
    det = OF[0] * abs(sensors.currOF.Err[1]) - OF[1] * abs(sensors.currOF.Err[0]); // N * E - E * N
    float ErrOFGC = ToDeg(atan2(det, dot));

    return (confirm(GpsGC, ErrGpsGC, OFGC, ErrOFGC, true));
}

// Confirm ground course based on GPS movement and Accelerometer movement with Magnetometer for rotation
bool AccGpsGC()
{
    if (abs(sensors.currGps.Airspeed[0]) < 0.05 && abs(sensors.currGps.Airspeed[1]) < 0.05)
    {
        // Speed is below potential error, GpsAcc is unusable
        return true;
    }
    std::vector<float> north{1, 0};

    //GPS and GPS Error
    std::vector<float> gps{sensors.currGps.Airspeed[0], sensors.currGps.Airspeed[1]};
    float dot = gps[0];  // N * 1 + E * 0
    float det = -gps[1]; // N * 0 - E * 1
    float GpsGC = atan2(det, dot);
    std::vector<float> ErrGps{abs(sensors.currGps.Airspeed[0]) - abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000)),
                              abs(sensors.currGps.Airspeed[1]) + abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000))};
    gps[0] = abs(gps[0]);
    gps[1] = abs(gps[1]);
    dot = gps[0] * ErrGps[0] + gps[1] * ErrGps[1]; // N * N + E * E
    det = gps[0] * ErrGps[1] - gps[1] * ErrGps[0]; // N * E - E * N
    float ErrGpsGC = ToDeg(atan2(det, dot));

    //Accelerometer and Accelerometer Error
    std::vector<float> Acc{sensors.currAccel.Velocity[0], sensors.currAccel.Velocity[1]};
    dot = Acc[0];
    det = -Acc[1];
    float AccGC = atan2(det, dot);
    Acc[0] = abs(Acc[0]);
    Acc[1] = abs(Acc[1]);
    dot = Acc[0] * abs(sensors.currAccel.Error) + Acc[1] * abs(sensors.currAccel.Error); // N * N + E * E
    det = Acc[0] * abs(sensors.currAccel.Error) - Acc[1] * abs(sensors.currAccel.Error); // N * E - E * N
    float ErrAccGC = ToDeg(atan2(det, dot));

    return (confirm(GpsGC, ErrGpsGC, AccGC, ErrAccGC, true));
}

// Confirm ground course based on Accelerometer and Optical Flow movement with Magnetometer for rotation
bool AccOFGC()
{
    if (abs(sensors.currAccel.Velocity[0] < sensors.currAccel.Error && abs(sensors.currAccel.Velocity[1]) < sensors.currAccel.Error))
    {
        // Speed is below potential error, GpsAcc is unusable
        return true;
    }
    std::vector<float> north{1, 0};

    //Accelerometer and Accelerometer Error
    std::vector<float> Acc{sensors.currAccel.Velocity[0], sensors.currAccel.Velocity[1]};
    float dot = Acc[0];  // N * 1 + E * 0
    float det = -Acc[1]; // N * 0 - E * 1
    float AccGC = atan2(det, dot);
    Acc[0] = abs(Acc[0]);
    Acc[1] = abs(Acc[1]);
    dot = Acc[0] * abs(sensors.currAccel.Error) + Acc[1] * abs(sensors.currAccel.Error); // N * N + E * E
    det = Acc[0] * abs(sensors.currAccel.Error) - Acc[1] * abs(sensors.currAccel.Error); // N * E - E * N
    float ErrAccGC = ToDeg(atan2(det, dot));

    //Optical Flow and Optical Flow Error
    std::vector<float> OF{sensors.currOF.VelNED[0], sensors.currOF.VelNED[1]};
    dot = OF[0];
    det = -OF[1];
    float OFGC = atan2(det, dot);
    std::vector<float> ErrOF{abs(sensors.currOF.VelNED[0]) - abs(sensors.currOF.Err[0]),
                             abs(sensors.currOF.VelNED[1]) + abs(sensors.currOF.Err[1])};
    OF[0] = abs(OF[0]);
    OF[1] = abs(OF[1]);
    dot = OF[0] * ErrOF[0] + OF[1] * ErrOF[1]; // N * N + E * E
    det = OF[0] * ErrOF[1] - OF[1] * ErrOF[0]; // N * E - E * N
    float ErrOFGC = ToDeg(atan2(det, dot));

    return (confirm(OFGC, ErrOFGC, AccGC, ErrAccGC, true));
}
