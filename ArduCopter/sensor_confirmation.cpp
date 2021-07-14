//PADLOCK
//Sensor Confirmation source code
//7/12/2021

#include "Copter.h"

//Constant declarations
static uint32_t DELAY = 45000000U;      // Start-Up delay to allow sensors to settle
static uint32_t BIAS_DELAY = 90000000U; // Delay after Start-Up to allow biasing
static const float ACC_ERR = 0.0015;    // (m/s/s)/(sqrt(Hz)) Accelerometer noise (LSM303D) Assuming 100Hz Bandwidth
static const float GYRO_ERR = 0.00019;  // (rad/s)/(sqrt(Hz)) Gyro Noise (L3GD20H) Assuming 50Hz Bandwidth
static const float RMS = 1.73;          // RMS constant useful for tri-axis errors
static const uint32_t GPS_RATE = 200;   // us, GPS Update rate

//----Function delcarations----//
void initialize();
void update();
bool run();
void recover();
void debug();
//----Specific Confirmations----//
bool AccGPS();
bool confirmAlt();

typedef Vector3f NED;
typedef Vector3f BF;

struct Accel
{
    NED Readings;       //m/s/s, Accelerometer readings rotated to NED
    NED Velocity;       //m/s
    float Error;        //m/s/s
    uint32_t Timestamp; //us

    bool update(const AP_InertialSensor *frontend, NED bias)
    {
        uint32_t dT = frontend->get_last_update_usec() - Timestamp; //us
        if (dT > 0)
        {
            NED newReading = AP_AHRS_DCM::get_singleton()->body_to_earth(frontend->get_accel()) - bias;
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

    void reset()
    {
        Readings.zero();
    }

    Gyro &operator=(const Gyro &rhs)
    {
        Readings = rhs.Readings;
        Timestamp = rhs.Timestamp;
        Error = rhs.Error;
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
        Timestamp = 0;
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
} sensors;

//Confirmation Data
static struct
{
    bool init = false;    //Initialized Structs
    bool gpsAvail;        //GPS Data has been updated
    uint32_t lastConfirm; //ms (Time that the last confirmation occurred)
    NED lastGPS;          //m/s (Velocity provided by last GPS Sensor readings)
    NED lastAcc;          //m/s (Velocity calculated from previous Acc Sensor data)
} framework;

static struct
{
    bool biased = false; //True when bias has been determined
    uint32_t Timestamp;  //Used to determine when readings change for bias
    uint32_t count = 0;  //Number of readings used in bias
    NED AccBias;         //Bias for NED acceleration
} bias;

void Copter::sensor_confirmation()
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
            gcs().send_text(MAV_SEVERITY_INFO, "PDLK: Biasing...");
            const AP_InertialSensor *IMU = AP_InertialSensor::get_singleton();
            bias.AccBias += AP_AHRS_DCM::get_singleton()->body_to_earth(IMU->get_accel());
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
            update();

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
    // Rest sensors
    sensors.currAccel.reset();
    sensors.nextAccel.reset();
    sensors.currGyro.reset();
    sensors.nextGyro.reset();
    sensors.currGps.reset();
    sensors.prevGps.reset();

    //Zero framework readings
    framework.lastGPS.zero();
    framework.lastAcc.zero();
    framework.gpsAvail = false;
    framework.init = true;
    framework.lastConfirm = 0;

    //Set bias
    bias.AccBias.zero();
    bias.biased = false;
    bias.count = 0;
    bias.Timestamp = AP_HAL::micros64();
}

void update()
{
    //IMU Sensors
    const AP_InertialSensor *IMU = AP_InertialSensor::get_singleton();
    if (((IMU->get_last_update_usec() / 1000) - sensors.currGps.Timestamp) >= GPS_RATE)
    {
        if (sensors.nextAccel.Timestamp == 0)
        {
            // Because our Velocity is based on change in time we need
            // to persist the timestamp from old to new accelerometer
            // frames
            sensors.nextAccel.Timestamp = sensors.currAccel.Timestamp;
        }

        sensors.nextAccel.update(IMU, bias.AccBias);
    }
    else
    {
        sensors.currAccel.update(IMU, bias.AccBias);
    }

    //GPS
    // The GPS will be lagging behind the accelerometer in practice so we want to
    // be sure to update nextAccel in the same moment that the GPS updates
    framework.gpsAvail = sensors.currGps.update(AP::gps().get_singleton(), sensors.prevGps);
}

bool run()
{
    if (framework.gpsAvail)
    {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && 0
        debug();
#endif

        if (!AccGPS())
        {
            gcs().send_text(MAV_SEVERITY_WARNING, "AccGPS Failed.");
        }
        // if (!confirmAlt())
        // {
        //     gcs().send_text(MAV_SEVERITY_WARNING, "Altitude outside of error ranges.");
        // }

        //Switch to next IMU data
        sensors.currAccel = sensors.nextAccel;
        sensors.nextAccel.reset();
        sensors.currGyro = sensors.nextGyro;
        sensors.nextGyro.reset();

        framework.gpsAvail = false;
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

bool confirm(const float a, const float a_err, const float b, const float b_err, bool wrap = false)
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

bool AccGPS()
{
    float dAccVel = sensors.currAccel.Velocity.length();
    Vector3f dGpsVel = sensors.currGps.Airspeed - sensors.prevGps.Airspeed;
    return (confirm(dAccVel, sensors.currAccel.Error, dGpsVel.length(), sensors.currGps.Sacc + sensors.prevGps.Sacc));
}

bool confirmAlt()
{
    //TODO: Confirmation between Barometer, GPS, and Acc
    return true;
}