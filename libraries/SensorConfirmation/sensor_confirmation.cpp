#include "AP_Logger/AP_Logger.h"
#include <SensorConfirmation/sensor_confirmation.h>

//----   Helper Functions  ----//
bool SensorConfirmation::confirm(const float a, const float a_err, const float b, const float b_err, bool wrap)
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
Vector3f SensorConfirmation::abs(const Vector3f &x)
{
    return Vector3f{std::abs(x.x), std::abs(x.y), std::abs(x.z)};
};
Vector2f SensorConfirmation::abs(const Vector2f &x)
{
    return Vector2f{std::abs(x.x), std::abs(x.y)};
};

//----General Functions----//

void SensorConfirmation::initialize()
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
}

void SensorConfirmation::update()
{
    const AP_InertialSensor *IMU = AP_InertialSensor::get_singleton();
    const OpticalFlow *AP_OF = OpticalFlow::get_singleton();

    //---OF---//
    // OF specific confirmation for accelerometer
    //RF
    sensors.rangefinder.update();

    //Update the ACCOF sensors
    framework.ofAvail = sensors.cOF.update(AP_OF, sensors.rangefinder);
    if (framework.ofAvail)
    {
        sensors.nOFAccel.Timestamp = sensors.cOFAccel.Timestamp;
        sensors.nOFAccel.update(IMU);
    }
    else
    {
        sensors.cOFAccel.update(IMU);
    }

    //---GPS---//
    // GPS specific confirmation updates below
    // The GPS will be lagging behind the accelerometer in practice so we want to
    // be sure to update nextAccel in the same moment that the GPS updates
    framework.gpsAvail = sensors.currGps.update(AP::gps().get_singleton(), sensors.prevGps);

    //IMU Sensors
    //Updating Current and Previous based on GPS Update rate
    uint32_t ts_imu = IMU->get_last_update_usec() / 1000;
    if((ts_imu > sensors.currGps.Timestamp) && (ts_imu - sensors.currGps.Timestamp) >= GPS_RATE)
    {
        sensors.nextAccel.update(IMU);
        sensors.nextGyro.update(IMU);
    }
    else
    {
        sensors.currAccel.update(IMU);
        sensors.currGyro.update(IMU);
    }

    uint32_t ts_of = sensors.currOF.Timestamp;
    if((ts_of > sensors.currGps.Timestamp) && (ts_of - sensors.currGps.Timestamp) >= GPS_RATE)
    {
        sensors.nextOF.update(AP_OF, sensors.rangefinder);
    }
    else
    {
        sensors.currOF.update(AP_OF, sensors.rangefinder);
    }

    log();
}

bool SensorConfirmation::run()
{
    bool res = true;
    if (framework.gpsAvail)
    {
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL && 0
        log();
    #endif
        AP_Logger::get_singleton()->Write_CNFR( sensors.prevOF,
                                                sensors.currOF,
                                                sensors.prevGps,
                                                sensors.currGps,
                                                sensors.currAccel,
                                                sensors.nextAccel,
                                                ToDeg(sensors.currGyro.Error * (sensors.currGyro.TimeContained / (float)1000000)));

        // GPS Confirmations
        res &= AccGPS();
        res &= GpsMagGC();
        res &= GpsOF();
        res &= GpsOFGC();
        
        //Switch to next IMU data
        sensors.currAccel = sensors.nextAccel;
        sensors.nextAccel.reset(sensors.currAccel.Timestamp);
        sensors.currGyro = sensors.nextGyro;
        sensors.nextGyro.reset(sensors.currGyro.Timestamp);

        //Switch to next OF data
        sensors.prevOF = sensors.currOF;
        sensors.currOF = sensors.nextOF;
        sensors.nextOF.reset(sensors.currOF.Timestamp);

        framework.gpsAvail = false;
    }
    if (framework.ofAvail)
    {
        AP_Logger::get_singleton()->Write_ACO(sensors.pOF.VelNE,
                                                sensors.pOF.Err,
                                                sensors.cOF.VelNE,
                                                sensors.cOF.Err,
                                                sensors.cOFAccel.Velocity,
                                                sensors.cOFAccel.Error);
        res &= AccOF();

        sensors.pOFAccel = sensors.cOFAccel;
        sensors.cOFAccel = sensors.nOFAccel;
        sensors.nOFAccel.reset();

        //Save current OF data and reset current
        sensors.pOF = sensors.cOF;
        sensors.cOF.reset(sensors.pOF.Timestamp);

        framework.ofAvail = false;
    }
    return res;
}

void SensorConfirmation::alert()
{
    gcs().send_text(MAV_SEVERITY_WARNING, "ALERT");
}

void SensorConfirmation::log()
{
    AP_Logger::get_singleton()->Write_SNSR(//ACO Data
                          sensors.cOFAccel,
                          sensors.cOF,
                          //CNF Data
                          sensors.currAccel,
                          sensors.currGps,
                          sensors.rangefinder);
}

void SensorConfirmation::confirmation()
{
    //Initialize struct data
    if (!framework.init)
    {
        initialize();
    }

    //Update sensor values
    update();
    log();

    //If run returns False, Recover
    //If run returns True, Nothing
    if (!run())
    {
        //alert();
    }
}
//----Reactive Attacker----//
float SensorConfirmation::NetGpsLimit() const
{
    float ret = 0;
    float currGps = sensors.currGps.Airspeed.length();
    float currGpsNE = Vector2f{sensors.currGps.Airspeed.x,
                               sensors.currGps.Airspeed.y}.length();
    float currAccel = sensors.currAccel.Velocity.length();
    float currOF = sensors.currOF.VelNE.length();
    //AccGPS limit, currGps is still technically the previous update
    ret = MAX(0.0F, 2*(sensors.currGps.Hacc / 0.2F) + sensors.currAccel.Error - std::abs(currAccel - currGps));

    //GpsOF limit
    ret = MIN(ret, (sensors.currGps.Hacc / 0.2F) + sensors.currOF.Err.length() - std::abs(currOF - currGpsNE));

    //Disallow returning a negative
    ret = MAX(0.0F, ret);

    return ret;
}

float SensorConfirmation::NetOFLimit() const
{
    float ret = 0;
    float currGpsNE = Vector2f{sensors.currGps.Airspeed.x,
                               sensors.currGps.Airspeed.y}.length();
    float currAccelNE = Vector2f{sensors.cOFAccel.Velocity.x,
                                 sensors.cOFAccel.Velocity.y}.length();
    float cOF = sensors.cOF.VelNE.length();
    float currOF = sensors.currOF.VelNE.length();
    //AccOF limit, Assuming that OF and Acc error is generally constant
    ret = MAX(0.0F, sensors.cOF.Err.length() + (sensors.cOFAccel.Error * 1.414/1.73) - std::abs(currAccelNE - cOF));

    //GpsOF limit
    ret = MIN(ret, (sensors.currGps.Hacc / 0.2F) + sensors.currOF.Err.length() - std::abs(currGpsNE - currOF));

    //Disallow returning a negative
    ret = MAX(0.0F, ret);

    //Convert ret velocity into flow rate
    ret = atanf(ret / (sensors.rangefinder.rf_filt.get() / 100)) ;
    return ret;   
}

//----Confirmation Functions----//

// AccGPS Net
bool SensorConfirmation::AccGPS()
{
    float dAccVel = sensors.currAccel.Velocity.length();
    Vector3f dGpsVel = sensors.currGps.Airspeed - sensors.prevGps.Airspeed;
    return (confirm(dAccVel, sensors.currAccel.Error * sqrtf(3.0F), dGpsVel.length(), (sensors.currGps.Hacc + sensors.prevGps.Hacc)/0.2F, false));
}

// AccOF Net
bool SensorConfirmation::AccOF()
{
    float dAccVel = Vector2f{sensors.cOFAccel.Velocity.x, sensors.cOFAccel.Velocity.y}.length();
    Vector2f dOFVel = sensors.cOF.VelNE - sensors.pOF.VelNE;
    return (confirm(dAccVel, sensors.cOFAccel.Error * sqrtf(2.0F), dOFVel.length(), sensors.cOF.Err.length() + sensors.pOF.Err.length(), false));
}

// GPSOF Net
bool SensorConfirmation::GpsOF()
{
    Vector2f GpsNE = Vector2f{sensors.currGps.Airspeed.x, sensors.currGps.Airspeed.y};
    return (confirm(sensors.currOF.VelNE.length(), sensors.currOF.Err.length(), GpsNE.length(), sqrtf(2.0F)*sensors.currGps.Hacc/0.2F, false));
}

// Confirm ground course based on GPS movement and Magnetometer heading
bool SensorConfirmation::GpsMagGC()
{
    if (std::abs(sensors.currGps.Airspeed[0]) < 0.05 && std::abs(sensors.currGps.Airspeed[1]) < 0.05)
    {
        // Speed is below potential error, GpsMag is unusable
        return true;
    }
    std::vector<float> north{1, 0};

    //GPS and GPS Error
    std::vector<float> gps{sensors.currGps.Airspeed[0], sensors.currGps.Airspeed[1]};
    float dot = gps[0];  // N * 1 + E * 0
    float det = -gps[1]; // N * 0 - E * 1
    float GpsGC = atan2f(det, dot);
    std::vector<float> ErrGps{std::abs(sensors.currGps.Airspeed[0]) - std::abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000)),
                                std::abs(sensors.currGps.Airspeed[1]) + std::abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000))};
    gps[0] = std::abs(gps[0]);
    gps[1] = std::abs(gps[1]);
    dot = gps[0] * ErrGps[0] + gps[1] * ErrGps[1]; // N * N + E * E
    det = gps[0] * ErrGps[1] - gps[1] * ErrGps[0]; // N * E - E * N
    float ErrGpsGC = ToDeg(atan2f(det, dot));

    //Magnetometer with Gyro Error
    const Matrix3f dcm = AP_AHRS::get_singleton()->get_DCM_rotation_body_to_ned();
    float MagGC = ToDeg(atan2f(dcm.b[0], dcm.a[0]));
    if (MagGC < 0)
        MagGC += 360;

    return (confirm(GpsGC, ErrGpsGC, MagGC, ToDeg(sensors.currGyro.Error * (sensors.currGyro.TimeContained / (float)1000000)), true));
}

// Confirm ground course based on GPS movement and Optical Flow movement with Magnetometer for rotation
bool SensorConfirmation::GpsOFGC()
{
    if (std::abs(sensors.currGps.Airspeed[0]) < 0.05 && std::abs(sensors.currGps.Airspeed[1]) < 0.05)
    {
        // Speed is below potential error, GpsOF is unusable
        return true;
    }
    std::vector<float> north{1, 0};

    //GPS and GPS Error
    std::vector<float> gps{sensors.currGps.Airspeed[0], sensors.currGps.Airspeed[1]};
    float dot = gps[0];  // N * 1 + E * 0
    float det = -gps[1]; // N * 0 - E * 1
    float GpsGC = ToDeg(atan2f(det, dot));
    GpsGC = (GpsGC > 0) ? 360 - GpsGC : std::abs(GpsGC);
    std::vector<float> ErrGps{std::abs(sensors.currGps.Airspeed[0]) - std::abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000)),
                                std::abs(sensors.currGps.Airspeed[1]) + std::abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000))};
    gps[0] = std::abs(gps[0]);
    gps[1] = std::abs(gps[1]);
    dot = gps[0] * ErrGps[0] + gps[1] * ErrGps[1]; // N * N + E * E
    det = gps[0] * ErrGps[1] - gps[1] * ErrGps[0]; // N * E - E * N
    float ErrGpsGC = ToDeg(atan2f(det, dot));

    //Optical Flow and Optical Flow Error
    std::vector<float> mOF{sensors.currOF.VelNE[0], sensors.currOF.VelNE[1]};
    dot = mOF[0];
    det = -mOF[1];
    float OFGC = ToDeg(atan2f(det, dot));
    OFGC = (OFGC > 0) ? 360 - OFGC : std::abs(OFGC);
    mOF[0] = std::abs(mOF[0]);
    mOF[1] = std::abs(mOF[1]);
    dot = mOF[0] * std::abs(sensors.currOF.Err[0]) + mOF[1] * std::abs(sensors.currOF.Err[1]); // N * N + E * E
    det = mOF[0] * std::abs(sensors.currOF.Err[1]) - mOF[1] * std::abs(sensors.currOF.Err[0]); // N * E - E * N
    float ErrOFGC = std::abs(ToDeg(atan2f(det, dot)));

    return (confirm(GpsGC, ErrGpsGC, OFGC, ErrOFGC, true));
}

// Confirm ground course based on GPS movement and Accelerometer movement with Magnetometer for rotation
bool SensorConfirmation::AccGpsGC()
{
    if (std::abs(sensors.currGps.Airspeed[0]) < 0.05 && std::abs(sensors.currGps.Airspeed[1]) < 0.05)
    {
        // Speed is below potential error, GpsAcc is unusable
        return true;
    }
    std::vector<float> north{1, 0};

    //GPS and GPS Error
    std::vector<float> gps{sensors.currGps.Airspeed[0], sensors.currGps.Airspeed[1]};
    float dot = gps[0];  // N * 1 + E * 0
    float det = -gps[1]; // N * 0 - E * 1
    float GpsGC = atan2f(det, dot);
    std::vector<float> ErrGps{std::abs(sensors.currGps.Airspeed[0]) - std::abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000)),
                                std::abs(sensors.currGps.Airspeed[1]) + std::abs(sensors.currGps.Hacc / (GPS_RATE / (float)1000))};
    gps[0] = std::abs(gps[0]);
    gps[1] = std::abs(gps[1]);
    dot = gps[0] * ErrGps[0] + gps[1] * ErrGps[1]; // N * N + E * E
    det = gps[0] * ErrGps[1] - gps[1] * ErrGps[0]; // N * E - E * N
    float ErrGpsGC = ToDeg(atan2f(det, dot));

    //Accelerometer and Accelerometer Error
    std::vector<float> Acc{sensors.currAccel.Velocity[0], sensors.currAccel.Velocity[1]};
    dot = Acc[0];
    det = -Acc[1];
    float AccGC = atan2f(det, dot);
    Acc[0] = std::abs(Acc[0]);
    Acc[1] = std::abs(Acc[1]);
    dot = Acc[0] * std::abs(sensors.currAccel.Error) + Acc[1] * std::abs(sensors.currAccel.Error); // N * N + E * E
    det = Acc[0] * std::abs(sensors.currAccel.Error) - Acc[1] * std::abs(sensors.currAccel.Error); // N * E - E * N
    float ErrAccGC = ToDeg(atan2f(det, dot));

    return (confirm(GpsGC, ErrGpsGC, AccGC, ErrAccGC, true));
}

// Confirm ground course based on Accelerometer and Optical Flow movement with Magnetometer for rotation
bool SensorConfirmation::AccOFGC()
{
    if (std::abs(sensors.cOFAccel.Velocity[0]) < sensors.cOFAccel.Error && std::abs(sensors.cOFAccel.Velocity[1]) < sensors.cOFAccel.Error)
    {
        // Speed is below potential error, GpsAcc is unusable
        return true;
    }
    std::vector<float> north{1, 0};

    //Accelerometer and Accelerometer Error
    std::vector<float> Acc{sensors.cOFAccel.Velocity[0], sensors.cOFAccel.Velocity[1]};
    float dot = Acc[0];  // N * 1 + E * 0
    float det = -Acc[1]; // N * 0 - E * 1
    float AccGC = atan2f(det, dot);
    Acc[0] = std::abs(Acc[0]);
    Acc[1] = std::abs(Acc[1]);
    dot = Acc[0] * std::abs(sensors.cOFAccel.Error) + Acc[1] * std::abs(sensors.cOFAccel.Error); // N * N + E * E
    det = Acc[0] * std::abs(sensors.cOFAccel.Error) - Acc[1] * std::abs(sensors.cOFAccel.Error); // N * E - E * N
    float ErrAccGC = ToDeg(atan2f(det, dot));

    //Optical Flow and Optical Flow Error
    std::vector<float> mOF{sensors.cOF.VelNE[0], sensors.cOF.VelNE[1]};
    dot = mOF[0];
    det = -mOF[1];
    float OFGC = atan2f(det, dot);
    std::vector<float> ErrOF{std::abs(sensors.cOF.VelNE[0]) - std::abs(sensors.cOF.Err[0]),
                                std::abs(sensors.cOF.VelNE[1]) + std::abs(sensors.cOF.Err[1])};
    mOF[0] = std::abs(mOF[0]);
    mOF[1] = std::abs(mOF[1]);
    dot = mOF[0] * ErrOF[0] + mOF[1] * ErrOF[1]; // N * N + E * E
    det = mOF[0] * ErrOF[1] - mOF[1] * ErrOF[0]; // N * E - E * N
    float ErrOFGC = ToDeg(atan2f(det, dot));

    return (confirm(OFGC, ErrOFGC, AccGC, ErrAccGC, true));
}

