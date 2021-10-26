#include "./sensor_confirmation.h"

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

    //Set bias
    bias.AccBias.zero();
    bias.biased = false;
    bias.count = 0;
    bias.Timestamp = AP_HAL::micros64();
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
        sensors.nOFAccel.update(IMU, bias.AccBias);
    }
    else
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

    if ((signed)((IMU->get_last_update_usec() / (float)1000) - sensors.currGps.Timestamp) >= GPS_RATE)
    {
        sensors.nextAccel.update(IMU, bias.AccBias);
        sensors.nextGyro.update(IMU);
    }
    else
    {
        sensors.currAccel.update(IMU, bias.AccBias);
        sensors.currGyro.update(IMU);
    }

    if ((signed)(sensors.currOF.Timestamp - sensors.currGps.Timestamp) >= GPS_RATE)
    {
        sensors.nextOF.update(AP_OF, sensors.rangefinder);
    }
    else
    {
        sensors.currOF.update(AP_OF, sensors.rangefinder);
    }
}

bool SensorConfirmation::run()
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
                                                sensors.prevGps.Hacc,
                                                sensors.currGps.Airspeed,
                                                sensors.currGps.Hacc,
                                                sensors.currAccel.Velocity,
                                                sensors.currAccel.Error,
                                                sensors.nextAccel.Velocity,
                                                sensors.nextAccel.Error,
                                                ToDeg(sensors.currGyro.Error * (sensors.currGyro.TimeContained / (float)1000000)));

        // if (!AccGPS())
        // {
        //     gcs().send_text(MAV_SEVERITY_WARNING, "AccGPS Failed.");
        // }
        // if (!GpsMagGC())
        // {
        //     gcs().send_text(MAV_SEVERITY_WARNING, "GpsMag Failed.");
        // }
        // if(!GpsOF())
        // {
        //     gcs().send_text(MAV_SEVERITY_WARNING, "GpsOF Failed.");
        // }
        // if (!GpsOFGC())
        // {
        //     gcs().send_text(MAV_SEVERITY_WARNING, "GpsOFGC Failed.");
        // }
        // if (!confirmAlt())
        // {
        //     gcs().send_text(MAV_SEVERITY_WARNING, "Altitude outside of error ranges.");
        // }

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

        framework.gpsAvail = false;
    }
    if (framework.ofAvail)
    {
        AP_Logger::get_singleton()->Write_ACO(sensors.pOF.VelNED,
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

        framework.ofAvail = false;
    }
    return true;
}

void SensorConfirmation::alert()
{
    gcs().send_text(MAV_SEVERITY_WARNING, "ALERT");
}

void SensorConfirmation::debug()
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

void SensorConfirmation::confirmation()
{
    const uint32_t time = AP_HAL::micros64();
    //Initialize struct data
    if (!framework.init)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "PDLK: Initializing...");
        initialize();
        BIAS_DELAY += time + DELAY;
        DELAY += time;
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
            update();

            //If run returns False, Recover
            //If run returns True, Nothing
            if (!run())
            {
                alert();
            }
        }
    }
}

//----Confirmation Functions----//

// Confirm change in velocity of GPS and Accelerometer
bool SensorConfirmation::AccGPS()
{
    float dAccVel = sensors.currAccel.Velocity.length();
    Vector3f dGpsVel = sensors.currGps.Airspeed - sensors.prevGps.Airspeed;
    return (confirm(dAccVel, sensors.currAccel.Error, dGpsVel.length(), sensors.currGps.Sacc + sensors.prevGps.Sacc, false));
}

// Confirm change in velocity of Accelerometer and OF
bool SensorConfirmation::AccOF()
{
    float dAccVel = sensors.cOFAccel.Velocity.length();
    Vector3f dOFVel = sensors.cOF.VelNED - sensors.pOF.VelNED;
    return (confirm(dAccVel, sensors.currAccel.Error, dOFVel.length(), sensors.cOF.Err.length() + sensors.pOF.Err.length(), false));
}

// Confirm velocity of GPS and Optical Flow Sensor
bool SensorConfirmation::GpsOF()
{
    return (confirm(sensors.currOF.VelNED.length(), sensors.currOF.Err.length(), sensors.currGps.Airspeed.length(), sensors.currGps.Sacc, false));
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
    std::vector<float> mOF{sensors.currOF.VelNED[0], sensors.currOF.VelNED[1]};
    dot = mOF[0];
    det = -mOF[1];
    float OFGC = ToDeg(atan2f(det, dot));
    OFGC = (GpsGC > 0) ? 360 - OFGC : std::abs(OFGC);
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
    if (std::abs(sensors.currAccel.Velocity[0] < sensors.currAccel.Error && std::abs(sensors.currAccel.Velocity[1]) < sensors.currAccel.Error))
    {
        // Speed is below potential error, GpsAcc is unusable
        return true;
    }
    std::vector<float> north{1, 0};

    //Accelerometer and Accelerometer Error
    std::vector<float> Acc{sensors.currAccel.Velocity[0], sensors.currAccel.Velocity[1]};
    float dot = Acc[0];  // N * 1 + E * 0
    float det = -Acc[1]; // N * 0 - E * 1
    float AccGC = atan2f(det, dot);
    Acc[0] = std::abs(Acc[0]);
    Acc[1] = std::abs(Acc[1]);
    dot = Acc[0] * std::abs(sensors.currAccel.Error) + Acc[1] * std::abs(sensors.currAccel.Error); // N * N + E * E
    det = Acc[0] * std::abs(sensors.currAccel.Error) - Acc[1] * std::abs(sensors.currAccel.Error); // N * E - E * N
    float ErrAccGC = ToDeg(atan2f(det, dot));

    //Optical Flow and Optical Flow Error
    std::vector<float> mOF{sensors.currOF.VelNED[0], sensors.currOF.VelNED[1]};
    dot = mOF[0];
    det = -mOF[1];
    float OFGC = atan2f(det, dot);
    std::vector<float> ErrOF{std::abs(sensors.currOF.VelNED[0]) - std::abs(sensors.currOF.Err[0]),
                                std::abs(sensors.currOF.VelNED[1]) + std::abs(sensors.currOF.Err[1])};
    mOF[0] = std::abs(mOF[0]);
    mOF[1] = std::abs(mOF[1]);
    dot = mOF[0] * ErrOF[0] + mOF[1] * ErrOF[1]; // N * N + E * E
    det = mOF[0] * ErrOF[1] - mOF[1] * ErrOF[0]; // N * E - E * N
    float ErrOFGC = ToDeg(atan2f(det, dot));

    return (confirm(OFGC, ErrOFGC, AccGC, ErrAccGC, true));
}

