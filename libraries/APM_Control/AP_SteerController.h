#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>

class AP_SteerController {
public:
    AP_SteerController()
    {
        AP_Param::setup_object_defaults(this, var_info);
        //@@ init variables
        /*
        window = 200;      // dt:0.02   dt*window = 4sec
        threshold = 20000;
        sq_err_sum = 0;
        index = 0;
        */
    }

    /* Do not allow copies */
    AP_SteerController(const AP_SteerController &other) = delete;
    AP_SteerController &operator=(const AP_SteerController&) = delete;

    /*
      return a steering servo output from -4500 to 4500 given a
      desired lateral acceleration rate in m/s/s. Positive lateral
      acceleration is to the right.
     */
	int32_t get_steering_out_lat_accel(float desired_accel);

    /*
      return a steering servo output from -4500 to 4500 given a
      desired yaw rate in degrees/sec. Positive yaw is to the right.
     */
	int32_t get_steering_out_rate(float desired_rate);

    /*
      return a steering servo output from -4500 to 4500 given a
      yaw error in centi-degrees
     */
	int32_t get_steering_out_angle_error(int32_t angle_err);

    /*
      return the steering radius (half diameter). Assumed to be half
      the P value.
     */
    float get_turn_radius(void) const { return _K_P * 0.5f; }

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

    const AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

    void set_reverse(bool reverse) {
        _reverse = reverse;
    }

private:
    AP_Float _tau;
	AP_Float _K_FF;
	AP_Float _K_P;
	AP_Float _K_I;
	AP_Float _K_D;
	AP_Float _minspeed;
    AP_Int16  _imax;
	uint32_t _last_t;
	float _last_out;

	AP_Float _deratespeed;
	AP_Float _deratefactor;
	AP_Float _mindegree;

    AP_Logger::PID_Info _pid_info {};

	AP_AHRS &_ahrs;
    //@@INVARIANT
    /*
    float    A[3][3] = {{ 0.0184f, -6.1930f, -0.3636f}, {0.0061f, 0.9083f, -0.0052f}, {0.0001f, 0.0192f, 1.0f}};
    float    B[3] = {0.0061f, 0.001f, 0.0f};
    float    C[3] = {201.6834f, 917.1042f, 53.3688f};
    float    D = 0.0f;

    float    inv_y;
    float    inv_x[3] = {0.0f, 0.0f, 0.0f};

    float    window;    // window size
    float    threshold;
    float    error[6000];   // squared error
    float    sq_err_sum;
    float    mse;           // mean squared error
    int      index;     // current index in window
    float    max_mse;
    */
    bool _reverse;
};
