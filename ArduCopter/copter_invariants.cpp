/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// populate the invariant model and parameters here
// =================================================================================================================
// invariant Model : ArduCopter
static const float    A[3][3] = {{ -0.000000018536, -0.0176, -0.0074}, {0.0000010503, 1.0000, -0.00018407}, {0.0000000026247, 0.0025, 1.0000}};
static const float    B[3] = {0.0000010503, 0.0000000026247, 0.0000000000032796};
static const float    C[3] = {2.2332, -0.7369, -0.6200};
static const float    D = 0.0;

// monitoring parameters
static const uint32_t  window = 11*10;   // window size x 10 (11 * 10)
static const float  threshold = 10262;  // threshold
// =================================================================================================================

// invariant state variables
static float    inv_y = 0.0f;;
static float    inv_x[3] = {0.0f, 0.0f, 0.0f};

// accumulated errors
static float    err_sum = 0;

// local variables
static uint32_t w_index = 0;    // current index in window
static float    max_mse = 0;    // max error (for logs)

// attack states
static bool     attack_detected = false;


void Copter::copter_invariants_check(float target, int measured)
{

    //--------------- invariant start
    // y = Cx[i] + Du[i]
    inv_y = (C[0]*inv_x[0] + C[1]*inv_x[1] + C[2]*inv_x[2]) + D*target;

    // x' = Ax[i] + Bu[i]
    float x0 = (A[0][0]*inv_x[0] + A[0][1]*inv_x[1] + A[0][2]*inv_x[2]) + B[0]*target;
    float x1 = (A[1][0]*inv_x[0] + A[1][1]*inv_x[1] + A[1][2]*inv_x[2]) + B[1]*target;
    float x2 = (A[2][0]*inv_x[0] + A[2][1]*inv_x[1] + A[2][2]*inv_x[2]) + B[2]*target;
    inv_x[0] = x0;
    inv_x[1] = x1;
    inv_x[2] = x2;

    // update invariants (for logs)
    AP::ahrs().set_invariant((int32_t)inv_y); 

    // error calculation
    float error = inv_y - measured;     // error at each time point
    err_sum += error*error;            // accumulated squared-error
    float mse = err_sum / (w_index + 1);        // mse in the current window
    AP::ahrs().set_ierror(mse);

    if(mse > max_mse) {     // maximum mse (for analysis)
        max_mse = mse;
    }

    w_index++;
    if(w_index >= window) {   // new window
        w_index = 0;
        err_sum = 0;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && 0
    // for logging
    uint64_t timestamp = AP_HAL::micros64();
    FILE *out_file = fopen("sim_log.csv", "a");
    fprintf(out_file, "%" PRIu64 ", %f, %f, %d\n", \
            timestamp, target, inv_y, measured);
    fclose(out_file);
#endif

    // check error
    if(mse > threshold && g.choi_ci == 1) {
        // raise an alram
        attack_detected = true;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "!!!ATTACK DETECTED!!!");

        // TODO recovery actions 

    }
}

