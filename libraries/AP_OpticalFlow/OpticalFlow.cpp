#include <AP_BoardConfig/AP_BoardConfig.h>
#include "OpticalFlow.h"
#include "AP_OpticalFlow_Onboard.h"
#include "AP_OpticalFlow_SITL.h"
#include "AP_OpticalFlow_Pixart.h"
#include "AP_OpticalFlow_PX4Flow.h"
#include "AP_OpticalFlow_CXOF.h"
#include "AP_OpticalFlow_MAV.h"
#include "AP_OpticalFlow_HereFlow.h"
#include "AP_OpticalFlow_MSP.h"
#include "AP_OpticalFlow_UPFLOW.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#ifndef OPTICAL_FLOW_TYPE_DEFAULT
 #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412 || defined(HAL_HAVE_PIXARTFLOW_SPI)
  #define OPTICAL_FLOW_TYPE_DEFAULT OpticalFlowType::PIXART
 #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
  #define OPTICAL_FLOW_TYPE_DEFAULT OpticalFlowType::BEBOP
 #else
  #define OPTICAL_FLOW_TYPE_DEFAULT OpticalFlowType::NONE
 #endif
#endif

const AP_Param::GroupInfo OpticalFlow::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Optical flow sensor type
    // @Description: Optical flow sensor type
    // @Values: 0:None, 1:PX4Flow, 2:Pixart, 3:Bebop, 4:CXOF, 5:MAVLink, 6:UAVCAN, 7:MSP, 8:UPFLOW
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 0,  OpticalFlow,    _type,   (int8_t)OPTICAL_FLOW_TYPE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: _FXSCALER
    // @DisplayName: X axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FXSCALER", 1,  OpticalFlow,    _flowScalerX,   0),

    // @Param: _FYSCALER
    // @DisplayName: Y axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FYSCALER", 2,  OpticalFlow,    _flowScalerY,   0),

    // @Param: _ORIENT_YAW
    // @DisplayName: Flow sensor yaw alignment
    // @Description: Specifies the number of centi-degrees that the flow sensor is yawed relative to the vehicle. A sensor with its X-axis pointing to the right of the vehicle X axis has a positive yaw angle.
    // @Units: cdeg
    // @Range: -17999 +18000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("_ORIENT_YAW", 3,  OpticalFlow,    _yawAngle_cd,   0),

    // @Param: _POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the optical flow sensor focal point in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the optical flow sensor focal point in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the optical flow sensor focal point in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_POS", 4, OpticalFlow, _pos_offset, 0.0f),

    // @Param: _ADDR
    // @DisplayName: Address on the bus
    // @Description: This is used to select between multiple possible I2C addresses for some sensor types. For PX4Flow you can choose 0 to 7 for the 8 possible addresses on the I2C bus.
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("_ADDR", 5,  OpticalFlow, _address,   0),

    //PADLOCK
    // @Param: _PDLK_X
    // @DisplayName: Replace rotation about x axis
    // @Description: Value to replace actual flow rate X reading with. Indicates front/back tilt and compensates for body rate.
    // @Units: rad/s
    // @Range: -3.14 3.14
    // @User: Advanced
    AP_GROUPINFO("_PDLK_X", 6,  OpticalFlow, _pdlk_attack_x,   0),

    // @Param: _PDLK_Y
    // @DisplayName: Replace rotation about x axis
    // @Description: Value to replace actual flow rate X reading with. Indicates front/back tilt and compensates for body rate.
    // #Units: rad/s
    // @Range: -3.14 3.14
    // @User: Advanced
    AP_GROUPINFO("_PDLK_Y", 7,  OpticalFlow, _pdlk_attack_y,   0),

    // @Param: _PDLK_ENABLE
    // @DisplayName: Enable OF spoofing
    // @Description: Enables or Disables optical flow spoofing.
    // @Values: 0:Disabled 1:Enabled Other:Disabled
    // @User: Advanced
    AP_GROUPINFO("_PDLK_ATK", 8,  OpticalFlow, _pdlk_attack_enable,   0),

    // @Param: _PDLK_ADV_ATK
    // @DisplayName: Enable Reactive Attacker for OF
    // @Description: Enable or Disable Reactive Attacker for OF.
    // @Values: 0:Disabled 1:Enabled Other:Disabled
    // @User: Advanced
    AP_GROUPINFO("_PDLK_ADV_ATK", 9,  OpticalFlow, _pdlk_adv_atk,   0),
    
    AP_GROUPEND
};

// default constructor
OpticalFlow::OpticalFlow()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

void OpticalFlow::init(uint32_t log_bit)
{
     _log_bit = log_bit;

    // return immediately if not enabled or backend already created
    if ((_type == (int8_t)OpticalFlowType::NONE) || (backend != nullptr)) {
        return;
    }

    switch ((OpticalFlowType)_type.get()) {
    case OpticalFlowType::NONE:
        break;
    case OpticalFlowType::PX4FLOW:
        backend = AP_OpticalFlow_PX4Flow::detect(*this);
        break;
    case OpticalFlowType::PIXART:
        backend = AP_OpticalFlow_Pixart::detect("pixartflow", *this);
        if (backend == nullptr) {
            backend = AP_OpticalFlow_Pixart::detect("pixartPC15", *this);
        }
        break;
    case OpticalFlowType::BEBOP:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
        backend = new AP_OpticalFlow_Onboard(*this);
#endif
        break;
    case OpticalFlowType::CXOF:
        backend = AP_OpticalFlow_CXOF::detect(*this);
        break;
    case OpticalFlowType::MAVLINK:
        backend = AP_OpticalFlow_MAV::detect(*this);
        break;
    case OpticalFlowType::UAVCAN:
#if HAL_ENABLE_LIBUAVCAN_DRIVERS
        backend = new AP_OpticalFlow_HereFlow(*this);
#endif
        break;
    case OpticalFlowType::MSP:
#if HAL_MSP_OPTICALFLOW_ENABLED
        backend = AP_OpticalFlow_MSP::detect(*this);
#endif
        break;
    case OpticalFlowType::UPFLOW:
        backend = AP_OpticalFlow_UPFLOW::detect(*this);
        break;
    case OpticalFlowType::SITL:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        backend = new AP_OpticalFlow_SITL(*this);
#endif
        break;
    }

    if (backend != nullptr) {
        backend->init();
    }
}

void OpticalFlow::update(void)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }
    if (backend != nullptr) {
        backend->update();
    }

    // only healthy if the data is less than 0.5s old
    _flags.healthy = (AP_HAL::millis() - _last_update_ms < 500);
}

void OpticalFlow::handle_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    if (backend != nullptr) {
        backend->handle_msg(msg);
    }
}

#if HAL_MSP_OPTICALFLOW_ENABLED
void OpticalFlow::handle_msp(const MSP::msp_opflow_data_message_t &pkt)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    if (backend != nullptr) {
        backend->handle_msp(pkt);
    }
}
#endif //HAL_MSP_OPTICALFLOW_ENABLED

void OpticalFlow::update_state(const OpticalFlow_state &state)
{
    _state = state;
    _last_update_ms = AP_HAL::millis();

    // write to log and send to EKF if new data has arrived
    AP::ahrs().writeOptFlowMeas(quality(),
                                _state.flowRate,
                                _state.bodyRate,
                                _last_update_ms,
                                get_pos_offset());
    Log_Write_Optflow();
}

void OpticalFlow::Log_Write_Optflow()
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }
    if (_log_bit != (uint32_t)-1 &&
        !logger->should_log(_log_bit)) {
        return;
    }

    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        time_us         : AP_HAL::micros64(),
        surface_quality : _state.surface_quality,
        flow_x          : _state.flowRate.x,
        flow_y          : _state.flowRate.y,
        body_x          : _state.bodyRate.x,
        body_y          : _state.bodyRate.y
    };
    logger->WriteBlock(&pkt, sizeof(pkt));
}



// singleton instance
OpticalFlow *OpticalFlow::_singleton;

namespace AP {

OpticalFlow *opticalflow()
{
    return OpticalFlow::get_singleton();
}

}
