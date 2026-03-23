#include "mc_config.h"
#include "config.h"
#include "bsp/bsp.h"

mc_config_t _mc_conf;

void mc_conf_init(void) {
    _mc_conf.pwm_freq = CONFIG_FOC_PWM_FREQ;
    _mc_conf.pwm_period = CONFIG_FOC_PWM_Period;
    _mc_conf.pwm_half_period = _mc_conf.pwm_period / 2;
    _mc_conf.ts = 1.0f / _mc_conf.pwm_freq;
#if CONFIG_FOC_PWM_UPDATE_REPEAT == 0
    _mc_conf.ts = _mc_conf.ts / 2.0f;
#endif
    _mc_conf.slow_ts = 1.0f / (float)CONFIG_SPEED_CTRL_FREQ;

    _mc_conf.motor_params.poles = CONFIG_MOTOR_PARAMS_Poles;
    _mc_conf.motor_params.ld = CONFIG_MOTOR_PARAMS_Ld;
    _mc_conf.motor_params.lq = CONFIG_MOTOR_PARAMS_Lq;
    _mc_conf.motor_params.rs = CONFIG_MOTOR_PARAMS_Rs;
    _mc_conf.motor_params.flux = CONFIG_MOTOR_PARAMS_Flux;
    _mc_conf.motor_params.J = CONFIG_MOTOR_PARAMS_J;
    _mc_conf.motor_params.max_rpm = CONFIG_MOTOR_PARAMS_MAX_RPM;
    _mc_conf.motor_params.encoder_offset = CONFIG_MOTOR_ENCODER_OFFSET;

    _mc_conf.board_params.vbus_voltage_max = CONFIG_HW_MAX_DC_VOL;
    _mc_conf.board_params.vbus_voltage_min = CONFIG_HW_MIN_DC_VOL;
    _mc_conf.board_params.vbus_curr_max = CONFIG_HW_MAX_DC_CURRENT;
    _mc_conf.board_params.phase_curr_max = CONFIG_HW_MAX_PHASE_CURR;


    /* dq current PI: bandwidth 500 Hz
     * kp = 0.157
     * ki = 942   */
    _mc_conf.control_params.pid[pid_id].kp = 0.157f;
    _mc_conf.control_params.pid[pid_id].ki = 942.0f;

    _mc_conf.control_params.pid[pid_iq].kp = 0.157f;
    _mc_conf.control_params.pid[pid_iq].ki = 942.0f;

    _mc_conf.control_params.pid[pid_vel].kp = 0.01f;
    _mc_conf.control_params.pid[pid_vel].ki = 0.001f;
}
