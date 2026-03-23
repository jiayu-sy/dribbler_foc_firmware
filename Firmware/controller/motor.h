#ifndef _MOTOR_H__
#define _MOTOR_H__

#include "types.h"
#include "controller.h"
#include "hw_samples.h"
#include "bsp/bsp.h"
#if CONFIG_USE_HALL_ENCODER
#  include "bsp/stm32g431/bsp_hall.h"
#endif

#define PLOT_MAX_IDS 4

typedef struct {
    bool  b_start;
    u8    mode_running;
    float torque_command;
    float velocity_command;
    float vdq_command[2];
    float currdq_command[2];
    u8    plot_ids[PLOT_MAX_IDS];

    hw_samples_t samples;
    controller_t controller;
#if CONFIG_USE_HALL_ENCODER
    hall_t hall;
#endif
} motor_t;

void motor_init(motor_t *motor);
bool motor_start(motor_t *motor, u8 mode);
bool motor_stop(motor_t *motor);
void motor_adc_irq_handler(motor_t *motor);
void motor_adc_compute_duty(motor_t *motor, u16 *ta, u16 *tb, u16 *tc);
void timer_up_irq_handler(motor_t *motor);
void mc_sched_irq_handler(motor_t *motor);

extern motor_t g_motor[1];

static inline motor_t *motor(int index) {
    return &g_motor[index];
}

static inline controller_t *controller(motor_t *motor) {
    return &motor->controller;
}

static inline void motor_set_target_velocity(motor_t *motor, float velocity) {
    motor->velocity_command = velocity;
}

static inline void motor_set_target_torque(motor_t *motor, float torque) {
    motor->torque_command = torque;
}

static inline void motor_set_target_vdq(motor_t *motor, float vd, float vq) {
    motor->vdq_command[0] = vd;
    motor->vdq_command[1] = vq;
}

static inline void motor_set_target_idq(motor_t *motor, float id, float iq) {
    motor->currdq_command[0] = id;
    motor->currdq_command[1] = iq;
}

/* encoder hooks (sim uses sim_encoder_get_*) */
static inline void motor_init_encoder(motor_t *motor) {
    (void)motor;
#if (CONFIG_USE_HALL_ENCODER)
    hall_dwt_init();           /* enable DWT cycle counter for velocity timing */
    motor->hall.last_state = 0;
    motor->hall.valid      = false;
#elif (CONFIG_USE_MAG_ENCODER)
#endif
}

static inline void motor_update_encoder(motor_t *motor) {
    (void)motor;
#if (CONFIG_USE_HALL_ENCODER)
    hall_check_stall(&motor->hall);   /* zero velocity on stall, called at slow rate */
#elif (CONFIG_USE_MAG_ENCODER)
    bsp_mag_encoder_update();         /* latch ADC1 + compute velocity, called each FOC cycle */
#endif
}

static inline float motor_get_encoder_velocity(motor_t *motor) {
    (void)motor;
#if (CONFIG_USE_HALL_ENCODER)
    return hall_get_velocity(&motor->hall);
#elif (CONFIG_USE_MAG_ENCODER)
    return bsp_mag_encoder_get_velocity();
#else
    return sim_encoder_get_velocity();
#endif
}

static inline float motor_get_encoder_angle(motor_t *motor) {
    (void)motor;
#if (CONFIG_USE_HALL_ENCODER)
    return hall_get_elec_angle(&motor->hall);
#elif (CONFIG_USE_MAG_ENCODER)
    return bsp_mag_encoder_get_elec_angle();
#else
    return sim_encoder_get_angle();
#endif
}

#endif /* _MOTOR_H__ */
