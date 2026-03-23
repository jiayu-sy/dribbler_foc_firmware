/* global configuration file */
#ifndef _CONFIG_H__
#define _CONFIG_H__

/* dq loop ramp count */
#ifndef CONFIG_DQ_LOOP_RAMP_COUNT
#define CONFIG_DQ_LOOP_RAMP_COUNT 8
#endif

/* speed loop frequency (Hz) */
#ifndef CONFIG_SPEED_CTRL_FREQ
#define CONFIG_SPEED_CTRL_FREQ 1000
#endif

/* simulation steps per PWM period (Ts_ctrl / Ts) */
#ifndef CONFIG_SIM_STEPS_PER_PWM
#define CONFIG_SIM_STEPS_PER_PWM 200
#endif

#ifndef CONFIG_DISABLE_RAMP
#define CONFIG_DISABLE_RAMP 0
#endif

/* PWM polarity: 1 = high first (match PWM_MODE1), 0 = low first */
#ifndef CONFIG_PWM_HIGH_FIRST
#define CONFIG_PWM_HIGH_FIRST 1
#endif

/* motor parames */
#ifndef CONFIG_MOTOR_PARAMS_Poles
#define CONFIG_MOTOR_PARAMS_Poles 7
#endif
#ifndef CONFIG_MOTOR_PARAMS_Ld
#define CONFIG_MOTOR_PARAMS_Ld 50e-6f      /* dribbler: Ls = 50 muH (SPMSM, Ld=Lq) */
#endif
#ifndef CONFIG_MOTOR_PARAMS_Lq
#define CONFIG_MOTOR_PARAMS_Lq 50e-6f      /* dribbler: Ls = 50 muH (SPMSM, Ld=Lq) */
#endif
#ifndef CONFIG_MOTOR_PARAMS_Rs
#define CONFIG_MOTOR_PARAMS_Rs 0.3f        /* dribbler: phase resistance 0.3 Ohm */
#endif
#ifndef CONFIG_MOTOR_PARAMS_Flux
#define CONFIG_MOTOR_PARAMS_Flux 363e-6f   /* dribbler: phi_f = Kt/(1.5*p) = 0.00381/10.5 */
#endif
#ifndef CONFIG_MOTOR_PARAMS_J
#define CONFIG_MOTOR_PARAMS_J 0.0020f
#endif
#ifndef CONFIG_MOTOR_PARAMS_MAX_RPM
#define CONFIG_MOTOR_PARAMS_MAX_RPM 60000  /* dribbler: Kv2500 × 24V = 60000 RPM */
#endif
#ifndef CONFIG_MOTOR_ENCODER_OFFSET
#define CONFIG_MOTOR_ENCODER_OFFSET 0.0f
#endif

/* invert electrical angle in controller */
#ifndef CONFIG_THETA_INVERT
#define CONFIG_THETA_INVERT 1
#endif

#ifndef CONFIG_THETA_OFFSET_RAD
#define CONFIG_THETA_OFFSET_RAD 0.0f
#endif

/* current polarity (1 or -1) applied in motor sample path */
#ifndef CONFIG_CURRENT_POLARITY
#define CONFIG_CURRENT_POLARITY 1.0f
#endif

#endif /* _CONFIG_H__ */
