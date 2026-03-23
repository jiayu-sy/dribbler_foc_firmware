/* Algorithm layer with core computational blocks */
#include "foc.h"
#include "pi.h"
#include "math/fast_math.h"
#include "bsp/bsp.h"
#include "mc_config.h"
#include "config.h"

#define SECTOR_1  0u
#define SECTOR_2  1u
#define SECTOR_3  2u
#define SECTOR_4  3u
#define SECTOR_5  4u
#define SECTOR_6  5u

u8 svpwm_get_duty(float alpha, float beta, float *duty) {
	float vabs = fabsf(alpha) + fabsf(beta);
	if (vabs < 1e-6f) {
		duty[0] = 0.5f;
		duty[1] = 0.5f;
		duty[2] = 0.5f;
		return 0xFF;
	}

	uint8_t sector = 0xFF;
	float   tA = 0.5f, tB = 0.5f, tC = 0.5f;  /* defensive init */
	float   X, Y, Z;

	if (beta >= 0) {
		if (alpha >= 0) {
			//quadrant I
			if ((ONE_BY_SQRT3 * beta) > alpha) {
				sector = SECTOR_2;
			} else {
				sector = SECTOR_1;
			}
		} else {
			//quadrant II
			if ((-ONE_BY_SQRT3 * beta) > alpha) {
				sector = SECTOR_3;
			} else {
				sector = SECTOR_2;
			}
		}
	} else {
		if (alpha >= 0) {
			//quadrant IV5
			if ((-ONE_BY_SQRT3 * beta) > alpha) {
				sector = SECTOR_5;
			} else {
				sector = SECTOR_6;
			}
		} else {
			//quadrant III
			if ((ONE_BY_SQRT3 * beta) > alpha) {
				sector = SECTOR_4;
			} else {
				sector = SECTOR_5;
			}
		}
	}

	X = (SQRT3 * beta);
	Y = 1.5f * alpha + (SQRT3_BY_2 * beta);
	Z = -1.5f * alpha + (SQRT3_BY_2 * beta);
	switch(sector) {
		case SECTOR_1: // 3
		{	
			// Vector on-times
            float T4 = -Z;
			float T6 = X;
		#if CONFIG_PWM_HIGH_FIRST
			tC = 0.5f * (1.0f - T4 - T6); 
        	tB = tC + T6;
        	tA = tB + T4;
		#else
			tA = 0.5f * (1.0f - T4 - T6);
			tB = tA + T4;
			tC = tB + T6;
		#endif
			break;
		}
		case SECTOR_2: // 1
		{
            // Vector on-times
			float T6 = Y;
			float T2 = Z;
		#if CONFIG_PWM_HIGH_FIRST
			tC = 0.5f * (1.0f - T6 - T2);
        	tA = tC + T6;
        	tB = tA + T2;
		#else
			tB = 0.5f * (1.0f - T6 - T2);
			tA = tB + T2;
			tC = tA + T6;
		#endif
			break;
		}
		case SECTOR_3: // 5
		{
            // Vector on-times
			float T2 = X;
			float T3 = -Y;
		#if CONFIG_PWM_HIGH_FIRST
			tA = 0.5f * (1.0f - T2 - T3);
			tC = tA + T3;
			tB = tC + T2;
		#else
			tB = 0.5f * (1.0f - T2 - T3);
			tC = tB + T2;
			tA = tC + T3;
		#endif
			break;
		}
		case SECTOR_4: // 4
		{
            // Vector on-times
			float T1 = -X;
			float T3 = Z;
		#if CONFIG_PWM_HIGH_FIRST
			tA = 0.5f * (1.0f - T1 - T3);
			tB = tA + T3;
			tC = tB + T1;
		#else
			tC = 0.5f * (1.0f - T1 - T3);
			tB = tC + T1;
			tA = tB + T3;
		#endif
			break;
		}
		case SECTOR_5: // 6
		{
            // Vector on-times
			float T1 = -Y;
			float T5 = -Z;
		#if CONFIG_PWM_HIGH_FIRST
			tB = 0.5f * (1.0f - T1 - T5);
			tA = tB + T5;
			tC = tA + T1;
		#else
			tC = 0.5f * (1.0f - T1 - T5);
			tA = tC + T1;
			tB = tA + T5;
		#endif
			break;
		}					
		case SECTOR_6: // 2
		{
            // Vector on-times
			float T4 = Y;
			float T5 = -X;
		#if CONFIG_PWM_HIGH_FIRST
			tB = 0.5f * (1.0f - T4 - T5);
			tC = tB + T5;
			tA = tC + T4;
		#else
			tA = 0.5f * (1.0f - T4 - T5);
			tC = tA + T4;
			tB = tC + T5;
		#endif
			break;
		}
		default:
			break;
	}	
	duty[0] = clamp_f(tA, 0, 1.0f);
	duty[1] = clamp_f(tB, 0, 1.0f);
	duty[2] = clamp_f(tC, 0, 1.0f);
	return sector;
}


void foc_init(foc_t *foc) {
    pi_reset(&foc->pi_con_id, 0);
    pi_reset(&foc->pi_con_iq, 0);

    foc->e_velocity_filted = 0;
    foc->id_set = 0;
    foc->iq_set = 0;
    foc->vd_set = 0;
    foc->vq_set = 0;
}

void foc_update_svpwm(foc_t *foc) {
    float s = sinf(foc->theta_e);
    float c = cosf(foc->theta_e);

    if (foc->vdc <= 0.0f) {
        foc->duty_norm[0] = 0;
        foc->duty_norm[1] = 0;
        foc->duty_norm[2] = 0;
        return;
    }

    float max_mag = foc->vdc * (ONE_BY_SQRT3 * CONFIG_SVM_MODULATION);

    if (!foc->b_openloop) {
        float max_vd = max_mag;
        foc->pi_con_id.max = max_vd;
        foc->pi_con_id.min = -max_vd;

        float err = foc->id_set - foc->id_mea;
        foc->vd_out = pi_run(&foc->pi_con_id, err, 0);

        float max_vq = sqrtsub2_f(max_vd, foc->vd_out);
        foc->pi_con_iq.max = max_vq;
        foc->pi_con_iq.min = -max_vq;

        err = foc->iq_set - foc->iq_mea;
        foc->vq_out = pi_run(&foc->pi_con_iq, err, 0);
    } 
    else {
        float max_vd = max_mag;
        foc->vd_out = clamp_f(foc->vd_set, -max_vd, max_vd);
        float max_vq = sqrtsub2_f(max_vd, foc->vd_out);
        foc->vq_out = clamp_f(foc->vq_set, -max_vq, max_vq);
    }

    park_inv(foc->vd_out, foc->vq_out, s, c, &foc->valpha_out, &foc->vbeta_out);
    float valpha_uni = foc->valpha_out / foc->vdc;
    float vbeta_uni = foc->vbeta_out / foc->vdc;

    foc->sector = svpwm_get_duty(valpha_uni, vbeta_uni, &foc->duty_norm[0]);
}
