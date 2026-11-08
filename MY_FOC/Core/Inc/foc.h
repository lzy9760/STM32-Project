#ifndef __FOC_H__
#define __FOC_H__

#include <stdint.h>
#include "motor1_drive.h"

extern volatile float foc_uq;
extern volatile float foc_ud;
extern volatile float foc_target_id;
extern volatile float foc_target_iq;
extern volatile float foc_iq; /* Latest running-loop Iq feedback, in amperes. */
extern volatile float foc_pp;
extern volatile float foc_dir; /* 编码器相对定子磁场的方向，由校准确定 */
extern volatile float enc_offset_deg;
extern volatile uint8_t foc_aligned;    
extern volatile float foc_speed_rad_s; /* Observer speed used by the speed PI, rad/s. */
extern volatile float foc_speed_raw_rad_s;
extern volatile float foc_speed_error_deg;
extern volatile float foc_speed_dt_s;
extern volatile float foc_target_speed_rad_s;
extern volatile float foc_pos_deg;
extern volatile float foc_pos_turn;
extern volatile float foc_target_pos_turn;
extern volatile float foc_pos_error_turn;

void foc_curr_loop_nb(void);
float foc_get_elec_angle(void);
void foc_pos_loop_nb(void);
void foc_speed_loop_nb(void);
void foc_update_speed(void);


#endif

