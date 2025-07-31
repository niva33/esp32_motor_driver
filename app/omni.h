#ifndef OMNI_H_
#define OMNI_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "driver/pulse_cnt.h"

#include "omni_bsp.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct
{
    struct 
    {
        bdc_motor_handle_t m_bdc_motor_m0;
        pcnt_unit_handle_t m_encoder_m0;
        pid_ctrl_block_handle_t m_pid_ctrl_m0;
    }drv;

} omni_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


extern omni_t* g_omni_app_default;

esp_err_t omni_init();


#ifdef __cplusplus
}
#endif


#endif