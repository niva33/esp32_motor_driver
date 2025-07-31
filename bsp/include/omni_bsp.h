#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
enum
{
    OMNI_TIMER_0 = 0,
    OMNI_TIMER_1 = 1,
    OMNI_TIMER_2,
    OMNI_TIMER_NUMBER
};

enum
{
    OMNI_BDC_MOTOR_M0 = 0,
    OMNI_BDC_MOTOR_M1 = 1,
    OMNI_BDC_MOTOR_M2,
    OMNI_BDC_MOTOR_NUMBER
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
esp_err_t omni_bsp_init();

ptimer_handle_t omni_get_timer(uint8_t _channel);

bdc_motor_handle_t omni_get_bdc_motor(uint8_t _motor_index);

pcnt_unit_handle_t omni_get_encoder(uint8_t _motor_index);






