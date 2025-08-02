#include "omni.h"
#include "omni_bsp.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BDC_PID_EXPECT_SPEED 400;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/

static void omni_main_proc(void* _arg)
{
    (void)_arg;
    static int32_t last_pulse_count = 0;
    int32_t cur_pulse_count = 0;
    pcnt_unit_get_count(g_omni_app_default->drv.m_encoder_m0, &cur_pulse_count);
    int32_t real_pulse = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;

    float error = BDC_PID_EXPECT_SPEED - real_pulse;
    float new_speed = 0;
    pid_compute(g_omni_app_default->drv.m_pid_ctrl_m0, errror, &new_speed);
    bdc_motor_set_speed(g_omni_app_default->drv.m_bdc_motor_m0, new_speed);
}

void omni_entry()
{
    omni_bsp_init();

    gptimer_handle_t main_thread = omni_get_timer(0);
    gptimer_event_callbacks_t cbs =
    {
        .on_alarm = omni_main_proc;
    }
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(main_thread, &cbs, queue));
    ESP_ERROR_CHECK(gptimer_enable(main_thread));
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 10000, // period = 10ms
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(main_thread, &alarm_config1));

    omni_init();

    ESP_ERROR_CHECK(gptimer_start(main_thread));

}