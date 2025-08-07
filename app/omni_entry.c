#include "omni.h"
#include "omni_bsp.h"
#include "esp_timer.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BDC_PID_EXPECT_SPEED (400u)

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
    int cur_pulse_count = 0;
    omni_t* temp = g_omni_app_default;
    ESP_ERROR_CHECK(pcnt_unit_get_count(g_omni_app_default->drv.m_encoder_m0, &cur_pulse_count));
    // int32_t real_pulse = cur_pulse_count - last_pulse_count;
    // last_pulse_count = cur_pulse_count;

    // float error = (float)BDC_PID_EXPECT_SPEED - real_pulse;

    // float new_speed = 0;
    // pid_compute(g_omni_app_default->drv.m_pid_ctrl_m0, error, &new_speed);
    bdc_motor_set_speed(g_omni_app_default->drv.m_bdc_motor_m0, 125u);
    
    ESP_LOGI("siuuu", "This is PID callback %d",cur_pulse_count);

}

void omni_entry()
{
    omni_bsp_init();

    ESP_LOGI("siuuu", "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = omni_main_proc,
        .arg = NULL,
        .name = "pid_loop"
    };

    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));
    omni_init();
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, 10 * 1000));
    gpio_set_level(GPIO_NUM_26, 1);

    // while(1)
    // {
    //     uart_read_bytes(CONSOLE_UART_PORT_NUM, &c, 1U, 20 / portTICK_PERIOD_MS);
    // }

}

