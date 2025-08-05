#include "omni.h"
#include "omni_bsp.h"
#include "esp_timer.h"
#include "pid_tuning.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BDC_PID_EXPECT_SPEED (400u)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void pid_start_func();
static void pid_param_adj(int argc, char **argv);


/*******************************************************************************
 * Variables
 ******************************************************************************/
esp_timer_handle_t s_pid_loop_timer = NULL;

static tinysh_cmd_t g_pid_cmd = 
{
    .parent = NULL,
    .name = "pid",
    .help = "PID Testing",
    .usage = "",
    .function = NULL,
    .next = NULL,
    .child = NULL
};

static tinysh_cmd_t g_pid_start = 
{
    .parent = NULL,
    .name = "start",
    .help = "pid visualizing",
    .usage = "",
    .function = pid_start_func,
    .next = NULL,
    .child = NULL
};

static tinysh_cmd_t g_pid_param_adj =
{
    .parent = NULL,
    .name = "adj",
    .help = "change parameters Kp, Kd, Ki",
    .usage = "",
    .function = pid_param_adj,
    .next = NULL,
    .child = NULL
};




/*******************************************************************************
 * Code
 ******************************************************************************/

static void pid_tuning_proc(void* _arg)
{
    (void)_arg;

    static uint8_t count = 0;
    static uint8_t sp[] = {10, 25, 5, 20};
    static uint8_t index = 0;

    //change setpoint after 1s (100 tick, each tick 10ms)
    //stop at the last setpoint
    count++;
    if(count > 100)
    {
        count = 0;
        index = index + 1;
    }
    if(index > 3)
    {
        esp_timer_stop(s_pid_loop_timer);
    }

    //start pid compute
    static int32_t last_pulse_count = 0;
    int cur_pulse_count = 0;
    omni_t* temp = g_omni_app_default;
    ESP_ERROR_CHECK(pcnt_unit_get_count(g_omni_app_default->drv.m_encoder_m0, &cur_pulse_count));
    int32_t real_pulse = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;

    float error = (float)sp[index] - real_pulse;

    float new_speed = 0;
    pid_compute(g_omni_app_default->drv.m_pid_ctrl_m0, error, &new_speed);
    bdc_motor_set_speed(g_omni_app_default->drv.m_bdc_motor_m0, new_speed);
    
    ESP_LOGI("PID", "This is PID callback %d",cur_pulse_count);

}

void pid_testing()
{
    omni_bsp_init();

    ESP_LOGI("siuuu", "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_tuning_proc,
        .arg = NULL,
        .name = "pid_loop"
    };

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &s_pid_loop_timer));
    omni_init();

    char c;
    while(1)
    {
        uart_read_bytes(CONSOLE_UART_PORT_NUM, &c, 1U, 20 / portTICK_PERIOD_MS);
    }
}