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
void esp_console_reg(void);
static int pid_tuning_proc_test_fn(int argc, char** argv);



/*******************************************************************************
 * Variables
 ******************************************************************************/
esp_timer_handle_t s_pid_loop_timer = NULL;

float g_fKp = 0;
float g_fKi = 0;
float g_fKd = 0;


/** Arguments used by 'join' function */
static struct {
    struct arg_int *kp;
    struct arg_int *ki;
    struct arg_int *kd;
    struct arg_end *end;
} s_pid_args;



/*******************************************************************************
 * Code
 ******************************************************************************/

static void pid_tuning_proc(void* _arg)
{
    (void)_arg;
    float error;
    float new_speed;
    int real_pulse;
    static uint8_t count = 0;
    static uint8_t sp[] = {10, 25, 5, 20};
    static uint8_t index = 0;
    static int32_t last_pulse_count = 0;


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
        bdc_motor_set_speed(g_omni_app_default->drv.m_bdc_motor_m0, 0u);
        pid_reset_ctrl_block(g_omni_app_default->drv.m_pid_ctrl_m0);
        pcnt_unit_clear_count(g_omni_app_default->drv.m_encoder_m0);
        error = 0;
        real_pulse = 0;
        index = 0;
        new_speed = 0;
        last_pulse_count = 0;
        esp_restart();
        return;
    }

    //start pid compute
    int cur_pulse_count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(g_omni_app_default->drv.m_encoder_m0, &cur_pulse_count));
    real_pulse = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;

    error = (float)sp[index] - real_pulse;

    new_speed = 0;
    pid_compute(g_omni_app_default->drv.m_pid_ctrl_m0, error, &new_speed);
    bdc_motor_set_speed(g_omni_app_default->drv.m_bdc_motor_m0, new_speed);
    gpio_set_level(GPIO_NUM_26, 1);
    
    // ESP_LOGI("","");

    // printf("%d ", 20);
    printf("%d ", real_pulse);
    printf("%d\n", sp[index]);
    // printf("%d ", cur_pulse_count);

    // printf("%d\n", 30);

}

static int pid_tuning_proc_test_fn(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &s_pid_args);
    if(nerrors != 0)
    {
        arg_print_errors(stderr, s_pid_args.end, argv[0]);
        return -1;
    }

    g_fKp = s_pid_args.kp->ival[0];
    g_fKi = s_pid_args.ki->ival[0];
    g_fKd = s_pid_args.kd->ival[0];

    pid_ctrl_parameter_t new_params = 
    {
        .kp = g_fKp,
        .ki = g_fKi,
        .kd = g_fKd,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = 399,
        .min_output   = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_update_parameters(g_omni_app_default->drv.m_pid_ctrl_m0, &new_params);

    ESP_LOGI("PID_tuning", "Kp change to %f ", g_fKp);
    ESP_LOGI("PID_tuning", "Ki change to %f ", g_fKi);
    ESP_LOGI("PID_tuning", "Kd change to %f ", g_fKd);



    ESP_ERROR_CHECK(esp_timer_start_periodic(s_pid_loop_timer, 10 * 1000));




    return 0;
}   


void esp_console_reg(void)
{
    s_pid_args.kp = arg_int0(NULL, "kp", "<float>", "Kp value");
    s_pid_args.ki = arg_int0(NULL, "ki", "<float>", "Ki value");
    s_pid_args.kd = arg_int0(NULL, "kd", "<float>", "Kd value");
    s_pid_args.end = arg_end(3);

    const esp_console_cmd_t pid_cmd = 
    {
        .command = "pid",
        .help = "Change PID parameters",
        .hint = NULL,
        .func = pid_tuning_proc_test_fn,
        .argtable = &s_pid_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&pid_cmd) );
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

    // while(1)
    // {

    // }
}