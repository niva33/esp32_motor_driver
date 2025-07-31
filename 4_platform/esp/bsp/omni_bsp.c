#include "bdc_motor.h"



/*******************************************************************************
* Definitions
******************************************************************************/
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

#define BDC_ENCODER_PCNT_HIGH_LIMIT     1000
#define BDC_ENCODER_PCNT_LOW_LIMIT      -1000
#define BDC_ENCODER_MAX_GLITCH_NS       1000

#define BDC_ENCODER_M0_GPIO_A
#define BDC_ENCODER_M0_GPIO_B




/*******************************************************************************
* Prototypes
******************************************************************************/
static void omni_io_init();
static void omni_timer_init();
static void omni_bdc_motor_init();
static void omni_encoder_init();

/*******************************************************************************
* Variables
******************************************************************************/
static gptimer_handle_t s_gptimer_0;

static gptimer_handle_t s_gptimer_1;

static bdc_motor_handle_t s_bdc_motor_m0;

// static bdc_motor_handle_t s_bdc_motor_m1;

// static bdc_motor_handle_t s_bdc_motor_m2;

static pcnt_unit_handle_t s_encoder_m0;

// static pcnt_unit_handle_t s_encoder_m1;

// static pcnt_unit_handle_t s_encoder_m2;





/*******************************************************************************
* Code
******************************************************************************/
static void omni_io_init()
{
    gpio_config_t io_conf = {};

    //W5500 SPI CS
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);


}

static void omni_timer_init()
{
    gptimer_config_t timer_config_0 = 
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config_0, &s_gptimer_0));

    gptimer_config_t timer_config_1 = 
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config_1, &s_gptimer_1));

}

static void omni_bdc_motor_init()
{
    bdc_motor_config_t bdc_motor_m0_config = 
    {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = 13,
        .shift_reg.data = 15,
        .shift_reg.clock_gpio_num = 4,
        .shift_reg.latch_gpio_num = 11 
    };

    bdc_motor_mcpwm_config_t bdc_motor_m0_mcpwm_config =
    {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ

    };

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&bdc_motor_m0_config,
                                               &bdc_motor_m0_mcpwm_config,
                                               &s_bdc_motor_m0));

}

static void omni_encoder_init()
{
    pcnt_unit_config_t encoder_m0_config = 
    {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&encoder_m0_config, &s_encoder_m0));
    
    pcnt_glitch_filter_config_t encoder_m0_filter_config = 
    {
        .max_glitch_ns = BDC_ENCODER_MAX_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_encoder_m0, &filter_config));

    pcnt_chan_config_t chan_a_config = 
    {
        .edge_gpio_num = BDC_ENCODER_M0_GPIO_A,
        .level_gpio_num = BDC_ENCODER_M0_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m0, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = 
    {
        .edge_gpio_num = BDC_ENCODER_M0_GPIO_B,
        .level_gpio_num = BDC_ENCODER_M0_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m0, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m0, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_encoder_m0, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_encoder_m0, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(s_encoder_m0));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_encoder_m0));
    ESP_ERROR_CHECK(pcnt_unit_start(s_encoder_m0));
}






