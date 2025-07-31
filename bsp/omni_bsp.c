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
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

#define BDC_ENCODER_PCNT_HIGH_LIMIT     1000
#define BDC_ENCODER_PCNT_LOW_LIMIT      -1000
#define BDC_ENCODER_MAX_GLITCH_NS       1000

#define SHIFT_REG_DS_GPIO               GPIO_NUM_22
#define SHIFT_REG_CLK_GPIO              GPIO_NUM_26
#define SHIFT_REG_LATCH_GPIO            GPIO_NUM_27


#define BDC_ENCODER_M0_GPIO_A           GPIO_NUM_13
#define BDC_ENCODER_M0_GPIO_B           GPIO_NUM_25
#define BDC_PWM_M0_GPIO                 GPIO_NUM_16

#define W5500_SPI_CS_GPIO               GPIO_NUM_5


//esp32 s3
// #define SHIFT_REG_DS_GPIO               GPIO_NUM_9
// #define SHIFT_REG_CLK_GPIO              GPIO_NUM_10
// #define SHIFT_REG_LATCH_GPIO            GPIO_NUM_11


// #define BDC_ENCODER_M0_GPIO_A           GPIO_NUM_12
// #define BDC_ENCODER_M0_GPIO_B           GPIO_NUM_13
// #define BDC_PWM_M0_GPIO                 GPIO_NUM_48

// #define W5500_SPI_CS_GPIO               GPIO_NUM_5


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
static gptimer_handle_t s_gptimer_0 = NULL;

static gptimer_handle_t s_gptimer_1 = NULL;

static bdc_motor_handle_t s_bdc_motor_m0 = NULL;

static bdc_motor_handle_t s_bdc_motor_m1 = NULL;

static bdc_motor_handle_t s_bdc_motor_m2 = NULL;

static pcnt_unit_handle_t s_encoder_m0 = NULL;

static pcnt_unit_handle_t s_encoder_m1 = NULL;

static pcnt_unit_handle_t s_encoder_m2 = NULL;





/*******************************************************************************
* Code
******************************************************************************/
static void omni_io_init()
{
    gpio_config_t io_conf = {};

    //W5500 SPI CS
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = W5500_SPI_CS_GPIO;
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
        .pwma_gpio_num = BDC_PWM_M0_GPIO,
        .shift_reg.data_gpio_num = SHIFT_REG_DS_GPIO,
        .shift_reg.clock_gpio_num = SHIFT_REG_CLK_GPIO,
        .shift_reg.latch_gpio_num = SHIFT_REG_LATCH_GPIO 
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
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_encoder_m0, &encoder_m0_filter_config));

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

void omni_bsp_init()
{
    omni_io_init();
    omni_timer_init();
    // omni_bdc_motor_init();
    // omni_encoder_init();
    ESP_LOGI("DONE", "install pcnt channels");
}


ptimer_handle_t omni_get_timer(uint8_t _channel)
{
    if(_channel == OMNI_TIMER_0)
    {
        return s_gptimer_0;
    }
    else if(_channel == OMNI_TIMER_1)
    {
        return s_gptimer_1;
    }
    return NULL;
}

bdc_motor_handle_t omni_get_bdc_motor(uint8_t _motor_index)
{
    if(_channel == OMNI_BDC_MOTOR_M0)
    {
        return s_bdc_motor_m0;
    }
    else if(_channel == OMNI_BDC_MOTOR_M1)
    {
        return s_bdc_motor_m1;
    }
    else if(_channel == OMNI_BDC_MOTOR_M2)
    {
        return s_bdc_motor_m2;
    }
    return NULL;
}

pcnt_unit_handle_t omni_get_encoder(uint8_t _motor_index)
{
    if(_channel == OMNI_BDC_MOTOR_M0)
    {
        return s_encoder_m0;
    }
    else if(_channel == OMNI_BDC_MOTOR_M1)
    {
        return s_encoder_m1;
    }
    else if(_channel == OMNI_BDC_MOTOR_M2)
    {
        return s_encoder_m2;
    }
    return NULL;
}



