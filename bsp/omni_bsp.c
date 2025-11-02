#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "w5500.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"


#include "omni_bsp.h"



/*******************************************************************************
* Definitions
******************************************************************************/
#define PROMPT_STR "uet_omni"

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"






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
static void initialize_filesystem(void);
static void initialize_nvs(void);


static void omni_io_init();
static void omni_timer_init();
static void omni_bdc_motor_init();
static void omni_encoder_init();
static void omni_pid_init();
static void omni_console_init();
static void omni_w5500_init();

//Callback function for w5500
static void omni_w5500_cs_select(void) { gpio_set_level(W5500_SPI_CS_GPIO, 0); }
static void omni_w5500_cs_deselect(void) { gpio_set_level(W5500_SPI_CS_GPIO, 1); }
static uint8_t omni_w5500_spi_readbyte(void);
static void omni_w5500_spi_writebyte(uint8_t wb);




// extern void esp_console_reg();



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

static pid_ctrl_block_handle_t s_pid_m0 = NULL;

static pid_ctrl_block_handle_t s_pid_m1 = NULL;

static pid_ctrl_block_handle_t s_pid_m2 = NULL;

static esp_console_repl_t *s_console_repl = NULL;

static spi_device_handle_t s_spi_handle = NULL;

static ema_t s_ema_m0;

static ema_t s_ema_m1;

static ema_t s_ema_m2;


static wiz_NetInfo s_w5500_server_info = 
{
    .mac = {0x03, 0x03, 0xDC, 0x12, 0x04, 0x56},
    .ip = {192, 168, 1, 103},
    .sn = {255, 255, 255, 0},
    .gw = {192, 168, 1, 1},
    .dns = {0, 0, 0, 0},
    .dhcp = NETINFO_STATIC
};

static omni_module_eth_info_t s_omni_module_eth_info[NUM_MODULES] = 
{
    //module 0
    {
        .mac = {0x30, 0xED, 0xA0, 0x10, 0xD2, 0xF8},
        .ip = {192, 168, 1, 100}
    },
    //module 1
    {
        .mac = {0x30, 0xED, 0xA0, 0x17, 0xD3, 0x0C},
        .ip = {192, 168, 1, 105}
    },
    //module 2
    {
        .mac = {0x30, 0xED, 0xA0, 0x17, 0xD2, 0xF8},
        .ip = {192, 168, 1, 110}
    },
    //module 3
    {
        .mac = {0x30, 0xED, 0xA0, 0x17, 0xD2, 0xE4},
        .ip = {192, 168, 1, 103}
    },
    //module 4
    {
        .mac = {0x30, 0xED, 0xA0, 0x17, 0xD2, 0xF0},
        .ip = {192, 168, 1, 102}
    },
    //module 5
    {
        .mac = {0x30, 0xED, 0xA0, 0x15, 0xD2, 0xF8},
        .ip = {192, 168, 1, 104}
    },
    //module 6
    {
        .mac = {0x30, 0xED, 0xA0, 0x16, 0xD2, 0xF8},
        .ip = {192, 168, 1, 105}
    },
    //module 7
    {
        .mac = {0x30, 0xED, 0xA0, 0x19, 0xD2, 0xF8},
        .ip = {192, 168, 1, 106}
    },
    //module 8
    {
        .mac = {0x30, 0xED, 0xA0, 0x21, 0xD2, 0xF8},
        .ip = {192, 168, 1, 107}
    },
    //module 9
    {
        .mac = {0x30, 0xED, 0xA0, 0x32, 0xD2, 0xF8},
        .ip = {192, 168, 1, 109}
    },
    //module 10
    {
        .mac = {0x30, 0xED, 0xA0, 0x56, 0xD2, 0xF8},
        .ip = {192, 168, 1, 99}
    }
};



/*******************************************************************************
* Code
******************************************************************************/
static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = 
    {
            .max_files = 4,
            .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGE("SYS", "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}


static void omni_io_init()
{
    gpio_config_t w5500_cs_conf = {};
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

    // gptimer_config_t timer_config_1 = 
    // {
    //     .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    //     .direction = GPTIMER_COUNT_UP,
    //     .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    // };

    // ESP_ERROR_CHECK(gptimer_new_timer(&timer_config_1, &s_gptimer_1));

}

static void omni_bdc_motor_init()
{
    bdc_motor_config_t bdc_motor_m0_config = 
    {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_M0_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_M0_MCPWM_GPIO_B,
    };
    bdc_motor_config_t bdc_motor_m1_config = 
    {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_M1_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_M1_MCPWM_GPIO_B,
    };
    bdc_motor_config_t bdc_motor_m2_config = 
    {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_M2_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_M2_MCPWM_GPIO_B,
    };

    bdc_motor_mcpwm_config_t bdc_motor_m0_mcpwm_config =
    {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ
    };
    bdc_motor_mcpwm_config_t bdc_motor_m1_mcpwm_config =
    {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ
    };
    bdc_motor_mcpwm_config_t bdc_motor_m2_mcpwm_config =
    {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ
    };


    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&bdc_motor_m0_config,
                                               &bdc_motor_m0_mcpwm_config,
                                               &s_bdc_motor_m0));

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&bdc_motor_m1_config,
                                               &bdc_motor_m1_mcpwm_config,
                                               &s_bdc_motor_m1));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&bdc_motor_m2_config,
                                               &bdc_motor_m2_mcpwm_config,
                                               &s_bdc_motor_m2));
    
    ESP_ERROR_CHECK(bdc_motor_enable(s_bdc_motor_m0));
    ESP_ERROR_CHECK(bdc_motor_enable(s_bdc_motor_m1));
    ESP_ERROR_CHECK(bdc_motor_enable(s_bdc_motor_m2));

    ESP_LOGI("bdc_motor", "Done");

}

static void omni_encoder_init()
{
    //encoder_m0
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

    pcnt_chan_config_t chan_a_config_m0 = 
    {
        .edge_gpio_num = BDC_M0_ENCODER_GPIO_A,
        .level_gpio_num = BDC_M0_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a_m0 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m0, &chan_a_config_m0, &pcnt_chan_a_m0));

    pcnt_chan_config_t chan_b_config_m0 = 
    {
        .edge_gpio_num = BDC_M0_ENCODER_GPIO_B,
        .level_gpio_num = BDC_M0_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b_m0 = NULL;
    // ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m0, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m0, &chan_b_config_m0, &pcnt_chan_b_m0));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_m0, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_m0, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_m0, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_m0, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    // ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_encoder_m0, BDC_ENCODER_PCNT_HIGH_LIMIT));
    // ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_encoder_m0, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(s_encoder_m0));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_encoder_m0));
    ESP_ERROR_CHECK(pcnt_unit_start(s_encoder_m0));


    //encoder_m1
    pcnt_unit_config_t encoder_m1_config = 
    {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&encoder_m1_config, &s_encoder_m1));
    
    pcnt_glitch_filter_config_t encoder_m1_filter_config = 
    {
        .max_glitch_ns = BDC_ENCODER_MAX_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_encoder_m1, &encoder_m1_filter_config));

    pcnt_chan_config_t chan_a_config_m1 = 
    {
        .edge_gpio_num = BDC_M1_ENCODER_GPIO_A,
        .level_gpio_num = BDC_M1_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a_m1 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m1, &chan_a_config_m1, &pcnt_chan_a_m1));

    pcnt_chan_config_t chan_b_config_m1 = 
    {
        .edge_gpio_num = BDC_M1_ENCODER_GPIO_B,
        .level_gpio_num = BDC_M1_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b_m1 = NULL;
    // ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m0, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m1, &chan_b_config_m1, &pcnt_chan_b_m1));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_m1, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_m1, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_m1, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_m1, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    // ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_encoder_m0, BDC_ENCODER_PCNT_HIGH_LIMIT));
    // ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_encoder_m0, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(s_encoder_m1));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_encoder_m1));
    ESP_ERROR_CHECK(pcnt_unit_start(s_encoder_m1));

    //encoder m2
    pcnt_unit_config_t encoder_m2_config = 
    {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&encoder_m2_config, &s_encoder_m2));
    
    pcnt_glitch_filter_config_t encoder_m2_filter_config = 
    {
        .max_glitch_ns = BDC_ENCODER_MAX_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_encoder_m2, &encoder_m2_filter_config));

    pcnt_chan_config_t chan_a_config_m2 = 
    {
        .edge_gpio_num = BDC_M2_ENCODER_GPIO_A,
        .level_gpio_num = BDC_M2_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a_m2 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m2, &chan_a_config_m2, &pcnt_chan_a_m2));

    pcnt_chan_config_t chan_b_config_m2 = 
    {
        .edge_gpio_num = BDC_M0_ENCODER_GPIO_B,
        .level_gpio_num = BDC_M0_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b_m2 = NULL;
    // ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m0, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_new_channel(s_encoder_m2, &chan_b_config_m2, &pcnt_chan_b_m2));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_m2, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_m2, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_m2, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_m2, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    // ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_encoder_m0, BDC_ENCODER_PCNT_HIGH_LIMIT));
    // ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_encoder_m0, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(s_encoder_m2));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_encoder_m2));
    ESP_ERROR_CHECK(pcnt_unit_start(s_encoder_m2));
}


static void omni_pid_init()
{
    //m0
    pid_ctrl_parameter_t pid_m0_runtime_param = 
    {
        .kp = 20.0,
        .ki = 3.0,
        .kd = 0,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = - (BDC_MCPWM_DUTY_TICK_MAX - 1),
        .max_integral = 1000,
        .min_integral = -1000,
    };
    
    pid_ctrl_config_t pid_m0_config = 
    {
        .init_param = pid_m0_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&pid_m0_config, &s_pid_m0));

    //m1
    pid_ctrl_parameter_t pid_m1_runtime_param = 
    {
        .kp = 25.0,
        .ki = 10,
        .kd = 0,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = - (BDC_MCPWM_DUTY_TICK_MAX - 1),
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_config_t pid_m1_config = 
    {
        .init_param = pid_m1_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_m1_config, &s_pid_m1));

    //m2
     pid_ctrl_parameter_t pid_m2_runtime_param = 
    {
        .kp = 25.0,
        .ki = 10,
        .kd = 0,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = - (BDC_MCPWM_DUTY_TICK_MAX - 1),
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_config_t pid_m2_config = 
    {
        .init_param = pid_m2_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_m2_config, &s_pid_m2));

}

static void omni_console_init()
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = 1024;

    initialize_nvs();

    initialize_filesystem();
    repl_config.history_save_path = HISTORY_PATH;
    ESP_LOGI("SYS", "Command history enabled");

    esp_console_register_help_command();
    // esp_console_reg();

    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}



static void omni_w5500_init()
{
    ESP_LOGI("W5500", "Initializing W5500 ...");
    gpio_config_t rst_gpio_config = 
    {
        .pin_bit_mask = (1ULL << W5500_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&rst_gpio_config);

    //force reset w5500
    gpio_set_level(W5500_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(W5500_RST_GPIO, 1);
    ESP_LOGI("W5500", "Reset W5500 ...");
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_config_t cs_gpio_config = {
        .pin_bit_mask = (1ULL << W5500_SPI_CS_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&cs_gpio_config);
    gpio_set_level(W5500_SPI_CS_GPIO, 0);

    spi_bus_config_t bus_cfg = 
    {
        .mosi_io_num=W5500_SPI_MOSI_GPIO,
        .miso_io_num=W5500_SPI_MISO_GPIO,
        .sclk_io_num=W5500_SPI_CLK_GPIO,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(W5500_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = 
    {
        .clock_speed_hz= W5500_SPI_CLK_FREQ, 
        .mode=0, 
        .spics_io_num=-1, 
        .queue_size=7
    };
    ESP_ERROR_CHECK(spi_bus_add_device(W5500_SPI_HOST, &dev_cfg, &s_spi_handle));
    //register callback for w5500
    reg_wizchip_cs_cbfunc(omni_w5500_cs_select, omni_w5500_cs_deselect);
    reg_wizchip_spi_cbfunc(omni_w5500_spi_readbyte, omni_w5500_spi_writebyte);
    wizchip_init(NULL, NULL);
    
    //Set interrupt mask W5500 - INT pin go low when interrupt issued
    setIMR(0xFF); // it is just work, dont touch it :)
    setSIMR(1); 
    ESP_LOGI("W5500", "W5500 global interrupt mask enabled.");

    //Verify IC version and SPI bus
    uint8_t version = getVERSIONR();
    if(version != 0x04)
    {
        ESP_LOGE("W5500", "SPI communication failed. Loop forever. Version %d", version);
        for(int k = 0; k <10; k++)
        {
            version = getSIMR();
            ESP_LOGE("W5500", "SPI communication faimkmkmkmled. Loop forever. Version %d", version);
            vTaskDelay(pdMS_TO_TICKS(20));

        }
        while(1);
    }
    else
    {
        ESP_LOGI("W5500", "W5500 Version: 0x%02X", version);
    }

    //Set W5500 server MAC, IP,..
    uint8_t esp32_s3_base_mac_addr[6];
    uint8_t init_mac_status = 0;
    ESP_ERROR_CHECK(esp_read_mac(esp32_s3_base_mac_addr, ESP_MAC_WIFI_STA));
    for(uint8_t module_index = 0; module_index < NUM_MODULES; module_index++)
    {
        if(memcmp(s_omni_module_eth_info[module_index].mac,
                   esp32_s3_base_mac_addr, 
                   sizeof(esp32_s3_base_mac_addr)) == 0)
        {
            //copy mac from predefine data
            memcpy(s_w5500_server_info.mac, 
                   esp32_s3_base_mac_addr, 
                   sizeof(esp32_s3_base_mac_addr));
            //copy ip from predefine data
            memcpy(s_w5500_server_info.ip, 
                   s_omni_module_eth_info[module_index].ip, 
                   sizeof(esp32_s3_base_mac_addr));
            wizchip_setnetinfo(&s_w5500_server_info);
            ESP_LOGI("W5500", "Network configured.");
            ESP_LOGI("W5500", "Base MAC Address (Wi-Fi STA): %02X:%02X:%02X:%02X:%02X:%02X",
                    s_w5500_server_info.mac[0],  s_w5500_server_info.mac[1],  s_w5500_server_info.mac[2],  
                    s_w5500_server_info.mac[3],  s_w5500_server_info.mac[4],  s_w5500_server_info.mac[5]);
            ESP_LOGI("W5500", "IP Address: %d.%d.%d.%d",
                    s_w5500_server_info.ip[0],  s_w5500_server_info.ip[1],  
                    s_w5500_server_info.ip[2],  s_w5500_server_info.ip[3]);
            init_mac_status = 1;
        }
    }

    // Use default MAC, IP if cant fine predefine MAC
    if(!init_mac_status)
    {
        ESP_LOGE("W5500", 
                 "Can't find predefined MAC Address. Use default MAC and IP");
        wizchip_setnetinfo(&s_w5500_server_info);
        ESP_LOGE("W5500", "Network configured.");
        ESP_LOGE("W5500", "Base MAC Address (Wi-Fi STA): %02X:%02X:%02X:%02X:%02X:%02X",
                    esp32_s3_base_mac_addr[0],  esp32_s3_base_mac_addr[1],  esp32_s3_base_mac_addr[2],  
                    esp32_s3_base_mac_addr[3],  esp32_s3_base_mac_addr[4],  esp32_s3_base_mac_addr[5]);
        ESP_LOGE("W5500", "IP Address: %d.%d.%d.%d",
                    s_w5500_server_info.ip[0],  s_w5500_server_info.ip[1],  
                    s_w5500_server_info.ip[2],  s_w5500_server_info.ip[3]);
    }

    // Check PHY
    ESP_LOGI("W5500", "Checking for Ethernet link...");
    ESP_LOGI("W5500", "Waiting till PHY ready ...\n");
    while (!(getPHYCFGR() & PHYCFGR_LNK_ON)) 
    {
    }
    ESP_LOGI("W5500", "Ethernet link is UP.");
    ESP_LOGI("W5500", "W5500 base setup done, set up connect in main()");
}

static uint8_t omni_w5500_spi_readbyte(void) 
{
    uint8_t rx_data = 0;
    spi_transaction_t t = {.length = 8, .tx_buffer = NULL, .rx_buffer = &rx_data};
    assert(spi_device_polling_transmit(s_spi_handle, &t) == ESP_OK);
    return rx_data;
}

static void omni_w5500_spi_writebyte(uint8_t wb) 
{
    spi_transaction_t t = {.length = 8, .tx_buffer = &wb, .rx_buffer = NULL};
    assert(spi_device_polling_transmit(s_spi_handle, &t) == ESP_OK);
}

static void filter_ema_init()
{
    ema_init(&s_ema_m0, FILTER_EMA_ALPHA, 0, 0);
    ema_init(&s_ema_m1, FILTER_EMA_ALPHA, 0, 0);
    ema_init(&s_ema_m2, FILTER_EMA_ALPHA, 0, 0);
    ESP_LOGI("EMA", "EMA Initialized done");

}

esp_err_t omni_bsp_init()
{
    omni_io_init();
    omni_timer_init();
    omni_bdc_motor_init();
    omni_encoder_init();
    omni_pid_init();
    // omni_console_init();
    omni_w5500_init();
    filter_ema_init();
    ESP_LOGI("DONE", "install pcnt channels");

    return ESP_OK;

}


gptimer_handle_t omni_get_timer(uint8_t _channel)
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
    if(_motor_index == OMNI_BDC_MOTOR_M0)
    {
        return s_bdc_motor_m0;
    }
    else if(_motor_index == OMNI_BDC_MOTOR_M1)
    {
        return s_bdc_motor_m1;
    }
    else if(_motor_index == OMNI_BDC_MOTOR_M2)
    {
        return s_bdc_motor_m2;
    }
    return NULL;
}

pcnt_unit_handle_t omni_get_encoder(uint8_t _motor_index)
{
    if(_motor_index == OMNI_BDC_MOTOR_M0)
    {
        return s_encoder_m0;
    }
    else if(_motor_index == OMNI_BDC_MOTOR_M1)
    {
        return s_encoder_m1;
    }
    else if(_motor_index == OMNI_BDC_MOTOR_M2)
    {
        return s_encoder_m2;
    }
    return NULL;
}

pid_ctrl_block_handle_t omni_get_pid(uint8_t _motor_index)
{
    if(_motor_index == OMNI_BDC_MOTOR_M0)
    {
        return s_pid_m0;
    }
    else if(_motor_index == OMNI_BDC_MOTOR_M1)
    {
        return s_pid_m1;
    }
    else if(_motor_index == OMNI_BDC_MOTOR_M2)
    {
        return s_pid_m2;
    }
    return NULL;
}

ema_t* omni_filter_ema_get_handle(uint8_t _motor_index)
{
    if(_motor_index == OMNI_BDC_MOTOR_M0)
    {
        return &s_ema_m0;
    }
    else if(_motor_index == OMNI_BDC_MOTOR_M1)
    {
        return &s_ema_m1;
    }
    else if(_motor_index == OMNI_BDC_MOTOR_M2)
    {
        return &s_ema_m2;
    }
    return NULL;
}

void tinysh_char_out(unsigned char c)
{
    uart_write_bytes(CONSOLE_UART_PORT_NUM, &c, 1U);
}


