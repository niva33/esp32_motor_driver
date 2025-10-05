#ifndef OMNI_BSP_H_
#define OMNI_BSP_H_

#ifdef __cplusplus
extern "C"{
#endif


#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "esp_vfs_dev.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <unistd.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "driver/uart_vfs.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include "esp_task_wdt.h"
#include "socket_.h"


#include "pid_ctrl.h"
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

#define CONSOLE_UART_PORT_NUM           CONFIG_ESP_CONSOLE_UART_NUM

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

#define BDC_ENCODER_PCNT_HIGH_LIMIT     30000
#define BDC_ENCODER_PCNT_LOW_LIMIT      -30000
#define BDC_ENCODER_MAX_GLITCH_NS       1000


//BDC_M0
#define BDC_M0_MCPWM_GPIO_A             GPIO_NUM_4
#define BDC_M0_MCPWM_GPIO_B             GPIO_NUM_5
#define BDC_M0_ENCODER_GPIO_A           GPIO_NUM_7
#define BDC_M0_ENCODER_GPIO_B           GPIO_NUM_6
#define BDC_M0_CUR_SENSE                GPIO_NUM_15
//BDC_M1
#define BDC_M1_MCPWM_GPIO_A             GPIO_NUM_17
#define BDC_M1_MCPWM_GPIO_B             GPIO_NUM_16
#define BDC_M1_ENCODER_GPIO_A           GPIO_NUM_18
#define BDC_M1_ENCODER_GPIO_B           GPIO_NUM_8
#define BDC_M1_CUR_SENSE                GPIO_NUM_9
//BDC_M2
#define BDC_M2_MCPWM_GPIO_A             GPIO_NUM_38
#define BDC_M2_MCPWM_GPIO_B             GPIO_NUM_2
#define BDC_M2_ENCODER_GPIO_A           GPIO_NUM_47
#define BDC_M2_ENCODER_GPIO_B           GPIO_NUM_48
#define BDC_M2_CUR_SENSE                GPIO_NUM_1

//W5500
#define W5500_SPI_CS_GPIO               GPIO_NUM_10
#define W5500_SPI_MOSI_GPIO             GPIO_NUM_11
#define W5500_SPI_MISO_GPIO             GPIO_NUM_13
#define W5500_SPI_CLK_GPIO              GPIO_NUM_12
#define W5500_INT_GPIO                  GPIO_NUM_14
#define W5500_RST_GPIO                  GPIO_NUM_21
#define W5500_SPI_HOST                  SPI2_HOST
#define W5500_TCP_SOCKET_NUM            (0u)
#define W5500_TCP_PORT                  (5000u)
#define W5500_SPI_CLK_FREQ              (50*1000*1000)



#define CONSOLE_UART_PORT_NUM           CONFIG_ESP_CONSOLE_UART_NUM
#define CONSOLE_TX_BUFFER_SIZE          0
#define CONSOLE_RX_BUFFER_SIZE          1024


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
esp_err_t omni_bsp_init();

gptimer_handle_t omni_get_timer(uint8_t _channel);

bdc_motor_handle_t omni_get_bdc_motor(uint8_t _motor_index);

pcnt_unit_handle_t omni_get_encoder(uint8_t _motor_index);

pid_ctrl_block_handle_t omni_get_pid(uint8_t _motor_index);







#ifdef __cplusplus
};
#endif

#endif /*OMNI_BSP_H_*/


