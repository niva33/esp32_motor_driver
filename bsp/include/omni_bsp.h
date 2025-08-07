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


