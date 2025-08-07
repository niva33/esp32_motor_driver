#ifndef CONSOLE_H_
#define CONSOLE_H_


#ifdef __cplusplus
extern "C"{
#endif

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"



/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
esp_err_t console_init();


#ifdef __cplusplus
}
#endif

#endif /*CONSOLE_H_*/