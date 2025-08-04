#ifndef CONSOLE_H_
#define CONSOLE_H_


#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include "esp_log.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
esp_err_t console_init();

esp_err_t console_printf();

esp_err_t console_putchar(char c);

esp_err_t console_scanf();

esp_err_t console_getchar();



#ifdef __cplusplus
}
#endif

#endif /*CONSOLE_H_*/