#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>



/*******************************************************************************
* Definitions
******************************************************************************/
typedef void hal_timer_t;

typedef void (*hal_timer_cb_fn_t)(void*);


/*******************************************************************************
* Prototypes
******************************************************************************/

hal_timer_t* hal_timer_initz(void* _instance, void* _config, void* _channel_config);

hal_timer_t* hal_timer_init(uint8_t _instance, void* _config, void* _user_config);

int32_t hal_timer_deinit(hal_timer_t* _this);

int32_t hal_timer_open(hal_timer_t* _this);

int32_t hal_timer_close(hal_timer_t* _this);

int32_t hal_timer_set_period(hal_timer_t* _this, uint32_t _period);

int32_t hal_timer_set_cb(hal_timer_t* _this, hal_timer_cb_fn_t _cb, void* _arg);

int32_t hal_timer_start(hal_timer_t* _this);

int32_t hal_timer_stop(hal_timer_t* _this);

#ifdef __cplusplus 
};
#endif

#endif // HAL_TIMER_H
