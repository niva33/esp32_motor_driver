#include "omni.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static esp_err_t omni_drv_create(omni_t* _app);


/*******************************************************************************
 * Variables
 ******************************************************************************/
static omni_t g_omni_app = 
{
    .drv = 
    {
        .m_bdc_motor_m0 = NULL,
        .m_bdc_motor_m1 = NULL,
        .m_bdc_motor_m2 = NULL,
        .m_encoder_m0 = NULL,
        .m_pid_ctrl_m0 = NULL,
    }
};

omni_t* g_omni_app_default = NULL;

/*******************************************************************************
 * Code
 ******************************************************************************/
static esp_err_t omni_drv_create(omni_t* _app)
{
    if(!_app)
    {
        return -1;
    }
    // create bdc_control
    _app->drv.m_bdc_motor_m0 = omni_get_bdc_motor(0);
    _app->drv.m_bdc_motor_m1 = omni_get_bdc_motor(1);
    _app->drv.m_bdc_motor_m2 = omni_get_bdc_motor(2);
    _app->drv.m_encoder_m0 = omni_get_encoder(0);
    _app->drv.m_encoder_m1 = omni_get_encoder(1);
    _app->drv.m_encoder_m2 = omni_get_encoder(2);
    _app->drv.m_pid_ctrl_m0 = omni_get_pid(0);
    _app->drv.m_pid_ctrl_m1 = omni_get_pid(1);
    _app->drv.m_pid_ctrl_m2 = omni_get_pid(2);
    _app->drv.m_ema_m0 = omni_filter_ema_get_handle(0);
    _app->drv.m_ema_m1 = omni_filter_ema_get_handle(1);
    _app->drv.m_ema_m2 = omni_filter_ema_get_handle(2);

    // for(int = 0; i < NUM_MODULES; i++)
    // {

    // }


    return 0;
}

esp_err_t omni_init()
{
    omni_t* app = &g_omni_app;
    if(omni_drv_create(app) < 0)
    {
        return -1;
    }

    g_omni_app_default = app;

    return 0;

}