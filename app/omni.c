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
    _app->drv.m_encoder_m0 = omni_get_encoder(0);
    _app->drv.m_pid_ctrl_m0 = omni_get_pid(0);
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