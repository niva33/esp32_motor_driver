#include "omni.h"
#include "omni_bsp.h"
#include "esp_timer.h"
#include "esp_sleep.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BDC_PID_EXPECT_SPEED        (0.0f)
#define USING_MA_FILTER

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
char g_data_buf[1024];

int32_t g_setpoint_vel[NUM_MOTORS];
uint8_t g_dir[NUM_MOTORS];

float g_check_real_pulse_m0 = 0;
float g_check_real_pulse_m1 = 0;
float g_check_real_pulse_m2 = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void omni_main_proc(void* _arg)
{
    (void)_arg;
    static int32_t last_pulse_count[NUM_MOTORS];
    uint8_t motor_idx;
    int cur_pulse_count[NUM_MOTORS];
    float real_pulse[NUM_MOTORS];
    float real_pulse_filter[NUM_MOTORS];
    float new_speed[NUM_MOTORS];
    float error[NUM_MOTORS];

    for (motor_idx = 0; motor_idx < NUM_MOTORS; motor_idx++)
    {
        //calculate real pulse
        ESP_ERROR_CHECK(pcnt_unit_get_count(g_omni_app_default->drv.m_encoder[motor_idx], &(cur_pulse_count[motor_idx])));
        real_pulse[motor_idx] = cur_pulse_count[motor_idx] - last_pulse_count[motor_idx];
        last_pulse_count[motor_idx] = cur_pulse_count[motor_idx];
        if(real_pulse[motor_idx] <  BDC_ENCODER_PCNT_LOW_LIMIT/4)
        {
            real_pulse[motor_idx] = real_pulse[motor_idx] + BDC_ENCODER_PCNT_HIGH_LIMIT;
        }
        if(real_pulse[motor_idx] > BDC_ENCODER_PCNT_HIGH_LIMIT/4)
        {
            real_pulse[motor_idx] = real_pulse[motor_idx] -  BDC_ENCODER_PCNT_HIGH_LIMIT;
        }

        //add filter
        ema_update(g_omni_app_default->drv.m_ema[motor_idx], real_pulse[motor_idx], &real_pulse_filter[motor_idx]);
        real_pulse[motor_idx] = real_pulse_filter[motor_idx];

        //pid control
        error[motor_idx] = g_setpoint_vel[motor_idx] - real_pulse[motor_idx];
        pid_compute(g_omni_app_default->drv.m_pid_ctrl[motor_idx], error[motor_idx], &new_speed[motor_idx]);
        if(new_speed[motor_idx] > 0 && g_dir[motor_idx] == 0)
        {
            new_speed[motor_idx] = 0;
        }
        else if(new_speed[motor_idx] < 0 && g_dir[motor_idx] == 1)
        {
            new_speed[motor_idx] = 0;
        }
        else if (new_speed[motor_idx] < 0 && g_dir[motor_idx] == 0)
        {
            new_speed[motor_idx] = - new_speed[motor_idx];
        }

        //control plant
        if(new_speed[motor_idx] == 0)
        {
            bdc_motor_brake(g_omni_app_default->drv.m_bdc_motor[motor_idx]);
        }
        else
        {
            bdc_motor_set_speed(g_omni_app_default->drv.m_bdc_motor[motor_idx], new_speed[motor_idx]);
            if(g_dir[motor_idx] == 1)
            {
                bdc_motor_forward(g_omni_app_default->drv.m_bdc_motor[motor_idx]);
            }
            else
            {
                bdc_motor_reverse(g_omni_app_default->drv.m_bdc_motor[motor_idx]);
            }
        }

       
    }
    g_check_real_pulse_m0 = real_pulse[0];
    g_check_real_pulse_m1 = real_pulse[1];
    g_check_real_pulse_m2 = real_pulse[2];

}                 

void omni_entry()
{
    omni_bsp_init();

    ESP_LOGI("siuuu", "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = omni_main_proc,
        .arg = NULL,
        .name = "pid_loop"
    };

    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));
    omni_init();
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, 10 * 1000));
    while(1)
    {
        uint8_t sn_sr = getSn_SR(W5500_TCP_SOCKET_NUM);
        if(sn_sr == SOCK_CLOSED)
        {
            socket(W5500_TCP_SOCKET_NUM, Sn_MR_TCP, W5500_TCP_PORT, 0x00);
        }
        else if (sn_sr == SOCK_INIT)
        {
            listen(W5500_TCP_SOCKET_NUM);
            uint8_t initial_sock_ir = getSn_IR(W5500_TCP_SOCKET_NUM);
            setSn_IR(W5500_TCP_SOCKET_NUM, initial_sock_ir); // Clear all interrupt
            ESP_LOGW("W5500", "Giá trị Sn_IR ban đầu (sau khi listen): 0x%02X", getSn_IR(W5500_TCP_SOCKET_NUM));
            
            // Config interrupt just when recv setpoint
            uint8_t socket_irq_mask = Sn_IR_RECV ;
            setSn_IMR(W5500_TCP_SOCKET_NUM, socket_irq_mask);
            setSIMR(1);
        }
        else if(sn_sr == SOCK_ESTABLISHED)
        {
            int int_status = gpio_get_level(W5500_INT_GPIO);
            if(int_status == 0)
            {
                setSn_IR(W5500_TCP_SOCKET_NUM, 0); // clear all flag
                uint16_t recv_size = getSn_RX_RSR(W5500_TCP_SOCKET_NUM);
                if (recv_size > 0) 
                {
                    int32_t ret = recv(W5500_TCP_SOCKET_NUM, (uint8_t*)g_data_buf, recv_size);
                    if(ret  > 0)
                    {
                        for(uint8_t motor_idx = 0; motor_idx < NUM_MOTORS; motor_idx++)
                        {
                            g_dir[motor_idx] = g_data_buf[0 + 4* motor_idx] - '0';
                            g_setpoint_vel[motor_idx] = (g_data_buf[1 + 4* motor_idx] - '0') * 10 + 
                                                        (g_data_buf[2 + 4* motor_idx] - '0');
                            if(g_dir[motor_idx] == 0)
                            {
                                g_setpoint_vel[motor_idx] = - g_setpoint_vel[motor_idx];
                            }
                        }
                        ESP_LOGI("TAG", "Received % d bytes: '%s'", ret, (char*)g_data_buf);
                    }
                }
            } 
            char response_msg[100];
            sprintf(response_msg, "$%.2f, %ld, %.2f, %ld, %0.2f, %ld \n",
                                  g_check_real_pulse_m0, 
                                  g_setpoint_vel[0], 
                                  g_check_real_pulse_m1,
                                  g_setpoint_vel[1],
                                  g_check_real_pulse_m2,
                                  g_setpoint_vel[2]);

            send(W5500_TCP_SOCKET_NUM, (uint8_t*)response_msg, strlen(response_msg));
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }

        else if(sn_sr == SOCK_CLOSE_WAIT)
        {
            setSn_CR(W5500_TCP_SOCKET_NUM, Sn_CR_DISCON);
            for(uint8_t motor_idx = 0; motor_idx < NUM_MOTORS; motor_idx++)
            {
                g_dir[motor_idx] = 0;
                g_setpoint_vel[motor_idx] = 0;
            }
        }    
    }

}

