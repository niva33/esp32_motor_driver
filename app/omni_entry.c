#include "omni.h"
#include "omni_bsp.h"
#include "esp_timer.h"
#include "esp_sleep.h"



/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BDC_PID_EXPECT_SPEED (30u)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
char g_data_buf[1024];
int32_t g_sp_0 = 0;
int32_t g_sp_1 = 0;
int32_t g_sp_2 = 0;
uint8_t g_dir_0 = 0;
uint8_t g_dir_1 = 0;
uint8_t g_dir_2 = 0;



/*******************************************************************************
 * Code
 ******************************************************************************/

static void omni_main_proc(void* _arg)
{
    (void)_arg;

    static int32_t last_pulse_count_m0 = 0;
    static int32_t last_pulse_count_m1 = 0;
    static int32_t last_pulse_count_m2 = 0;
    int cur_pulse_count_0 = 0;
    int cur_pulse_count_1 = 0;
    int cur_pulse_count_2 = 0;
    int32_t real_pulse_m0 = 0;
    int32_t real_pulse_m1 = 0;
    int32_t real_pulse_m2 = 0;


    omni_t* temp = g_omni_app_default;
    ESP_ERROR_CHECK(pcnt_unit_get_count(g_omni_app_default->drv.m_encoder_m0, &cur_pulse_count_0));
    ESP_ERROR_CHECK(pcnt_unit_get_count(g_omni_app_default->drv.m_encoder_m1, &cur_pulse_count_1));
    ESP_ERROR_CHECK(pcnt_unit_get_count(g_omni_app_default->drv.m_encoder_m2, &cur_pulse_count_2));

    //m0
    real_pulse_m0 = cur_pulse_count_0 - last_pulse_count_m0;
    last_pulse_count_m0 = cur_pulse_count_0;
    float error_m0 = (float)g_sp_0 - real_pulse_m0;
    float new_speed_m0 = 0;
    pid_compute(g_omni_app_default->drv.m_pid_ctrl_m0, error_m0, &new_speed_m0);
    if(g_dir_0 == 1 && new_speed_m0 <=0)
    {
        new_speed_m0 = 0;
    }
    else if(g_dir_0 == 0 && new_speed_m0 >= 0)
    {
        new_speed_m0 = 0;
    }

    if(new_speed_m0 < 0)
    {
        new_speed_m0 = - new_speed_m0;
    }
    bdc_motor_set_speed(g_omni_app_default->drv.m_bdc_motor_m0, new_speed_m0);

    //m1
    real_pulse_m1 = cur_pulse_count_1 - last_pulse_count_m1;
    last_pulse_count_m1 = cur_pulse_count_1;
    float error_m1 = (float)g_sp_1 - real_pulse_m1;
    float new_speed_m1 = 0;
    pid_compute(g_omni_app_default->drv.m_pid_ctrl_m1, error_m1, &new_speed_m1);
    if(g_dir_1 == 1 && new_speed_m1 <=0)
    {
        new_speed_m1 = 0;
    }
    else if(g_dir_1 == 0 && new_speed_m1 >= 0)
    {
        new_speed_m1 = 0;
    }

    if(new_speed_m1 < 0)
    {
        new_speed_m1 = - new_speed_m1;
    }
    bdc_motor_set_speed(g_omni_app_default->drv.m_bdc_motor_m1, new_speed_m1);

    //m2
    real_pulse_m2 = cur_pulse_count_2 - last_pulse_count_m2;
    last_pulse_count_m2 = cur_pulse_count_2;
    float error_m2 = (float)g_sp_2 - real_pulse_m2;
    float new_speed_m2 = 0;
    pid_compute(g_omni_app_default->drv.m_pid_ctrl_m2, error_m2, &new_speed_m2);
    if(g_dir_2 == 1 && new_speed_m2 <=0)
    {
        new_speed_m2 = 0;
    }
    else if(g_dir_2 == 0 && new_speed_m2 >= 0)
    {
        new_speed_m2 = 0;
    }

    if(new_speed_m2 < 0)
    {
        new_speed_m2 = - new_speed_m2;
    }
    bdc_motor_set_speed(g_omni_app_default->drv.m_bdc_motor_m2, new_speed_m2);

    if(g_dir_0 == 1)
    {
        bdc_motor_forward(g_omni_app_default->drv.m_bdc_motor_m0);
    }
    else
    {
        bdc_motor_reverse(g_omni_app_default->drv.m_bdc_motor_m0);
    }

    if(g_dir_1 == 1)
    {
        bdc_motor_forward(g_omni_app_default->drv.m_bdc_motor_m1);
    }
    else
    {
        bdc_motor_reverse(g_omni_app_default->drv.m_bdc_motor_m1);
    }

    if(g_dir_2 == 1)
    {
        bdc_motor_forward(g_omni_app_default->drv.m_bdc_motor_m2);
    }
    else
    {
        bdc_motor_reverse(g_omni_app_default->drv.m_bdc_motor_m2);
    }

    
    // bdc_motor_forward(g_omni_app_default->drv.m_bdc_motor_m1);
    // bdc_motor_forward(g_omni_app_default->drv.m_bdc_motor_m2);

    
    ESP_LOGI("PID", "real_pulse_m0 %d, g_sp_0 %d, real_pulse_m1 %d, g_sp_1 %d, real_pulse_m2 %d, g_sp_2 %d",
                    real_pulse_m0, g_sp_0, real_pulse_m1, g_sp_1, real_pulse_m2, g_sp_2) ;

    // ESP_LOGI("PID", "real_pulse_m0 %d, real_pulse_m1 %d, real_pulse_m2 %d, new_speed_m0 %f, new_speed_m1 %f, new_speed_m2 %f",
    //                 real_pulse_m0, real_pulse_m1, real_pulse_m2, new_speed_m0, new_speed_m1, new_speed_m2) ;

        // ESP_LOGI("PID", "real_pulse_m0 %d, g_sp_0 %d, new_speed_m0 %f,g_dir_0 %d",
        //              real_pulse_m0, g_sp_0, new_speed_m0, g_dir_0);
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
    // bdc_motor_enable(g_omni_app_default->drv.m_bdc_motor_m0);
    // bdc_motor_set_speed(g_omni_app_default->drv.m_bdc_motor_m0, 300u);
    // bdc_motor_forward(g_omni_app_default->drv.m_bdc_motor_m0);



    // gpio_set_level(GPIO_NUM_26, 1);

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
            setSn_IR(W5500_TCP_SOCKET_NUM, initial_sock_ir); // Xóa mọi cờ cũ để bắt đầu sạch
            ESP_LOGW("W5500", "Giá trị Sn_IR ban đầu (sau khi listen): 0x%02X", getSn_IR(W5500_TCP_SOCKET_NUM));
            
            // Cấu hình ngắt cho các sự kiện mong muốn
            uint8_t socket_irq_mask = Sn_IR_RECV ;
            setSn_IMR(W5500_TCP_SOCKET_NUM, socket_irq_mask);
            setSIMR(1);
        }
        else if(sn_sr == SOCK_ESTABLISHED)
        {
            uint8_t sn_cr = getSn_CR(W5500_TCP_SOCKET_NUM);
            uint8_t sn_ir = getSn_IR(W5500_TCP_SOCKET_NUM);
            uint8_t sn_imr = getSn_IMR(W5500_TCP_SOCKET_NUM);
            uint16_t intlevel = getINTLEVEL();

            int int_status = gpio_get_level(W5500_INT_GPIO);
            if(int_status == 0)
            {
                setSn_IR(W5500_TCP_SOCKET_NUM, 0); // Xóa mọi cờ cũ để bắt đầu sạch
                uint16_t recv_size = getSn_RX_RSR(W5500_TCP_SOCKET_NUM);
                if (recv_size > 0) 
                {
                    int32_t ret = recv(W5500_TCP_SOCKET_NUM, (uint8_t*)g_data_buf, recv_size);
                    if(ret  > 0)
                    {
                        g_dir_0 = g_data_buf[0] - '0';
                        g_dir_1 = g_data_buf[1] - '0';
                        g_dir_2 = g_data_buf[2] - '0';
                        g_sp_0 = (g_data_buf[3] - '0') * 10 + (g_data_buf[4] - '0');
                        g_sp_1 = (g_data_buf[5] - '0') * 10 + (g_data_buf[6] - '0');
                        g_sp_2 = (g_data_buf[7] - '0') * 10 + (g_data_buf[8] - '0');
                        if(g_dir_0 == 0)
                        {
                            g_sp_0 = - g_sp_0;
                        }
                        if(g_dir_1 == 0)
                        {
                            g_sp_1 = - g_sp_1;
                        }
                        if(g_dir_2 == 0)
                        {
                            g_sp_2 = - g_sp_2;
                        }
                        ESP_LOGI("TAG", "Received %d bytes: '%s'", ret, (char*)g_data_buf);
                        char* response_msg = "\nRecv setpoint";
                        send(W5500_TCP_SOCKET_NUM, (uint8_t*)response_msg, strlen(response_msg));
                    }
                }
            }   
        }
        else if(sn_sr == SOCK_CLOSE_WAIT)
        {
            setSn_CR(W5500_TCP_SOCKET_NUM, Sn_CR_DISCON);
            g_sp_0 = 0;
            g_sp_1 = 0;
            g_sp_2 = 0;
            // check = 0;
        }    

    }

}

