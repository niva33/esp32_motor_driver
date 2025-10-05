idf_component_register(SRCS "pid_tuning.c" 
                    INCLUDE_DIRS "."
                    REQUIRES "bdc_motor" "pid_ctl" "bsp" "app"
                    PRIV_REQUIRES "driver" "esp_timer"   "esp_adc"  )
