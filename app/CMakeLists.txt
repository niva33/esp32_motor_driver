idf_component_register(SRCS "omni.c" "omni_entry.c" 
                    INCLUDE_DIRS "."
                    REQUIRES "bdc_motor" "pid_ctl" "bsp" 
                    PRIV_REQUIRES "driver" "esp_timer"  )
