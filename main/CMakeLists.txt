idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES "app" "pid_tuning"
                       PRIV_REQUIRES "driver" "esp_timer") 
