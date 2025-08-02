idf_component_register(SRCS "omni_bsp.c"
                        INCLUDE_DIRS "include"
                        PRIV_REQUIRES "driver"
                        REQUIRES "bdc_motor" "pid_ctl")
