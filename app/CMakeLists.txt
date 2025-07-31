idf_component_register(SRCS "omni.c"
                    INCLUDE_DIRS "."
                    REQUIRES "bdc_motor" "pid_ctl" "bsp"
                    PRIV_REQUIRES "driver")
