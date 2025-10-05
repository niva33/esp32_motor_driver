idf_component_register(
    SRCS "socket_.c" "w5500.c"
         "wizchip_conf.c"
        
    INCLUDE_DIRS "."
    PRIV_REQUIRES "driver"
)