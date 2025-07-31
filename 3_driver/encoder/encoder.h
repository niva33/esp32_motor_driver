

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct 
{
    pcnt_unit_config_t m_unit_config;
    pcnt_unit_handle_t m_pcnt_unit;
    pcnt_glitch_filter_config_t m_filter_config;
    pcnt_chan_config_t m_chan_a_config;
    pcnt_channel_handle_t m_pcnt_chan_a;
    pcnt_chan_config_t m_chan_b_config;
    pcnt_channel_handle_t m_pcnt_chan_b;

} encoder_t;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
encoder_t* encoder_init(int32_t _channel_a, int32_t _channel_b,
                        uint32_t _glitch_ns);


int32_t encoder_open(encoder_t* _this);

int32_t encoder_get_count(encoder_t* _this, int32_t* _pulse_count);


