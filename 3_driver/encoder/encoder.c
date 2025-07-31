


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_PCNT_HIGH_LIMIT 100
#define EXAMPLE_PCNT_LOW_LIMIT -100


/*******************************************************************************
 * Prototypes
 ******************************************************************************/





/*******************************************************************************
 * Variables
 ******************************************************************************/






/*******************************************************************************
 * Code
 ******************************************************************************/
encoder_t* encoder_init(int32_t _channel_a,
                        int32_t _channel_b,
                        uint32_t _glitch_ns)
{
    encoder_t* encoder = (encoder*)malloc(sizeof(encoder_t));

    pcnt_unit_config_t unit_config = 
    {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };

    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_new_unit(&unit_config, &pcnt_unit);

    pcnt_glitch_filter_config_t filter_config = 
    {
        .max_glitch_ns = _glitch_ns,
    };

    pcnt_chan_config_t chan_a_config = 
    {
        .edge_gpio_num = _channel_a,
        .level_gpio_num = _channel_b,
    };

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);

    pcnt_chan_config_t chan_b_config = 
    {
        .edge_gpio_num = _channel_b,
        .level_gpio_num = _channel_a,
    };

    pcnt_channel_handle_t pcnt_chan_b = NULL;
    pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b);

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    encoder->m_unit_config = unit_config;
    encoder->m_pcnt_unit = pcnt_unit;
    encoder->m_filter_config = filter_config;
    encoder->m_chan_a_config = chan_a_config;
    encoder->m_pcnt_chan_a = pcnt_chan_a;
    encoder->m_chan_b_config = chan_b_config;
    encoder->m_pcnt_chan_b = pcnt_chan_b;

    return encoder;
}

int32_t encoder_open(encoder_t* _this)
{
    if(!_this)
    {
        return -1;
    }
    ESP_ERROR_CHECK(pcnt_unit_enable(_this->m_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(_this->m_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(_this->m_pcnt_unit));
    return 0;
}

int32_t encoder_get_count(encoder_t* _this, int32_t* _pulse_count)
{
    if(!_this)
    {
        return -1;
    }
    
    pcnt_unit_get_count(_this->m_pcnt_unit, _pulse_count);

    return 0;
}
