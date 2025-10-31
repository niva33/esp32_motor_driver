#include "ema.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/******************************************************************************
* Prototypes
******************************************************************************/
static int32_t ema_alpha_valid(ema_sample_t _a);

/******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 * ****************************************************************************/
 
static int32_t ema_alpha_valid(ema_sample_t _a)
{
    return (_a > 0.0f) && (_a <= 1.0f);
}
 
int32_t ema_init(ema_t* _this, ema_sample_t _alpha, 
                 ema_sample_t _seed, uint8_t _seed_now)
{
    if(!_this || !ema_alpha_valid(_alpha))
    {
        return -1;
        
    }
    _this->alpha = _alpha;
    if(_seed_now)
    {
        _this->cur_output = _seed;
        _this->initialized = 1u;
    }
    else
    {
        _this->cur_output = 0.0f;
        _this->initialized = 0u;
    }
    
    return 0;
}

int32_t ema_update(ema_t* _this, ema_sample_t _value, ema_sample_t* _out)
{
    if(!_this || !_out)
    {
        return -1;
    }
    if(!(_this->initialized))
    {
        _this->cur_output = _value;
        _this->initialized = 1u;
        *_out = _this->cur_output;
        return 0;
    }
    _this->cur_output = (_this->alpha) * _value + (1.0f - _this->alpha)*(_this->cur_output);
    *_out = _this->cur_output;
    return 0;

}

int32_t ema_reset(ema_t* _this, ema_sample_t _seed, ema_sample_t _seednow)
{
    if(!_this)
    {
        return -1;
    }
    if(_seednow)
    {
        _this->cur_output = _seed;
        _this->initialized = 1u;
    }
    else
    {
        _this->cur_output = 0.0f;
        _this->initialized = 0u;
    }
    
    return 0;
}

int32_t ema_get(ema_t* _this, ema_sample_t *_out)
{
    if(!_this || !_out)
    {
        return -1;
    }
    *_out = _this->cur_output;
    
    return 0;
}

int32_t ema_set_alpha(ema_t* _this, ema_sample_t _new_alpha)
{
    if(!_this || ema_alpha_valid(_new_alpha))
    {
        return -1;
    }
    _this->alpha = _new_alpha;
    
    return 0;
}

ema_sample_t ema_get_alpha_from_window(ema_sample_t _n)
{
    if(_n <= 0.0f)
    {
        return 0.0f;
    }
    
    return (ema_sample_t)(2.0f / (_n + 1.0f));
}



