/**
 * @file    exponential_moving_average.h
 * @brief   Exponential Moving Average (EMA) filter interface.
 * @details This header provides the API for an Exponential Moving Average filter.
 *          The implementation is designed for embedded systems where minimal
 *          computational cost and deterministic behavior are required.
 *
 * @note    All functions in this module are reentrant as long as each instance
 *          of ema_t is accessed by a single context at a time.
 * @note    Floating-point arithmetic is used; ensure that the target platform
 *          supports it or replace with fixed-point equivalents.
 *
 * @version 1.0
 * @date    2025-11-01
 * @author  dungtn
 */

#ifndef EXPONENTIAL_MOVING_AVERAGE_H
#define EXPONENTIAL_MOVING_AVERAGE_H

/*==============================================================================
 * Includes
 *============================================================================*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @typedef ema_sample_t
 * @brief   Type used for EMA computation.
 *
 * @details The sample type can be modified (e.g., to int16_t or int32_t)
 *          if a fixed-point implementation is required.
 */
typedef float ema_sample_t;

/**
 * @struct ema_t
 * @brief  Structure representing an EMA filter instance.
 *
 * @var ema_t::alpha
 *      Smoothing coefficient (0 < alpha <= 1.0F).
 * @var ema_t::cur_output
 *      Most recent EMA output sample.
 * @var ema_t::initialized
 *      Initialization flag. Set to 1U when the filter has been seeded.
 */
typedef struct
{
    ema_sample_t alpha;
    ema_sample_t cur_output;
    uint8_t initialized;
} ema_t;

/*==============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief   Initialize the EMA filter instance.
 *
 * @param[in,out] _this       Pointer to the EMA instance to initialize.
 * @param[in]     _alpha      Smoothing coefficient (0 < _alpha <= 1.0F).
 * @param[in]     _seed       Initial output value (used when _seed_now != 0U).
 * @param[in]     _seed_now   Non-zero value seeds the filter immediately with _seed.
 *
 * @return 0 if successful; negative value if input parameters are invalid.
 *
 * @note    This function must be called before any call to ema_update().
 */
int32_t ema_init(ema_t* _this,
                 ema_sample_t _alpha,
                 ema_sample_t _seed,
                 uint8_t _seed_now);

/**
 * @brief   Update the EMA filter with a new input sample.
 *
 * @param[in,out] _this   Pointer to EMA instance.
 * @param[in]     _value  New input sample.
 * @param[out]    _out    Pointer to variable to store the filtered output.
 *
 * @return 0 if successful; negative value if input pointers are invalid.
 *
 * @note    On the first call (if not initialized), the filter output will be
 *          seeded using the first sample to prevent transient spikes.
 */
int32_t ema_update(ema_t* _this,
                   ema_sample_t _value,
                   ema_sample_t* _out);

/**
 * @brief   Reset the EMA filter state.
 *
 * @param[in,out] _this      Pointer to EMA instance.
 * @param[in]     _seed      New seed value.
 * @param[in]     _seednow   Non-zero value seeds output immediately with _seed.
 *
 * @return 0 if successful; negative value if input pointer is invalid.
 *
 * @note    The alpha coefficient remains unchanged.
 */
int32_t ema_reset(ema_t* _this,
                  ema_sample_t _seed,
                  ema_sample_t _seednow);

/**
 * @brief   Retrieve the current EMA output value.
 *
 * @param[in]  _this   Pointer to EMA instance.
 * @param[out] _out    Pointer to variable to store the current EMA output.
 *
 * @return 0 if successful; negative value if instance is uninitialized or null.
 */
int32_t ema_get(ema_t* _this,
                ema_sample_t *_out);

/**
 * @brief   Change the alpha coefficient at runtime.
 *
 * @param[in,out] _this        Pointer to EMA instance.
 * @param[in]     _new_alpha   New alpha coefficient (0 < _new_alpha <= 1.0F).
 *
 * @return 0 if successful; negative value if parameters are invalid.
 *
 * @note    This function does not modify the current output value.
 */
int32_t ema_set_alpha(ema_t* _this,
                      ema_sample_t _new_alpha);

/**
 * @brief   Compute alpha coefficient from an equivalent moving window size.
 *
 * @param[in] _n  Equivalent number of samples (window length).
 *
 * @return Alpha coefficient corresponding to the given window length.
 *         Returns 0.0F if input is invalid (e.g., _n <= 0.0F).
 *
 * @note    The conversion is based on the empirical relation:
 *          alpha = 2 / (N + 1)
 */
ema_sample_t ema_get_alpha_from_window(ema_sample_t _n);

#endif /* EXPONENTIAL_MOVING_AVERAGE_H */
