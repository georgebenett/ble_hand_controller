#ifndef ELASTIC_H
#define ELASTIC_H

#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"

// Elastic slider configuration
#define ELASTIC_SPRING_CONSTANT 0.15f    // Spring constant (higher = stiffer spring)
#define ELASTIC_DAMPING_FACTOR 0.85f     // Damping factor (higher = less oscillation)
#define ELASTIC_UPDATE_PERIOD_MS 16      // Animation update period in ms (60fps)
#define ELASTIC_MIN_VELOCITY 0.5f        // Minimum velocity to continue animation

/**
 * @brief Initialize the elastic slider module
 */
void elastic_init(void);

/**
 * @brief Create an elastic slider
 *
 * @param parent Parent object
 * @param rest_value Value to return to (usually 0)
 * @param min Minimum slider value
 * @param max Maximum slider value
 * @return lv_obj_t* Slider object
 */
lv_obj_t* elastic_slider_create(lv_obj_t* parent, int32_t rest_value, int32_t min, int32_t max);

/**
 * @brief Set callback function for when slider value changes
 *
 * @param slider Elastic slider object
 * @param cb Callback function
 * @param user_data User data to pass to callback
 */
void elastic_slider_set_value_changed_cb(lv_obj_t* slider, void (*cb)(int32_t value, void* user_data), void* user_data);

/**
 * @brief Get current value of elastic slider
 *
 * @param slider Elastic slider object
 * @return int32_t Current value
 */
int32_t elastic_slider_get_value(lv_obj_t* slider);

/**
 * @brief Set value of elastic slider
 *
 * @param slider Elastic slider object
 * @param value New value
 * @param anim Whether to animate the change
 */
void elastic_slider_set_value(lv_obj_t* slider, int32_t value, bool anim);

/**
 * @brief Set spring constant for a specific slider
 *
 * @param slider Elastic slider object
 * @param spring_constant New spring constant
 */
void elastic_slider_set_spring_constant(lv_obj_t* slider, float spring_constant);

/**
 * @brief Set damping factor for a specific slider
 *
 * @param slider Elastic slider object
 * @param damping_factor New damping factor
 */
void elastic_slider_set_damping_factor(lv_obj_t* slider, float damping_factor);

#endif // ELASTIC_H