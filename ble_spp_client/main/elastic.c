#include "elastic.h"
#include "esp_log.h"
#include <math.h>

#define TAG "ELASTIC"

typedef struct {
    lv_obj_t* slider;                // LVGL slider object
    int32_t rest_value;              // Value to return to (usually 0)
    float velocity;                  // Current velocity
    float spring_constant;           // Spring constant (higher = stiffer spring)
    float damping_factor;            // Damping factor (higher = less oscillation)
    bool animation_active;           // Whether animation is currently running
    lv_timer_t* timer;               // Timer for animation
    void (*value_changed_cb)(int32_t value, void* user_data);  // Value changed callback
    void* user_data;                 // User data for callback
} elastic_slider_t;

// Forward declarations
static void elastic_slider_event_cb(lv_event_t* e);
static void elastic_animation_timer_cb(lv_timer_t* timer);

// Global map to store slider -> data mapping
#define MAX_ELASTIC_SLIDERS 10
static struct {
    lv_obj_t* slider;
    elastic_slider_t* data;
} elastic_sliders[MAX_ELASTIC_SLIDERS] = {0};

static elastic_slider_t* find_slider_data(lv_obj_t* slider) {
    for (int i = 0; i < MAX_ELASTIC_SLIDERS; i++) {
        if (elastic_sliders[i].slider == slider) {
            return elastic_sliders[i].data;
        }
    }
    return NULL;
}

static void add_slider_data(lv_obj_t* slider, elastic_slider_t* data) {
    for (int i = 0; i < MAX_ELASTIC_SLIDERS; i++) {
        if (elastic_sliders[i].slider == NULL) {
            elastic_sliders[i].slider = slider;
            elastic_sliders[i].data = data;
            return;
        }
    }
    ESP_LOGE(TAG, "Too many elastic sliders, increase MAX_ELASTIC_SLIDERS");
}

static void remove_slider_data(lv_obj_t* slider) {
    for (int i = 0; i < MAX_ELASTIC_SLIDERS; i++) {
        if (elastic_sliders[i].slider == slider) {
            elastic_sliders[i].slider = NULL;
            elastic_sliders[i].data = NULL;
            return;
        }
    }
}

void elastic_init(void) {
    // Initialize the slider data array
    for (int i = 0; i < MAX_ELASTIC_SLIDERS; i++) {
        elastic_sliders[i].slider = NULL;
        elastic_sliders[i].data = NULL;
    }
}

lv_obj_t* elastic_slider_create(lv_obj_t* parent, int32_t rest_value, int32_t min, int32_t max) {
    // Create slider
    lv_obj_t* slider = lv_slider_create(parent);
    lv_slider_set_range(slider, min, max);
    lv_slider_set_value(slider, rest_value, LV_ANIM_OFF);

    // Create elastic slider data
    elastic_slider_t* data = (elastic_slider_t*)lv_mem_alloc(sizeof(elastic_slider_t));
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for elastic slider data");
        return NULL;
    }

    // Initialize data
    data->slider = slider;
    data->rest_value = rest_value;
    data->velocity = 0.0f;
    data->spring_constant = ELASTIC_SPRING_CONSTANT;
    data->damping_factor = ELASTIC_DAMPING_FACTOR;
    data->animation_active = false;
    data->timer = NULL;
    data->value_changed_cb = NULL;
    data->user_data = NULL;

    // Store in our global map
    add_slider_data(slider, data);

    // Add event handler
    lv_obj_add_event_cb(slider, elastic_slider_event_cb, LV_EVENT_ALL, NULL);

    return slider;
}

void elastic_slider_set_value_changed_cb(lv_obj_t* slider, void (*cb)(int32_t value, void* user_data), void* user_data) {
    elastic_slider_t* data = find_slider_data(slider);
    if (data) {
        data->value_changed_cb = cb;
        data->user_data = user_data;
    }
}

int32_t elastic_slider_get_value(lv_obj_t* slider) {
    return lv_slider_get_value(slider);
}

void elastic_slider_set_value(lv_obj_t* slider, int32_t value, bool anim) {
    lv_slider_set_value(slider, value, anim ? LV_ANIM_ON : LV_ANIM_OFF);
}

void elastic_slider_set_spring_constant(lv_obj_t* slider, float spring_constant) {
    elastic_slider_t* data = find_slider_data(slider);
    if (data) {
        data->spring_constant = spring_constant;
    }
}

void elastic_slider_set_damping_factor(lv_obj_t* slider, float damping_factor) {
    elastic_slider_t* data = find_slider_data(slider);
    if (data) {
        data->damping_factor = damping_factor;
    }
}

static void start_elastic_animation(elastic_slider_t* data) {
    if (!data->animation_active) {
        // Create timer if it doesn't exist
        if (data->timer == NULL) {
            data->timer = lv_timer_create(elastic_animation_timer_cb, ELASTIC_UPDATE_PERIOD_MS, data);
        } else {
            lv_timer_resume(data->timer);
        }
        data->animation_active = true;
    }
}

static void stop_elastic_animation(elastic_slider_t* data) {
    if (data->animation_active) {
        if (data->timer != NULL) {
            lv_timer_pause(data->timer);
        }
        data->animation_active = false;
        data->velocity = 0.0f;
    }
}

static void elastic_animation_timer_cb(lv_timer_t* timer) {
    elastic_slider_t* data = (elastic_slider_t*)timer->user_data;

    // Get current value
    int32_t current_value = lv_slider_get_value(data->slider);

    // Calculate distance from rest position
    float displacement = current_value - data->rest_value;

    // Calculate spring force (F = -kx)
    float force = -data->spring_constant * displacement;

    // Update velocity (a = F/m, where we assume m=1 for simplicity)
    data->velocity += force;

    // Apply damping
    data->velocity *= data->damping_factor;

    // Update position
    int32_t new_value = current_value + (int32_t)data->velocity;

    // Clamp to slider range
    int32_t min_value = lv_slider_get_min_value(data->slider);
    int32_t max_value = lv_slider_get_max_value(data->slider);
    if (new_value < min_value) {
        new_value = min_value;
        data->velocity = -data->velocity * 0.5f; // Bounce with energy loss
    } else if (new_value > max_value) {
        new_value = max_value;
        data->velocity = -data->velocity * 0.5f; // Bounce with energy loss
    }

    // Update slider
    lv_slider_set_value(data->slider, new_value, LV_ANIM_OFF);

    // Call value changed callback if set
    if (data->value_changed_cb) {
        data->value_changed_cb(new_value, data->user_data);
    }

    // Stop animation if we're close enough to rest value and velocity is small
    if (fabsf(displacement) < 1.0f && fabsf(data->velocity) < ELASTIC_MIN_VELOCITY) {
        lv_slider_set_value(data->slider, data->rest_value, LV_ANIM_OFF);
        stop_elastic_animation(data);
    }
}

static void elastic_slider_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* slider = lv_event_get_target(e);
    elastic_slider_t* data = find_slider_data(slider);

    if (!data) return;

    if (code == LV_EVENT_RELEASED) {
        // When user releases the slider, start the elastic animation
        start_elastic_animation(data);
    }
    else if (code == LV_EVENT_PRESSED) {
        // When user presses the slider, stop the elastic animation
        stop_elastic_animation(data);
    }
    else if (code == LV_EVENT_VALUE_CHANGED) {
        // Call value changed callback if set
        if (data->value_changed_cb) {
            data->value_changed_cb(lv_slider_get_value(slider), data->user_data);
        }
    }
    else if (code == LV_EVENT_DELETE) {
        // Clean up when slider is deleted
        if (data->timer != NULL) {
            lv_timer_del(data->timer);
        }
        remove_slider_data(slider);
        lv_mem_free(data);
    }
}