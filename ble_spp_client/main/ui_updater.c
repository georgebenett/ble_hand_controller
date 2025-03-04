#include "ui_updater.h"
#include "esp_log.h"
#include "adc.h"
#include "ble_spp_client.h"
#include "ui/ui.h"
#include "sleep.h"
#include "elastic.h"

#define TAG "UI_UPDATER"

// Gesture detection configuration
#define SLIDE_DOWN_THRESHOLD 100  // Minimum vertical distance for slide
#define SLIDE_START_Y_THRESHOLD 50  // Maximum Y position to start slide
#define SLIDE_TIMEOUT_MS 500  // Maximum time for slide gesture

static uint8_t connection_quality = 0;
static bool gesture_test_enabled = false;

static lv_obj_t* get_current_screen(void) {
    return lv_scr_act();
}

void ui_update_connection_quality(int rssi) {
    // If RSSI is 0 or positive, consider it as disconnected
    if (rssi >= 0) {
        connection_quality = 0;
    } else {
        // Normalize RSSI to percentage
        connection_quality = ((rssi + 100) * 100) / 70;

        // Clamp percentage between 0 and 100
        if (connection_quality > 100) connection_quality = 100;

    }
    // Update the UI
    ui_update_connection_icon();
}

void ui_update_connection_icon(void) {
    if (ui_no_connection_icon == NULL) return;

    // Only update if home screen is active
    if (get_current_screen() == ui_home_screen) {
        if (!is_connect) {
            lv_img_set_src(ui_no_connection_icon, &ui_img_no_connection_png);
            return;
        }

        if (connection_quality < 15) {
            lv_img_set_src(ui_no_connection_icon, &ui_img_33_connection_png);
        }
        else if (connection_quality < 35) {
            lv_img_set_src(ui_no_connection_icon, &ui_img_66_connection_png);
        }
        else {
            lv_img_set_src(ui_no_connection_icon, &ui_img_full_connection_png);
        }
    }
}

static void gesture_test_cb(lv_event_t *e) {
    if (!gesture_test_enabled) return;

    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    lv_obj_t *current_screen = lv_scr_act();

    switch (dir) {
        case LV_DIR_LEFT:
            ESP_LOGI(TAG, "Gesture: SWIPE LEFT");
            break;
        case LV_DIR_RIGHT:
            ESP_LOGI(TAG, "Gesture: SWIPE RIGHT");
            break;
        case LV_DIR_TOP:
            ESP_LOGI(TAG, "Gesture: SWIPE UP");
            // If on shutdown screen, go back to home screen with animation
            if (current_screen == ui_shutdown_screen) {
                // Load home screen with animation (slide up from bottom)
                lv_scr_load_anim(ui_home_screen, LV_SCR_LOAD_ANIM_MOVE_TOP, 300, 0, false);
            }
            break;
        case LV_DIR_BOTTOM:
            ESP_LOGI(TAG, "Gesture: SWIPE DOWN");
            // If on home screen, go to shutdown screen with animation
            if (current_screen == ui_home_screen) {
                // Load shutdown screen with animation (slide down from top)
                lv_scr_load_anim(ui_shutdown_screen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 300, 0, false);
            }
            break;
        default:
            // Don't log anything for no gesture
            break;
    }
}

void ui_init_gesture_test(void) {
    // Add gesture event callback to home screen
    lv_obj_add_event_cb(ui_home_screen, gesture_test_cb, LV_EVENT_GESTURE, NULL);

    // Also add gesture event callback to shutdown screen
    lv_obj_add_event_cb(ui_shutdown_screen, gesture_test_cb, LV_EVENT_GESTURE, NULL);

    ESP_LOGI(TAG, "Gesture test initialized on both screens");
}

void ui_enable_gesture_test(bool enable) {
    gesture_test_enabled = enable;
    ESP_LOGI(TAG, "Gesture test %s", enable ? "enabled" : "disabled");
}

void ui_updater_init(void) {
    // Initialize gesture handling
    ui_init_gesture_handling();

    // Initialize gesture test
    ui_init_gesture_test();

    // Enable gesture test by default
    ui_enable_gesture_test(true);

    // Initialize the turnoff slider
    ui_init_turnoff_slider();
}

void ui_update_speed(int32_t value) {
    if (ui_Label1 == NULL) return;

    // Only update if home screen is active
    if (get_current_screen() == ui_home_screen) {
        lv_label_set_text_fmt(ui_Label1, "%ld", value);
    }
}

int get_connection_quality(void) {
    return connection_quality;
}

void ui_handle_touch_event(lv_event_t * e) {
    // This function is now empty as we're removing the custom slide implementation
    // We'll rely on LVGL's built-in gesture detection via LV_EVENT_GESTURE
}

void ui_init_gesture_handling(void) {
    // Make sure both screens can receive gestures
    lv_obj_clear_flag(ui_home_screen, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_flag(ui_home_screen, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_clear_flag(ui_shutdown_screen, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_flag(ui_shutdown_screen, LV_OBJ_FLAG_CLICKABLE);
}

// Add this function to handle elastic slider value changes
static void turnoff_slider_value_changed_cb(int32_t value, void* user_data) {
    ESP_LOGI("APP", "Slider value: %ld", value);

    // If slider reaches 100, trigger shutdown
    if (value >= 100) {
        vTaskDelay(100);
        enter_deep_sleep();
    }
}

// Modified function to initialize the slider
void ui_init_turnoff_slider(void) {
    if (ui_turnoffslider == NULL) return;

    // Replace the standard slider with an elastic one
    lv_obj_t* elastic_slider = elastic_slider_create(lv_obj_get_parent(ui_turnoffslider), 0, 0, 100);

    // Copy the position and size of the original slider
    lv_obj_set_pos(elastic_slider, lv_obj_get_x(ui_turnoffslider), lv_obj_get_y(ui_turnoffslider));
    lv_obj_set_size(elastic_slider, lv_obj_get_width(ui_turnoffslider), lv_obj_get_height(ui_turnoffslider));

    // Set callback for value changes
    elastic_slider_set_value_changed_cb(elastic_slider, turnoff_slider_value_changed_cb, NULL);

    // Delete the original slider
    lv_obj_del(ui_turnoffslider);

    // Update the global reference
    ui_turnoffslider = elastic_slider;
}
