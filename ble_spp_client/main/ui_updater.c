#include "ui_updater.h"
#include "esp_log.h"
#include "adc.h"
#include "ble_spp_client.h"
#include "ui/ui.h"
#include "sleep.h"

#define TAG "UI_UPDATER"

// Gesture detection configuration
#define SLIDE_DOWN_THRESHOLD 100  // Minimum vertical distance for slide
#define SLIDE_START_Y_THRESHOLD 50  // Maximum Y position to start slide
#define SLIDE_TIMEOUT_MS 500  // Maximum time for slide gesture

typedef struct {
    int16_t start_x;
    int16_t start_y;
    uint32_t start_time;
    bool tracking;
} gesture_state_t;

static gesture_state_t gesture_state = {0};

static uint8_t connection_quality = 0;

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

void ui_updater_init(void) {
    // Initialize gesture handling
    ui_init_gesture_handling();
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
    lv_event_code_t code = lv_event_get_code(e);

    // Only track gestures on home screen
    if (lv_scr_act() != ui_home_screen) {
        return;
    }

    if (code == LV_EVENT_PRESSED) {
        lv_indev_t * indev = lv_indev_get_act();
        lv_point_t point;
        lv_indev_get_point(indev, &point);

        // Start tracking if touch begins near top of screen
        if (point.y < SLIDE_START_Y_THRESHOLD) {
            gesture_state.start_x = point.x;
            gesture_state.start_y = point.y;
            gesture_state.start_time = lv_tick_get();
            gesture_state.tracking = true;
        }
    }
    else if (code == LV_EVENT_PRESSING) {
        if (gesture_state.tracking) {
            lv_indev_t * indev = lv_indev_get_act();
            lv_point_t point;
            lv_indev_get_point(indev, &point);

            // Calculate vertical distance and time
            int16_t delta_y = point.y - gesture_state.start_y;
            uint32_t delta_time = lv_tick_get() - gesture_state.start_time;

            // Check if it's a valid slide down gesture
            if (delta_y > SLIDE_DOWN_THRESHOLD && delta_time < SLIDE_TIMEOUT_MS) {
                lv_disp_load_scr(ui_shutdown_screen);
                gesture_state.tracking = false;
            }
        }
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        gesture_state.tracking = false;
    }
}

void ui_init_gesture_handling(void) {
    // Add touch event handler to home screen
    lv_obj_add_event_cb(ui_home_screen, ui_handle_touch_event, LV_EVENT_ALL, NULL);
}
