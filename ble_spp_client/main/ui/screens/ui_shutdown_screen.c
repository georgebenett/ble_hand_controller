// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "../ui.h"

void ui_shutdown_screen_screen_init(void)
{
    ui_shutdown_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_shutdown_screen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_shutdown_screen, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_shutdown_screen, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_shutdown = lv_label_create(ui_shutdown_screen);
    lv_obj_set_width(ui_shutdown, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_shutdown, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_shutdown, 0);
    lv_obj_set_y(ui_shutdown, -30);
    lv_obj_set_align(ui_shutdown, LV_ALIGN_CENTER);
    lv_label_set_text(ui_shutdown, "turning off");
    lv_obj_set_style_text_font(ui_shutdown, &ui_font_bebas75, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Bar4 = lv_bar_create(ui_shutdown_screen);
    lv_bar_set_value(ui_Bar4, 50, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_Bar4, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_Bar4, 300);
    lv_obj_set_height(ui_Bar4, 20);
    lv_obj_set_x(ui_Bar4, 0);
    lv_obj_set_y(ui_Bar4, 60);
    lv_obj_set_align(ui_Bar4, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_Bar4, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Bar4, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);


}
