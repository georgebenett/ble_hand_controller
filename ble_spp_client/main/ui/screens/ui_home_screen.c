// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "../ui.h"

void ui_home_screen_screen_init(void)
{
    ui_home_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_home_screen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_home_screen, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_home_screen, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label1 = lv_label_create(ui_home_screen);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, 0);
    lv_obj_set_y(ui_Label1, -20);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "0");
    lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Label1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label1, &ui_font_bebas120, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label2 = lv_label_create(ui_home_screen);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label2, 0);
    lv_obj_set_y(ui_Label2, 40);
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "km/h");
    lv_obj_set_style_text_font(ui_Label2, &ui_font_bebas_small, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Bar2 = lv_bar_create(ui_home_screen);
    lv_bar_set_value(ui_Bar2, 25, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_Bar2, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_Bar2, 150);
    lv_obj_set_height(ui_Bar2, 10);
    lv_obj_set_x(ui_Bar2, 275);
    lv_obj_set_y(ui_Bar2, 131);
    lv_obj_set_align(ui_Bar2, LV_ALIGN_CENTER);

    ui_Bar3 = lv_bar_create(ui_home_screen);
    lv_bar_set_value(ui_Bar3, 25, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_Bar3, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_Bar3, 150);
    lv_obj_set_height(ui_Bar3, 10);
    lv_obj_set_x(ui_Bar3, 278);
    lv_obj_set_y(ui_Bar3, 70);
    lv_obj_set_align(ui_Bar3, LV_ALIGN_CENTER);

    ui_controller_battery_icon = lv_img_create(ui_home_screen);
    lv_img_set_src(ui_controller_battery_icon, &ui_img_battery_icon_png);
    lv_obj_set_width(ui_controller_battery_icon, LV_SIZE_CONTENT);   /// 50
    lv_obj_set_height(ui_controller_battery_icon, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_x(ui_controller_battery_icon, 80);
    lv_obj_set_y(ui_controller_battery_icon, -120);
    lv_obj_set_align(ui_controller_battery_icon, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_controller_battery_icon, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_controller_battery_icon, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_controller_battery_icon, 200);

    ui_controller_battery_text = lv_label_create(ui_home_screen);
    lv_obj_set_width(ui_controller_battery_text, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_controller_battery_text, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_controller_battery_text, 80);
    lv_obj_set_y(ui_controller_battery_text, -120);
    lv_obj_set_align(ui_controller_battery_text, LV_ALIGN_CENTER);
    lv_label_set_text(ui_controller_battery_text, "90");
    lv_obj_set_style_text_color(ui_controller_battery_text, lv_color_hex(0x232222), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_controller_battery_text, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_controller_battery_text, &ui_font_bebas_small, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_no_connection_icon = lv_img_create(ui_home_screen);
    lv_img_set_src(ui_no_connection_icon, &ui_img_no_connection_png);
    lv_obj_set_width(ui_no_connection_icon, LV_SIZE_CONTENT);   /// 50
    lv_obj_set_height(ui_no_connection_icon, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_x(ui_no_connection_icon, 45);
    lv_obj_set_y(ui_no_connection_icon, -121);
    lv_obj_set_align(ui_no_connection_icon, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_no_connection_icon, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_no_connection_icon, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_no_connection_icon, 135);

    ui_Label5 = lv_label_create(ui_home_screen);
    lv_obj_set_width(ui_Label5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label5, -60);
    lv_obj_set_y(ui_Label5, 120);
    lv_obj_set_align(ui_Label5, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label5, "trip");
    lv_obj_set_style_text_font(ui_Label5, &ui_font_bebas_small, LV_PART_MAIN | LV_STATE_DEFAULT);


}