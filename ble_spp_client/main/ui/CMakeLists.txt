SET(SOURCES screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_106406083.c
    images/ui_img_battery_100_png.c
    images/ui_img_battery101_png.c
    images/ui_img_battery_0_png.c
    fonts/ui_font_bebas120.c
    fonts/ui_font_bebas_small.c)

add_library(ui ${SOURCES})
