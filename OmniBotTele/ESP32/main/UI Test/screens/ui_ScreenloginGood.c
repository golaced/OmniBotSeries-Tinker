// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: ESP32Box

#include "../ui.h"

void ui_ScreenloginGood_screen_init(void)
{
    ui_ScreenloginGood = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScreenloginGood, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label20 = lv_label_create(ui_ScreenloginGood);
    lv_obj_set_width(ui_Label20, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label20, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label20, 2);
    lv_obj_set_y(ui_Label20, 70);
    lv_obj_set_align(ui_Label20, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label20, "Connecting to Robot !");

    ui_Image2 = lv_img_create(ui_ScreenloginGood);
    lv_img_set_src(ui_Image2, &ui_img_705703283);
    lv_obj_set_width(ui_Image2, LV_SIZE_CONTENT);   /// 124
    lv_obj_set_height(ui_Image2, LV_SIZE_CONTENT);    /// 124
    lv_obj_set_x(ui_Image2, 4);
    lv_obj_set_y(ui_Image2, -15);
    lv_obj_set_align(ui_Image2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image2, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_Image2, 200);

    lv_obj_add_event_cb(ui_ScreenloginGood, ui_event_ScreenloginGood, LV_EVENT_ALL, NULL);

}
