/********************************************************************************
* @file    info_box.c
* @author  Nam Tran
* @version 2.0
* @date    22/05/2022
* @brief   Custom component - Info box template on the home screen inheritated from lv_obj_t
*********************************************************************************
*/
#include "info_box.h"



/* -------------------------------------------------------------------------- */
/*                              GLOBAL FUNCTIONS                              */
/* -------------------------------------------------------------------------- */

/*  Due to memory leak when creating a custom object, 
    accessing children's will now be done through lv_obj_get_child(parent, idx)
*/
lv_obj_t * info_box_create (lv_obj_t * parent, const char * label_text)
{
    //containter
    lv_obj_t * ui_cont = lv_obj_create(parent);

    lv_obj_set_size(ui_cont,200,85);
    lv_obj_set_align(ui_cont, LV_ALIGN_CENTER);


    lv_obj_clear_flag(ui_cont, LV_OBJ_FLAG_SCROLLABLE);

    //lv_obj_set_style_bg_color(ui_cont, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);


    //label
    lv_obj_t * ui_label = lv_label_create(ui_cont);

    lv_obj_set_size(ui_label,LV_SIZE_CONTENT,LV_SIZE_CONTENT);

    lv_obj_set_pos(ui_label, -9,-10);
    lv_label_set_text_static(ui_label,label_text);
    
    lv_obj_set_style_text_font(ui_label, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);


    // display value
    lv_obj_t * ui_value = lv_label_create(ui_cont);

    lv_obj_set_size(ui_value,LV_SIZE_CONTENT,LV_SIZE_CONTENT);
    lv_obj_set_pos(ui_value,-3,15);
    lv_obj_set_align(ui_value,LV_ALIGN_BOTTOM_RIGHT);

    lv_label_set_text(ui_value,"-1 C");
    lv_obj_set_style_text_font(ui_value, &lv_font_montserrat_30, LV_PART_MAIN | LV_STATE_DEFAULT);


    return ui_cont;
}

void info_box_update_value(lv_obj_t * obj, const int value, const char * fmt) {
    lv_obj_t * ui_value = lv_obj_get_child(obj, INFO_BOX_VALUE_CHILD_ID);
    lv_label_set_text_fmt(ui_value, fmt, value);
}

void info_box_update_value_float(lv_obj_t * obj, const float value, const char * fmt) {
    lv_obj_t * ui_value = lv_obj_get_child(obj, INFO_BOX_VALUE_CHILD_ID);
    lv_label_set_text_fmt(ui_value, fmt, value);
}