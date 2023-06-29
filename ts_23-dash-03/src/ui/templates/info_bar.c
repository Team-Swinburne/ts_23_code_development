/********************************************************************************
* @file    info_bar.c
* @author  Nam Tran
* @version 2.0
* @date    22/05/2022
* @brief   Custom component - Info bar template on the home screen inheritated from lv_obj_t
*********************************************************************************
*/
#include "info_bar.h"



/* -------------------------------------------------------------------------- */
/*                              GLOBAL FUNCTIONS                              */
/* -------------------------------------------------------------------------- */

/*  Due to memory leak when creating a custom object, 
    accessing children's will now be done through lv_obj_get_child(parent, idx)
*/
lv_obj_t * info_bar_create (lv_obj_t * parent, const char * label_text, const int min_val, const int max_val)
{
    /* ------------------------------- Containter ------------------------------- */
    lv_obj_t * ui_cont = lv_obj_create(parent);

    lv_obj_set_size(ui_cont,280,75);
    lv_obj_set_align(ui_cont, LV_ALIGN_CENTER);

    // style
    lv_obj_clear_flag(ui_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(ui_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(ui_cont, 6, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* ---------------------------------- Label --------------------------------- */
    lv_obj_t * ui_label = lv_label_create(ui_cont);

    lv_obj_set_size(ui_label,LV_SIZE_CONTENT,LV_SIZE_CONTENT);

    lv_label_set_text_static(ui_label,label_text);
    lv_obj_set_style_text_font(ui_label, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
    

    /* ------------------------------ Display value ----------------------------- */
    lv_obj_t * ui_value = lv_label_create(ui_cont);

    lv_obj_set_size(ui_value,LV_SIZE_CONTENT,LV_SIZE_CONTENT);
    lv_obj_set_align(ui_value,LV_ALIGN_BOTTOM_RIGHT);

    lv_label_set_text(ui_value,"-1 C");
    lv_obj_set_style_text_font(ui_value, &lv_font_montserrat_30, LV_PART_MAIN | LV_STATE_DEFAULT);


    /* ----------------------------------- Bar ---------------------------------- */
    static lv_style_t style_bg;
    static lv_style_t style_indic;

    lv_style_init(&style_bg);
    lv_style_set_border_color(&style_bg, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_border_width(&style_bg, 2);
    lv_style_set_pad_all(&style_bg, 5); /*To make the indicator smaller*/
    lv_style_set_radius(&style_bg, 5);
    lv_style_set_anim_time(&style_bg, 100);

    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_radius(&style_indic, 3);

    lv_obj_t * bar = lv_bar_create(ui_cont);
    lv_obj_remove_style_all(bar);  /*To have a clean start*/
    lv_obj_add_style(bar, &style_bg, 0);
    lv_obj_add_style(bar, &style_indic, LV_PART_INDICATOR);
    lv_bar_set_range(bar, min_val, max_val);

    lv_obj_set_size(bar, 165, 33);
    lv_obj_align(bar,LV_ALIGN_BOTTOM_LEFT,5,0);

    return ui_cont;
}


void info_bar_update_value(lv_obj_t * obj, const int16_t value, const char * fmt) {
    lv_obj_t * ui_value = lv_obj_get_child(obj, INFO_BAR_VALUE_CHILD_ID);
    lv_obj_t * ui_bar = lv_obj_get_child(obj, INFO_BAR_BAR_CHILD_ID);

    lv_label_set_text_fmt(ui_value, fmt, value);
    lv_bar_set_value(ui_bar, value, LV_ANIM_ON);
}

void info_bar_update_color(lv_obj_t * obj, lv_color_t color) {
    lv_obj_t * ui_bar = lv_obj_get_child(obj, INFO_BAR_BAR_CHILD_ID);
    lv_obj_set_style_border_color(ui_bar, color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_bar, color, LV_PART_MAIN | LV_PART_INDICATOR);
}