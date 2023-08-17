/********************************************************************************
* @file    warning.h
* @author  Nam Tran
* @date    16/06/2022
* @brief   Quick function to draw and hide warning/error
*********************************************************************************
*/

/* -------------------------------------------------------------------------- */
/*                                   INCLUDE                                  */
/* -------------------------------------------------------------------------- */
#include "warning_error.h"

/* -------------------------------------------------------------------------- */
/*                              STATICS FUNCTIONS                             */
/* -------------------------------------------------------------------------- */
static lv_obj_t * get_error_label(lv_obj_t * obj)
{
    return lv_obj_get_child(lv_obj_get_child(obj,0),0);
}
/* -------------------------------------------------------------------------- */
/*                              GLOBAL FUNCTIONS                              */
/* -------------------------------------------------------------------------- */
lv_obj_t * button_warning_create(lv_obj_t * parent)
{
    /*Init style*/
    static lv_style_t warning_line_style;
    lv_style_init(&warning_line_style);
    lv_style_set_line_width(&warning_line_style, 30);
    lv_style_set_line_color(&warning_line_style, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_line_rounded(&warning_line_style, true);


    lv_obj_t * warning_line = lv_line_create(parent);
    
    static lv_point_t line_points[] = {{0,0},{800,0},{800, 480},{0, 480},{0,0}};
    lv_line_set_points(warning_line,line_points,5);

    lv_obj_add_style(warning_line,&warning_line_style,0);
    lv_obj_add_flag(warning_line, LV_OBJ_FLAG_HIDDEN);

    return warning_line;
}

void show_precharge_warning(lv_obj_t * obj)
{
    lv_obj_set_style_line_color(obj,lv_palette_main(LV_PALETTE_ORANGE),0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);

}

void show_drive_warning(lv_obj_t * obj)
{
    lv_obj_set_style_line_color(obj,lv_palette_darken(LV_PALETTE_LIGHT_BLUE,4),0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

void show_trailbraking_warning(lv_obj_t * obj)
{
    lv_obj_set_style_line_color(obj,lv_palette_main(LV_PALETTE_PINK),0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

void hide_warning(lv_obj_t * obj)
{
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

/* --------------------------------- Errors --------------------------------- */
lv_obj_t * critical_error_create(lv_obj_t * parent)
{
    lv_obj_t * outter_cont = lv_obj_create(parent);

    lv_obj_set_size(outter_cont,800,480);
    lv_obj_set_style_border_color(outter_cont,lv_palette_darken(LV_PALETTE_RED,3),0);
    lv_obj_set_style_border_width(outter_cont,15,0);
    lv_obj_set_style_bg_color(outter_cont,lv_color_black(),0);
    lv_obj_set_style_bg_opa(outter_cont,LV_OPA_50,0);
    lv_obj_clear_flag(outter_cont, LV_OBJ_FLAG_SCROLLABLE);
    
    lv_obj_t * inner_cont = lv_obj_create(outter_cont);

    lv_obj_set_size(inner_cont,LV_SIZE_CONTENT,LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(inner_cont,lv_color_black(),0);
    lv_obj_set_style_bg_opa(inner_cont,LV_OPA_80,0);
    lv_obj_center(inner_cont);

    lv_obj_t * label = lv_label_create(inner_cont);
    lv_obj_set_style_text_color(label,lv_palette_lighten(LV_PALETTE_RED,1),0);
    lv_obj_set_style_text_font(label,&lv_font_montserrat_32,0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_add_flag(outter_cont, LV_OBJ_FLAG_HIDDEN);
    return outter_cont;
}

void show_IMD_error(lv_obj_t * obj)
{
    lv_obj_t * label = get_error_label(obj);
    lv_label_set_text(label,"Critical error: IMD TRIGGERED");
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);

}

void show_BSPD_error(lv_obj_t * obj)
{
    lv_obj_t * label = get_error_label(obj);
    lv_label_set_text(label,"Critical error: BSPD TRIGGERED");
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);

}

void show_PDOC_error(lv_obj_t * obj, uint8_t device)
{
    lv_obj_t * label = get_error_label(obj);
    switch (device)
    {
    case 0: // precharge
        lv_label_set_text(label,"! Critical error !\n PRECHARGE PDOC TRIGGERED");
        break;
    case 1: // discharge
        lv_label_set_text(label,"! Critical error !\n DISCHARGE PDOC TRIGGERED");
    default:
        break;
    }
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);

}
void show_AMS_error(lv_obj_t * obj, uint8_t error_code)
{
    lv_obj_t * label = get_error_label(obj);
    if (error_code & 0b00010000)
    {
        lv_label_set_text(label,"Critical AMS error: ORION TIMEOUT");
    }
    if (error_code & 0b0000100)
    {
        lv_label_set_text(label,"Critical AMS error: LOW CELL VOLTAGE");
    }
    if (error_code & 0b00000100)
    {
        lv_label_set_text(label,"Critical AMS error: HIGH CELL VOLTAGE");
    }
    if (error_code & 0b00000010)
    {
        lv_label_set_text(label,"Critical AMS error: PACK OVERHEAT");
    }
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);

}

void hide_critical_error(lv_obj_t * obj)
{
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
}
