/********************************************************************************
* @file    info.h
* @author  Nam Tran
* @version 2.0
* @date    22/05/2022
* @brief   Custom component - Info box template on the home screen inheritated from lv_obj_t
*********************************************************************************
* @details 
* ----------------------------
* Definitions and declarations
* 
* ----------------------------
* Functions
*   load_home() prototype
*********************************************************************************
*/

/* --------------------------------- GUARDS --------------------------------- */
#pragma once
#ifdef __cplusplus
 extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*                                   INCLUDES                                 */
/* -------------------------------------------------------------------------- */
#include "lvgl.h"

/* -------------------------------------------------------------------------- */
/*                                   DEFINES                                  */
/* -------------------------------------------------------------------------- */
enum info_bar_child_id 
{
    INFO_BAR_LABEL_CHILD_ID,
    INFO_BAR_VALUE_CHILD_ID,
    INFO_BAR_BAR_CHILD_ID
};

/* -------------------------------------------------------------------------- */
/*                              GLOBAL PROTOTYPE                              */
/* -------------------------------------------------------------------------- */

// Followed this tutorial https://github.com/lvgl/lv_demos/blob/master/src/lv_demo_music/lv_demo_music_list.c
/**
 * @brief 
 * 
 * @param parent the parent object
 * @param label_text bar's label
 * @return lv_obj_t* 
 */
lv_obj_t * info_bar_create (lv_obj_t * parent,  const char * label_text, const int min_val, const int max_val);

/**
 * @brief 
 * 
 * @param obj 
 * @param text 
 * @param ... 
 */
void info_bar_update_value(lv_obj_t * obj, const int16_t value, const char * fmt);

void info_bar_update_color(lv_obj_t * obj, lv_color_t color);
/* --------------------------------- GUARDS --------------------------------- */
#ifdef __cplusplus
}
#endif