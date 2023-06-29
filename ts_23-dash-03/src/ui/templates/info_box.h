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
enum info_box_child_id 
{
    INFO_BOX_LABEL_CHILD_ID,
    INFO_BOX_VALUE_CHILD_ID,
    INFO_BOX_UNIT_CHILD_ID
};

/* -------------------------------------------------------------------------- */
/*                              GLOBAL PROTOTYPE                              */
/* -------------------------------------------------------------------------- */

// Followed this tutorial https://github.com/lvgl/lv_demos/blob/master/src/lv_demo_music/lv_demo_music_list.c
/**
 * @brief 
 * 
 * @param parent the parent object
 * @param label_text box's label
 * @return lv_obj_t* 
 */
lv_obj_t * info_box_create (lv_obj_t * parent,  const char * label_text);

void info_box_update_value(lv_obj_t * obj, const int value, const char * fmt);
void info_box_update_value_float(lv_obj_t * obj, const float value, const char * fmt);
/* --------------------------------- GUARDS --------------------------------- */
#ifdef __cplusplus
}
#endif