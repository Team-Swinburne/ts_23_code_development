/********************************************************************************
* @file    warning.h
* @author  Nam Tran
* @date    16/06/2022
* @brief   Quick function to draw and hide warning/error
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

/* -------------------------------------------------------------------------- */
/*                              GLOBAL PROTOTYPE                              */
/* -------------------------------------------------------------------------- */
/**
 * @brief Create button warning line, default to hidden, 
 * 
 * @param parent the parrent obj
 * @return lv_obj_t*
 */
lv_obj_t * button_warning_create(lv_obj_t * parent);

void show_precharge_warning(lv_obj_t * obj);
void show_drive_warning(lv_obj_t * obj);
void show_trailbraking_warning(lv_obj_t * obj);
void hide_warning(lv_obj_t * obj);


/**
 * @brief Create critical error pop up, default to hidden
 * 
 * @param parent the parrent obj
 * @return lv_obj_t* 
 */
lv_obj_t * critical_error_create(lv_obj_t * parent);

void show_IMD_error(lv_obj_t * obj);
void show_PDOC_error(lv_obj_t * obj, uint8_t device);
void show_AMS_error(lv_obj_t * obj, uint8_t error_code);
void show_BSPD_error(lv_obj_t * obj);
void hide_critical_error(lv_obj_t * obj);

/* --------------------------------- GUARDS --------------------------------- */
#ifdef __cplusplus
}
#endif