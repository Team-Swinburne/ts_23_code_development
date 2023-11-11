/********************************************************************************
* @file    splash.c
* @author  Andrew Gray, Christian Lazarovski, Tansel Kahrahman, Nam Tran
* @version 2.0
* @date    19/04/2022
* @brief   Splash screen
*********************************************************************************
* @details 
* ----------------------------
* Definitions and declarations
* 
* ----------------------------
* Functions*
* 
*********************************************************************************
*/
 
/* -------------------------------------------------------------------------- */
/*                                   INCLUDES                                 */
/* -------------------------------------------------------------------------- */
#include "lvgl.h"
#include "lv_conf.h"

#include "screen_splash.h"

// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>

/* -------------------------------------------------------------------------- */
/*                                 DECLARATIONS                               */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                   TYPEDEF                                  */
/* -------------------------------------------------------------------------- */
 
/* -------------------------------------------------------------------------- */
/*                                  FUNCTIONS                                 */
/* -------------------------------------------------------------------------- */

// load the splash screen
void load_splash()
{
    LV_IMG_DECLARE(ts20splash);

    // create and style the screen
    lv_obj_t * scr = lv_obj_create(NULL);
    lv_obj_set_size(scr, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);

    //
    lv_disp_set_bg_color(lv_disp_get_default(),lv_color_black());

    lv_obj_t * wp = lv_img_create(scr);
    lv_img_set_src(wp, &ts20splash);
    lv_obj_set_pos(wp, 0, 0);
    lv_obj_align(wp, LV_ALIGN_CENTER, 0, 0);

    lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_FADE_ON, 3000, 500, false);
}