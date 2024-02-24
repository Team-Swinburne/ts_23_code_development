/********************************************************************************
* @file    main.cpp
* @author  Nam Tran
* @version 1.0
* @date    19-04-2020
* @brief   D
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

#include "backend.hpp"
#include "ui/screen_splash.h"
#include "ui/screen_home.h"
/* -------------------------------------------------------------------------- */
/*                                   DEFINES                                  */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                              STATIC PROTOTYPE                              */
/* -------------------------------------------------------------------------- */
void load_nav();
void rotate_screen();
/* -------------------------------------------------------------------------- */
/*                                  FUNCTIONS                                 */
/* -------------------------------------------------------------------------- */
void load_nav_timer(lv_timer_t *timer)
{
	load_nav();
}

void rotate_screen_timer(lv_timer_t *timer)
{
	rotate_screen();
}

int main() {
	lv_init();
	backend_init();
	/*Set default screen to black*/
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);

	/*Load splash screen*/
	load_splash();

	/*Load navigaion after 3s of start (3s animation + 2s delay)*/
	lv_timer_t * timer = lv_timer_create_basic();
	lv_timer_set_cb(timer,load_nav_timer);
	lv_timer_set_period(timer,4000);
    
	lv_timer_set_repeat_count(timer,1);
    //load_nav();

	lv_timer_t * timer1 = lv_timer_create_basic();
	lv_timer_set_cb(timer1,rotate_screen_timer);
	lv_timer_set_period(timer1,3000);
    
	lv_timer_set_repeat_count(timer1,1);
	
	backend_loop();
}


/**
 * @brief Main navigation of the dash using tileview
 * 
 */
void load_nav()
{
	/*Create the screen and tileview*/
    lv_obj_t * scr = lv_obj_create(NULL);
    lv_obj_t * tv = lv_tileview_create(scr);

	/*Tile1: just a label*/
    lv_obj_t * tile1 = lv_tileview_add_tile(tv, 0, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    lv_obj_clear_flag(tile1, LV_OBJ_FLAG_SCROLLABLE);
    load_home(tile1);
    
	lv_scr_load(scr);

}

/**
 * @brief Rotate screen 180 deg
 * 
 */
void rotate_screen()
{
    lv_disp_set_rotation(NULL,LV_DISP_ROT_180);
}