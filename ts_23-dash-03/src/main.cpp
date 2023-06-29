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
// #include "ui/screen_home.h"
#include "ui/ui.h"
/* -------------------------------------------------------------------------- */
/*                                   DEFINES                                  */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                              STATIC PROTOTYPE                              */
/* -------------------------------------------------------------------------- */
void load_nav();
/* -------------------------------------------------------------------------- */
/*                                  FUNCTIONS                                 */
/* -------------------------------------------------------------------------- */
void load_nav_timer(lv_timer_t *timer)
{
	// load_nav();
	ui_init();
}

int main() {
	lv_init();
	backend_init();
	/*Set default screen to black*/
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);

	/*Load splash screen*/
	load_splash();

	/*Load navigaion after 6s of start (3s animation + 3s delay)*/
	lv_timer_t * timer = lv_timer_create_basic();
	lv_timer_set_cb(timer,load_nav_timer);
	lv_timer_set_period(timer,3000);
    
	lv_timer_set_repeat_count(timer,1);
    //load_nav();
	
	backend_loop();
}


// /**
//  * @brief Main navigation of the dash using tileview
//  * 
//  */
// void load_nav()
// {
// 	/*Create the screen and tileview*/
//     lv_obj_t * scr = lv_obj_create(NULL);
//     lv_obj_t * tv = lv_tileview_create(scr);

// 	/*Tile1: just a label*/
//     lv_obj_t * tile1 = lv_tileview_add_tile(tv, 0, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
//     lv_obj_clear_flag(tile1, LV_OBJ_FLAG_SCROLLABLE);
//     load_home(tile1);
    


//     /*Tile2: a button*/
//     lv_obj_t * tile2 = lv_tileview_add_tile(tv, 1, 0, LV_DIR_LEFT | LV_DIR_RIGHT);

//     lv_obj_t * btn = lv_btn_create(tile2);

//     lv_obj_t * label = lv_label_create(btn);
//     lv_label_set_text(label, "Scroll up or right");

//     lv_obj_set_size(btn, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
//     lv_obj_center(btn);

// 	/*Load the screen*/
// 	lv_scr_load(scr);
	
// }