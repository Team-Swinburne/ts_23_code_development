/********************************************************************************
* @file    touchpad.c
* @author  Internet
* @version 1.0
* @date    07/06/22
* @brief   touchpad driver for the dash
*********************************************************************************
*/

/* -------------------------------------------------------------------------- */
/*                                   INCLUDE                                  */
/* -------------------------------------------------------------------------- */
#include "stm32f4xx.h"

#include "..\bsp_drivers\stm32469i_discovery.h"
#include "..\bsp_drivers\stm32469i_discovery_ts.h"
#include "..\tft\tft.h"
#include "lvgl.h"
#include "touchpad.h"
/* -------------------------------------------------------------------------- */
/*                              STATIC PROTOTYPE                              */
/* -------------------------------------------------------------------------- */
static void touchpad_read(lv_indev_drv_t * drv, lv_indev_data_t *data);

/* -------------------------------------------------------------------------- */
/*                               STATIC VARIABLE                              */
/* -------------------------------------------------------------------------- */
static TS_StateTypeDef  TS_State;

/* -------------------------------------------------------------------------- */
/*                              GLOBAL FUNCTIONS                              */
/* -------------------------------------------------------------------------- */

/**
 * Initialize your input devices here
 */
void touchpad_init(void)
{
    BSP_TS_Init(TFT_HOR_RES, TFT_VER_RES);
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);            /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;       /*This function will be called periodically (by the library) to get the mouse position and state*/
    lv_indev_drv_register(&indev_drv);
}

/* -------------------------------------------------------------------------- */
/*                               STATIC FUNCTION                              */
/* -------------------------------------------------------------------------- */

/**
 * Read an input device
 * @param indev_id id of the input device to read
 * @param x put the x coordinate here
 * @param y put the y coordinate here
 * @return true: the device is pressed, false: released
 */
static void touchpad_read(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
    // Read your touchpad
    //printf("I'm in here\r\n");
    //printf(" Don't Touch me you pervert! Points touched: %d \r\n",TS_State.touchDetected);
    BSP_TS_GetState(&TS_State);
    if(TS_State.touchDetected) {
        data->point.x = TS_State.touchX[0];
        data->point.y = TS_State.touchY[0];
        data->state = LV_INDEV_STATE_PR;
        //printf(" Don't Touch me you pervert! Touched at %d, %d \r\n",data->point.x,data->point.y);
    } else {
        data->point.x = 0;
        data->point.y = 0;
        data->state = LV_INDEV_STATE_REL;
    }
    //return false;   //false: no more data to read because we are no buffering
}