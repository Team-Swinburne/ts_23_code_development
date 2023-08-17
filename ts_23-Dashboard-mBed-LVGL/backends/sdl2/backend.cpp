#include <unistd.h>
#define SDL_MAIN_HANDLED        /*To fix SDL's "undefined reference to WinMain" issue*/
#include <SDL2/SDL.h>
#include "display/monitor.h"
#include "indev/mouse.h"
#include "indev/mousewheel.h"
#include "indev/keyboard.h"
#include "sdl/sdl.h"
#include "backend.hpp"

#include "stdio.h"
/* -------------------------------------------------------------------------- */
/*                              STATIC VARIABLES                              */
/* -------------------------------------------------------------------------- */
static VehicleState		    vehicle_state={0,false,false,false,false,false,false,false,false,0};
static MotorInfo		    motor_info={0,0,0,0,0};
static AccumulatorInfo		accum_info={0,0,0.0f,0};
static MiscInfo			    misc_info={0,0,0,0,0,0};
/* -------------------------------------------------------------------------- */
/*                              STATIC FUNCTIONS                              */
/* -------------------------------------------------------------------------- */
/**
 * @brief Simulate CAN traffic
 * 
 */
static void data_gen(lv_timer_t *timer)
{
    //vehicle_state.error_ams = 0x02;
    //vehicle_state.trailbraking_active = 1;
    if (motor_info.mc_temp >= 120)
    {
        motor_info.mc_temp = 0;
        motor_info.mc_voltage = 0;
        motor_info.motor_speed = 0;
        motor_info.motor_temp = 0;
        motor_info.lgbt_temp = 0;

        accum_info.pack_voltage = 420;
        accum_info.pack_current = 0;
        accum_info.min_cell_volt = 0;
        accum_info.max_temp = 0;
        
        misc_info.lv_bus_voltage = 0;
        misc_info.throttle_pct = 0;
        misc_info.brake_pct = 0;
        misc_info.regen_pct = 0;
        misc_info.live_speed = 0;
        misc_info.max_rpm = 0;

        vehicle_state.ams_state++;
    }
    else
    {
        motor_info.mc_temp += 1.1f;
        motor_info.mc_voltage++;
        motor_info.motor_speed++;
        motor_info.motor_temp+= 1.1f;
        motor_info.lgbt_temp++;

        accum_info.pack_voltage += 5;
        accum_info.pack_current++;
        accum_info.min_cell_volt+= 1.1f;
        accum_info.max_temp+= 1.1f;

        misc_info.lv_bus_voltage+=1.1f;
        misc_info.throttle_pct++;
        misc_info.brake_pct++;
        misc_info.regen_pct++;
        misc_info.live_speed++;
        misc_info.max_rpm++;

    //     vehicle_state.ams_state++;
        // vehicle_state.apps_disagree = !vehicle_state.apps_disagree;
        // vehicle_state.trailbraking_active = !vehicle_state.trailbraking_active;
        // vehicle_state.precharge_pressed = !vehicle_state.precharge_pressed;
        // vehicle_state.drive_pressed = !vehicle_state.drive_pressed;
    }
}

/**
 * A task to measure the elapsed time for LittlevGL
 * @param data unused
 * @return never return
 */
static int tick_thread(void * data)
{
    (void)data;

    while(1) {
        SDL_Delay(5);   /*Sleep for 5 millisecond*/
        lv_tick_inc(5); /*Tell LittelvGL that 5 milliseconds were elapsed*/
    }

    return 0;
}
/* -------------------------------------------------------------------------- */
/*                              GLOBAL FUNCTIONS                              */
/* -------------------------------------------------------------------------- */
VehicleState		can_get_vehicle_state() {return vehicle_state;}
MotorInfo			can_get_motor_info() {return motor_info;}
AccumulatorInfo		can_get_accum_info() {return accum_info;}
MiscInfo			can_get_misc_info() {return misc_info;}

void backend_init(void)
{
    // Workaround for sdl2 `-m32` crash
    // https://bugs.launchpad.net/ubuntu/+source/libsdl2/+bug/1775067/comments/7
    #ifndef WIN32
        setenv("DBUS_FATAL_WARNINGS", "0", 1);
    #endif

    /* Add a display
     * Use the 'monitor' driver which creates window on PC's monitor to simulate a display*/

    static lv_disp_draw_buf_t disp_buf;
    static lv_color_t buf[SDL_HOR_RES * 10];                          /*Declare a buffer for 10 lines*/
    lv_disp_draw_buf_init(&disp_buf, buf, NULL, SDL_HOR_RES * 10);    /*Initialize the display buffer*/

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);              /*Basic initialization*/
    disp_drv.flush_cb = sdl_display_flush;    /*Used when `LV_VDB_SIZE != 0` in lv_conf.h (buffered drawing)*/
    disp_drv.draw_buf = &disp_buf;
    disp_drv.hor_res = SDL_HOR_RES;
    disp_drv.ver_res = SDL_VER_RES;
    //disp_drv.disp_fill = monitor_fill;      /*Used when `LV_VDB_SIZE == 0` in lv_conf.h (unbuffered drawing)*/
    //disp_drv.disp_map = monitor_map;        /*Used when `LV_VDB_SIZE == 0` in lv_conf.h (unbuffered drawing)*/
    lv_disp_drv_register(&disp_drv);

    /* Add the mouse as input device
     * Use the 'mouse' driver which reads the PC's mouse*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);            /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = sdl_mouse_read;       /*This function will be called periodically (by the library) to get the mouse position and state*/
    lv_indev_drv_register(&indev_drv);

    //set screen to black
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);
    
    sdl_init();

    /* Tick init.
     * You have to call 'lv_tick_inc()' in periodically to inform LittelvGL about how much time were elapsed
     * Create an SDL thread to do this*/
    SDL_CreateThread(tick_thread, "tick", NULL);

    lv_timer_t * timer = lv_timer_create(data_gen,100,NULL);
}

void backend_loop(void)
{
    while(1) {
        SDL_Delay(5);
        lv_timer_handler();
    }
}
