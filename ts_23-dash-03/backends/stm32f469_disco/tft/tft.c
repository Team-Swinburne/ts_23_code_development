/********************************************************************************
* @file    tft.c
* @author  Nam Tran
* @version 2.0
* @date    19/04/2022
* @brief   tft driver to interface between lvgl and mbed stm bsp drivers
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
#include "tft.h"
#include "lvgl.h"
#include "stm32f4xx.h"
#include "bsp_drivers/stm32469i_discovery.h"
#include "bsp_drivers/stm32469i_discovery_lcd.h"
#include "bsp_drivers/stm32469i_discovery_sdram.h"
/* -------------------------------------------------------------------------- */
/*                                   DEFINES                                  */
/* -------------------------------------------------------------------------- */
#define DMA_STREAM               DMA2_Stream0
#define DMA_CHANNEL              DMA_CHANNEL_0
#define DMA_STREAM_IRQ           DMA2_Stream0_IRQn
#define DMA_STREAM_IRQHANDLER    DMA2_Stream0_IRQHandler

/* -------------------------------------------------------------------------- */
/*                              STATIC VARIABLES                              */
/* -------------------------------------------------------------------------- */
static __IO uint32_t * frame_buffer = (__IO uint32_t*) (SDRAM_DEVICE_ADDR);

static DMA_HandleTypeDef 	DmaHandle;
static lv_disp_drv_t     	disp_drv; 
static int32_t				x1_flush;
static int32_t 				y1_flush;
static int32_t 				x2_flush;
static int32_t    			y2_fill;
static int32_t 				y_fill_act;
static const lv_color_t *	buf_to_flush;
/* -------------------------------------------------------------------------- */
/*                                  PROTOTYPE                                 */
/* -------------------------------------------------------------------------- */
static void tft_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

/*DMA to flush to frame buffer*/
static void DMA_Config(void);
static void DMA_TransferComplete(DMA_HandleTypeDef *han);
static void DMA_TransferError(DMA_HandleTypeDef *han);

/* -------------------------------------------------------------------------- */
/*                                  FUNCTIONS                                 */
/* -------------------------------------------------------------------------- */
void tft_init() 
{
    BSP_LCD_Init();
	BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE);
	BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_BACKGROUND, LCD_FB_START_ADDRESS);
  	BSP_LCD_Clear(LCD_COLOR_BLACK);

    /*Initialize DMA*/
    DMA_Config();

    /*A static or global variable to store the buffers*/
    static lv_disp_draw_buf_t disp_buf;

    /*Static or global buffer(s)*/
    static lv_color_t buf[TFT_HOR_RES * 10];
    
    /*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */
    lv_disp_draw_buf_init(&disp_buf, buf, NULL, TFT_HOR_RES*10);

    /*Basic initialization*/
    lv_disp_drv_init(&disp_drv);            
    disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
    disp_drv.flush_cb = tft_flush;        	/*Set a flush callback to draw to the display*/
    disp_drv.hor_res = TFT_HOR_RES;   	/*Set the horizontal resolution in pixels*/
    disp_drv.ver_res = TFT_VER_RES;    	/*Set the vertical resolution in pixels*/
    disp_drv.sw_rotate = 1;
    lv_disp_drv_register(&disp_drv);

    //lv_disp_set_rotation(NULL,LV_DISP_ROT_180);

}

/* -------------------------------------------------------------------------- */
/*                               STATIC FUNCTION                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Flush the content of the internal buffer to the display
 */
static void tft_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    int32_t x1 = area->x1;
    int32_t x2 = area->x2;
    int32_t y1 = area->y1;
    int32_t y2 = area->y2;
    /*Return if the area is out the screen*/
 
    if(x2 < 0) return;
    if(y2 < 0) return;
    if(x1 > TFT_HOR_RES - 1) return;
    if(y1 > TFT_VER_RES - 1) return;
 
    /*Truncate the area to the screen*/
    int32_t act_x1 = x1 < 0 ? 0 : x1;
    int32_t act_y1 = y1 < 0 ? 0 : y1;
    int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
    int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;
 
    x1_flush = act_x1;
    y1_flush = act_y1;
    x2_flush = act_x2;
    y2_fill = act_y2;
    y_fill_act = act_y1;
    buf_to_flush = color_p;

    HAL_StatusTypeDef err;
    uint32_t length = (x2_flush - x1_flush + 1)*2;
    err = HAL_DMA_Start_IT(&DmaHandle,
                            (uint32_t)buf_to_flush, 
                            (uint32_t)&frame_buffer[y_fill_act * TFT_HOR_RES + x1_flush],
                            length);
    if(err != HAL_OK)
    {
        while(1);   /*Halt on error*/
    }
}

static void DMA_Config(void)
{
    /*## -1- Enable DMA2 clock #################################################*/
    __HAL_RCC_DMA2_CLK_ENABLE();
 
    /*##-2- Select the DMA functional Parameters ###############################*/
    DmaHandle.Init.Channel = DMA_CHANNEL;                   		/* DMA_CHANNEL_0                    */
    DmaHandle.Init.Direction = DMA_MEMORY_TO_MEMORY;                /* M2M transfer mode                */
    DmaHandle.Init.PeriphInc = DMA_PINC_ENABLE;                     /* Peripheral increment mode Enable */
    DmaHandle.Init.MemInc = DMA_MINC_ENABLE;                        /* Memory increment mode Enable     */
    DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   /* Peripheral data alignment : 16bit */
    DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;      /* memory data alignment : 16bit     */
    DmaHandle.Init.Mode = DMA_NORMAL;                               /* Normal DMA mode                  */
    DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;                    /* priority level : high            */
    DmaHandle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;                  /* FIFO mode enabled                */
    DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL; /* FIFO threshold: 1/4 full   */
    DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;                    /* Memory burst                     */
    DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;                 /* Peripheral burst                 */
 
    /*##-3- Select the DMA instance to be used for the transfer : DMA2_Stream0 #*/
    DmaHandle.Instance = DMA_STREAM;
 
    /*##-4- Initialize the DMA stream ##########################################*/
    if(HAL_DMA_Init(&DmaHandle) != HAL_OK)
    {
        while(1)
        {
        }
    }
 
    /*##-5- Select Callbacks functions called after Transfer complete and Transfer error */
    HAL_DMA_RegisterCallback(&DmaHandle, HAL_DMA_XFER_CPLT_CB_ID, DMA_TransferComplete);
    HAL_DMA_RegisterCallback(&DmaHandle, HAL_DMA_XFER_ERROR_CB_ID, DMA_TransferError);
 
    /*##-6- Configure NVIC for DMA transfer complete/error interrupts ##########*/
    HAL_NVIC_SetPriority(DMA_STREAM_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(DMA_STREAM_IRQ);
}
 
/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void DMA_TransferComplete(DMA_HandleTypeDef *han)
{
    y_fill_act ++;
 
    if(y_fill_act > y2_fill) {
        //SCB_CleanInvalidateDCache();
        //SCB_InvalidateICache();
        lv_disp_flush_ready(&disp_drv);
    } else {
        uint32_t length = (x2_flush - x1_flush + 1);
        buf_to_flush += x2_flush - x1_flush + 1;
        /*##-7- Start the DMA transfer using the interrupt mode ####################*/
        /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
        /* Enable All the DMA interrupts */
#if LV_COLOR_DEPTH == 24 || LV_COLOR_DEPTH == 32
        length *= 2; /* STM32 DMA uses 16-bit chunks so multiply by 2 for 32-bit color */
#endif
        if(HAL_DMA_Start_IT(han,(uint32_t)buf_to_flush, (uint32_t)&frame_buffer[y_fill_act * TFT_HOR_RES + x1_flush],
                            length) != HAL_OK)
        {
            while(1);   /*Halt on error*/
        }
    }
}
 
/**
  * @brief  DMA conversion error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
static void DMA_TransferError(DMA_HandleTypeDef *han)
{
 
}
 
/**
  * @brief  This function handles DMA Stream interrupt request.
  * @param  None
  * @retval None
  */
void DMA_STREAM_IRQHANDLER(void)
{
    /* Check the interrupt and clear flag */
    HAL_DMA_IRQHandler(&DmaHandle);
}