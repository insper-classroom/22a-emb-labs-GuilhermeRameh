/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"

#include "clockSymbol.h"

/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

#define LV_HOR_RES_MAX          (320)
#define LV_VER_RES_MAX          (240)

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;

// font declare
LV_FONT_DECLARE(dseg70);
LV_FONT_DECLARE(dseg40);
LV_FONT_DECLARE(dseg24);

// global
static  lv_obj_t * labelBtn1;
static  lv_obj_t * labelBtnM;
static  lv_obj_t * labelBtnT;

static  lv_obj_t * labelBtnUp;
static  lv_obj_t * labelBtnDown;

lv_obj_t * labelFloor;
lv_obj_t * labelFloorDecimal;
lv_obj_t * labelSetValue;
lv_obj_t * labelClock;

uint32_t current_hour, current_min, current_sec;
uint32_t current_year, current_month, current_day, current_week;

SemaphoreHandle_t xSemaphoreSeconds;
volatile char settings_flag = 0;

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_LCD_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/*              RTC                                                     */
/************************************************************************/

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}


void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	
	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		xSemaphoreGiveFromISR(xSemaphoreSeconds, pdFALSE);
		// o código para irq de segundo vem aqui
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		// o código para irq de alame vem aqui

	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/

static void event_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_CLICKED) {
		lv_obj_clean(lv_scr_act());
	}
}

static void menu_handler(lv_event_t * e) {
	
}

static void clk_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_CLICKED) {
		if (settings_flag)
			settings_flag = 0;
		else
			settings_flag = 1;
	}
}

static void up_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	char *c;
	int temp;
	char *hour_c;
	char *minute_c;
	int end;
	if(code == LV_EVENT_CLICKED) {
		if (settings_flag){
			c = lv_label_get_text(labelClock);
			for(int i = 0; i<6; i++){
				if(c[i] != ':'){
					if (i<=1)
						hour_c[i] = c[i];
					else
						minute_c[i-end] = c[i];
				}
				else
					end = i;
			}
			printf("%c", hour_c);
			printf("%c", minute_c);
		}
		else {
			c = lv_label_get_text(labelSetValue);
			temp = atoi(c);
			lv_label_set_text_fmt(labelSetValue, "%02d", temp + 1);
		}
	}
}

static void down_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	char *c;
	int temp;
	if(code == LV_EVENT_CLICKED) {
		c = lv_label_get_text(labelSetValue);
		temp = atoi(c);
		lv_label_set_text_fmt(labelSetValue, "%02d", temp - 1);
	}
}


void lv_termostato(void){
	static lv_style_t style;
	lv_style_init(&style);
	lv_style_set_bg_color(&style, lv_color_black());
	//lv_style_set_border_color(&style, lv_color_white());
	lv_style_set_border_width(&style, 5);
	
	// Btn POWER
	lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btn1, LV_ALIGN_BOTTOM_LEFT, 0, 0);
	lv_obj_add_style(btn1, &style, 0);
	lv_obj_set_width(btn1, 60);  lv_obj_set_height(btn1, 60);

	labelBtn1 = lv_label_create(btn1);
	lv_label_set_text(labelBtn1, "[ " LV_SYMBOL_POWER);
	lv_obj_center(labelBtn1);
	
	// Btn M
	lv_obj_t * btnM = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btnM, menu_handler, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btnM, btn1, LV_ALIGN_RIGHT_MID, 40, -22);
	lv_obj_add_style(btnM, &style, 0);
	lv_obj_set_width(btnM, 60);  lv_obj_set_height(btnM, 60);

	labelBtnM = lv_label_create(btnM);
	lv_label_set_text(labelBtnM, "| M |");
	lv_obj_center(labelBtnM);

	//labelBtnT = lv_label_create(btnClk);
	//lv_label_set_text(labelBtnT, );
	//lv_obj_center(labelBtnT);
	
	// Btn donw
	lv_obj_t * btndown = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btndown, down_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btndown, LV_ALIGN_BOTTOM_RIGHT, -20, 0);
	lv_obj_add_style(btndown, &style, 0);
	lv_obj_set_width(btndown, 60);  lv_obj_set_height(btndown, 60);

	labelBtnDown = lv_label_create(btndown);
	lv_label_set_text(labelBtnDown, "| " LV_SYMBOL_DOWN " ]");
	lv_obj_center(labelBtnDown);
	
	// Btn up
	lv_obj_t * btnup = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btnup, up_handler, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btnup, btndown, LV_ALIGN_LEFT_MID, -80, -22);
	lv_obj_add_style(btnup, &style, 0);
	lv_obj_set_width(btnup, 60);  lv_obj_set_height(btnup, 60);

	labelBtnUp = lv_label_create(btnup);
	lv_label_set_text(labelBtnUp, "[" LV_SYMBOL_UP);
	lv_obj_center(labelBtnUp);
	
	// ------------------ Labels not Btn ----------------------------
	// label floor
	labelFloor = lv_label_create(lv_scr_act());
	lv_obj_align(labelFloor, LV_ALIGN_LEFT_MID, 35 , -45);
	lv_obj_set_style_text_font(labelFloor, &dseg70, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelFloor, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelFloor, "%02d", 23);
	
	labelFloorDecimal = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelFloorDecimal, labelFloor, LV_ALIGN_RIGHT_MID, 60, 0);
	lv_obj_set_style_text_font(labelFloorDecimal, &dseg40, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelFloorDecimal, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelFloorDecimal, ".%d", 4);
	
	// label set value
	labelSetValue = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelSetValue, labelFloor, LV_ALIGN_RIGHT_MID, 130 , -10);
	lv_obj_set_style_text_font(labelSetValue, &dseg40, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelSetValue, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelSetValue, "%02d", 22);
	
	// label clock
	labelClock = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelClock, labelFloor, LV_ALIGN_RIGHT_MID, 110 , -40);
	lv_obj_set_style_text_font(labelClock, &dseg24, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelClock, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelClock, "%02d:%02d", 17, 47);
	
	
	// Btn Clock
	lv_obj_t * btnClk = lv_imgbtn_create(lv_scr_act());
	lv_imgbtn_set_src(btnClk, LV_IMGBTN_STATE_RELEASED, NULL, &clockSymbol, NULL);
	lv_obj_add_event_cb(btnClk, clk_handler, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btnClk, btnM, LV_ALIGN_RIGHT_MID, 158, 2);
	lv_obj_add_style(btnClk, &style, 0);
	lv_obj_set_width(btnClk, 30);  lv_obj_set_height(btnClk, 30);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_lcd(void *pvParameters) {
	int px, py;
	
	lv_termostato();

	for (;;)  {
		lv_tick_inc(50);
		lv_task_handler();
		vTaskDelay(50);
	}
}

static void task_rtc(){
	
	calendar rtc_initial = {2022, 3, 16, 11, 11, 45 , 0};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_SR_ALARM | RTC_SR_SEC);
	
	for (;;) {
		if (xSemaphoreTake(xSemaphoreSeconds, 100)){
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			if (current_sec%2==0)
				lv_label_set_text_fmt(labelClock, "%02d:%02d", current_hour, current_min);
			else
				lv_label_set_text_fmt(labelClock, "%02d %02d", current_hour, current_min);
		}
	}
	
}

/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void configure_lcd(void) {
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	
	ili9341_init();
	ili9341_backlight_on();
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	
	/* IMPORTANT!!!
	* Inform the graphics library that you are ready with the flushing*/
	lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	
	if (readPoint(&px, &py))
		data->state = LV_INDEV_STATE_PRESSED;
	else
		data->state = LV_INDEV_STATE_RELEASED; 
	
	data->point.x = px;
	data->point.y = py;
}

void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	sysclk_init();
	configure_console();

	/* LCd, touch and lvgl init*/
	configure_lcd();
	configure_touch();
	configure_lvgl();
	
	xSemaphoreSeconds = xSemaphoreCreateBinary();
	if (xSemaphoreSeconds == NULL)
		printf("Falha ao criar Semáforo \n");

	/* Create task to control oled */
	if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}
	
	if (xTaskCreate(task_rtc, "rtc", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create rtc task\r\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){ }
}
