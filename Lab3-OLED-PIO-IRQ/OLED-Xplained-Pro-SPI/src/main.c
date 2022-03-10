#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// DEFINES
#define LED2_PIO           PIOC
#define LED2_PIO_ID        ID_PIOC
#define LED2_PIO_IDX       30
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)

#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

#define BUT1_PIO  PIOD
#define BUT1_PIO_ID   ID_PIOD
#define BUT1_PIO_IDX  28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

#define BUT2_PIO  PIOC
#define BUT2_PIO_ID   ID_PIOC
#define BUT2_PIO_IDX  31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO  PIOA
#define BUT3_PIO_ID   ID_PIOA
#define BUT3_PIO_IDX  19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)


void pisca_led(int n, int t);
void but_callback(void);
void but2_callback(void);
void but3_callback(void);
void init(void);
void update_display(void);

// Global var
volatile but_flag = 0;
volatile stop_blink_flag = 0;
int timer = 0;
int frequencia = 1;
char buffer [128];


// FUNCOES	
void pisca_led(int n, int freq){
	double periodo = 1.0/freq;
	double progress = 0;
	double bar = 0;
	int time_delay = periodo*1000;//não faz sentido quanto tempo ele demora
	gfx_mono_generic_draw_rect(60, 8, 60, 16, 1);
	for (int i=0;i<=30;i++){
		if (stop_blink_flag){
			stop_blink_flag = 0;
			break;
		}
		progress = ((double)i/(30))*100;
		bar = (double)(60.0/100)*progress;
		gfx_mono_generic_draw_filled_rect(60, 8, bar, 16, 1);
		pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
		delay_ms(time_delay);
		pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
		delay_ms(time_delay);
	}
}

void but_callback(void)
{
	for (int i=0; i<3500000; i++){
		if (!pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
			timer = i;
		} else if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK) && timer>3000000){
			frequencia -= 1;
			timer = 0;
			but_flag = 1;
			break;
		} else if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK) && timer<3000000 && timer!=0){
			frequencia += 1;
			timer = 0;
			but_flag = 1;
			break;
		} else if (pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK) && timer>3000000){
			pio_clear(LED_PIO, LED_IDX_MASK);
			stop_blink_flag = 1;
			break;
		}
	}
	update_display();
}
void but2_callback(void)
{
	for (int i=0; i<3500000; i++){
		if (!pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)) {
			timer = i;
		} else if (pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK) && timer>3000000){
			stop_blink_flag = 1;
			timer = 0;
			break;
		}
	}
}
void but3_callback(void)
{
	for (int i=0; i<3500000; i++){
		if (!pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)) {
			timer = i;
		} else if (pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK) && timer<3000000 && timer!=0){
			frequencia -= 1;
			timer = 0;
			break;
		}
	}
}

void update_display(void){
	sprintf(buffer, "%dHz", frequencia);
	gfx_mono_draw_string(buffer, 8,8, &sysfont);
	gfx_mono_generic_draw_filled_rect(60,8,60,16,0);
}

void init(void){
	// Initialize the board clock
	sysclk_init();
	
	board_init();
	sysclk_init();
	delay_init();
	
	// Init OLED
	gfx_mono_ssd1306_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	
	//Inicializa PC8 como saída
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
	
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX_MASK,
	PIO_IT_EDGE,
	but_callback);
	
	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_PIO_IDX_MASK,
	PIO_IT_EDGE,
	but2_callback);
	
	pio_handler_set(BUT3_PIO,
	BUT3_PIO_ID,
	BUT3_PIO_IDX_MASK,
	PIO_IT_EDGE,
	but3_callback);
	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);
	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4
}


int main (void)
{
	init();	
	
  /* Insert application code here, after the board has been initialized. */
	while(1) {
		update_display();
		if (but_flag){
			pisca_led(4, frequencia);
			but_flag = 0;
		}
		// Entra em sleep mode
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
