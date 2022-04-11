#include <asf.h>
#include <stdio.h>

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

#define TGR_PIO  PIOD
#define TGR_ID   ID_PIOD
#define TGR_PIO_IDX  26
#define TGR_PIO_IDX_MASK (1u << TGR_PIO_IDX)

#define ECHO_PIO  PIOD
#define ECHO_ID   ID_PIOD
#define ECHO_PIO_IDX  11
#define ECHO_PIO_IDX_MASK (1u << ECHO_PIO_IDX)

void but_callback(void);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void update_display(void);
void init(void);

// Global var
int y1, y2, y3, y4;
volatile send_signal_flag = 0;
volatile receive_signal_flag = 0;
int freq = 1/(0.000058*2);
double tempo = 0;
volatile echo_error = 0;
volatile flag_echo = 0;
volatile draw_grafico_indice = 0;

char buffer [128];
int valores[5] = {0,0,0,0,0};
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);


// FUNCOES
void but_callback(void)
{
	if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		send_signal_flag = 1;
	}
}

void echo_callback(void){
	if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK)){
		RTT_init(freq, 0, 0);
	}else{
		tempo = rtt_read_timer_value(RTT);
		int pulses = (int)(freq*0.011764)*2;
		if (tempo > pulses){
			echo_error = 1;
		}
		receive_signal_flag = 1;
	}
}

void trigger(void){
	pio_set(TGR_PIO, TGR_PIO_IDX_MASK);
	delay_us(10);	
	pio_clear(TGR_PIO, TGR_PIO_IDX_MASK);
}

void update_display(void){
	gfx_mono_generic_draw_filled_rect(0,0,128,32,0);
	gfx_mono_draw_string(buffer, 8,8, &sysfont);
}

int calc_distancia(int t){
	return (100.0*(t*0.000058)*340)/2;
}

//#####################################################
// INIT FUNCTIONS
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {
	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		//modifica toda vez que o alarme for acionado
		//echo_error = 1;
	}
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		// modifica a cada iteração
	}
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
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(TGR_ID);
	pmc_enable_periph_clk(ECHO_ID);
	
	//Inicializa PC8 como saída
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(TGR_PIO, TGR_PIO_IDX_MASK, 0, 0, 0);
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK, PIO_DEFAULT);
	
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX_MASK,
	PIO_IT_EDGE,
	but_callback);
	
	pio_handler_set(ECHO_PIO,
	ECHO_ID,
	ECHO_PIO_IDX_MASK,
	PIO_IT_EDGE,
	echo_callback);

	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_IDX_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(ECHO_ID);
	NVIC_SetPriority(ECHO_ID, 3);
}


int main (void)
{
	init();
	
	/* Insert application code here, after the board has been initialized. */
	while(1) {
		if (send_signal_flag==1){
			trigger();
			send_signal_flag = 0;
		}
		if (receive_signal_flag){
			int distancia = calc_distancia(tempo);
			if (!echo_error){
				sprintf(buffer, "%03d", distancia);
				valores[draw_grafico_indice] = distancia;
			}else{
				sprintf(buffer, "Erro");
				valores[draw_grafico_indice] = 400;
				echo_error = 0;
			}
			if (draw_grafico_indice >= 4){
				valores[0] = valores[1];
				valores[1] = valores[2];
				valores[2] = valores[3];
				valores[3] = valores[4];
			}
			else{
				draw_grafico_indice += 1;
			}
			update_display();
			
			y1 = 32-(((float)valores[0]/400)*32);
			gfx_mono_generic_draw_filled_rect(64, y1, 15, 32, 1);
			y2 = 32-(((float)valores[1]/400)*32);
			gfx_mono_generic_draw_filled_rect(64+16, y2, 15, 32, 1);
			y3 = 32-(((float)valores[2]/400)*32);
			gfx_mono_generic_draw_filled_rect(64+(2*16), y3, 15, 32, 1);
			y4 = 32-(((float)valores[3]/400)*32);
			gfx_mono_generic_draw_filled_rect(64+(3*16), y4, 7, 32, 1);
			
			receive_signal_flag = 0;
			//delay_ms(100);
		}
	}
}
