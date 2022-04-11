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


void but_callback(void);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void update_display(void);
void init(void);

// Global var
volatile send_signal_flag = 0;
volatile receive_signal_flag = 0;

char buffer [128];


// FUNCOES
void but_callback(void)
{
	if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		send_signal_flag = 1;
	}
}

double calc_distancia(int t){
	return (t*34000)/2;
}

//#####################################################
// INIT FUNCTIONS
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void TC1_Handler(void) {
	volatile uint32_t status = tc_get_status(TC0, 1);
	
	if(pio_get_output_data_status(LED_PIO, LED_IDX_MASK)){
		pio_clear(LED_PIO, LED_IDX_MASK);
	}
	else{
		pio_set(LED_PIO,LED_IDX_MASK);
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

	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4
}


int main (void)
{
	init();
	
	/* Insert application code here, after the board has been initialized. */
	while(1) {
		if (send_signal_flag){
			TC_init(TC0, ID_TC1, 1, 4);
			tc_start(TC0, 1);
			send_signal_flag = 0;
		}
		if (receive_signal_flag){
			double distancia = calc_distancia(2);
		}
		// Entra em sleep mode
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
