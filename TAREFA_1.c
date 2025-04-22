#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"     
#include "hardware/pwm.h"  
#include "pico/bootrom.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "pio_matrix.pio.h"
#include "hardware/timer.h"

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define VRX 27
#define VRY 26  
#define BUZZER_A 21
#define BUZZER_B 10
#define LED_VERMELHO 13
#define LED_VERDE 11
#define LED_AZUL 12
#define BOTAO_A 5
#define BOTAO_B 6

// Variáveis globais
uint idx = 13;
bool sinal = true;
static volatile uint32_t last_time = 0;
bool primeira_vez = true;
bool ativar_contagem;
bool frase_a, frase_b;
bool pausar_timer;
bool vermelho, verde;
bool buzzp, buzzn;

// Display
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
ssd1306_t ssd;

// Matriz de led's
PIO pio;
uint sm;
uint32_t VALOR_LED;
unsigned char R, G, B;
#define NUM_PIXELS 25
#define OUT_PIN 7

void buzzer_positivo();

double X[] = {
    0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0, 
    0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 
    0.1, 0.0, 0.0, 0.0, 0.1
};

double V[] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 
    0.0, 0.1, 0.0, 0.0, 0.0
};

double apaga[] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0
};

// Números de 0 a 9
double matrix[10][25] = {
    {0.0, 0.2, 0.2, 0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.2, 0.2, 0.2, 0.0},
    {0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0},
    {0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0},
    {0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0},
    {0.0, 0.2, 0.0, 0.2, 0.0, 0.0, 0.2, 0.0, 0.2, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0},
    {0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0},
    {0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.2, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0},
    {0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0},
    {0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.2, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.2, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0},
    {0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.2, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0}
};


uint32_t matrix_rgb(double b, double r, double g)
{
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

// Desenha na matriz de leds em verde
void desenho_pio_green(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){

    for (int16_t i = 0; i < NUM_PIXELS; i++) {
            valor_led = matrix_rgb(b = 0.0, r=0.0, desenho[24-i]);
            pio_sm_put_blocking(pio, sm, valor_led);
    }
}

// Desenha na matriz de leds em vermelho
void desenho_pio_red(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){

    for (int16_t i = 0; i < NUM_PIXELS; i++) {
            valor_led = matrix_rgb(b = 0.0, desenho[24-i], g = 0.0);
            pio_sm_put_blocking(pio, sm, valor_led);
    }
}

// Ativação do buzzer
void buzz(uint8_t BUZZER_PIN, uint16_t freq, uint16_t duration) {
    int period = 1000000 / freq;
    int pulse = period / 2;
    int cycles = freq * duration / 1000;
    for (int j = 0; j < cycles; j++) {
        gpio_put(BUZZER_PIN, 1);
        sleep_us(pulse);
        gpio_put(BUZZER_PIN, 0);
        sleep_us(pulse);
    }
}

void buzz_for_duration(uint8_t BUZZER_PIN, uint16_t freq, uint16_t duration, uint16_t total_time_ms) {
    uint16_t elapsed_time = 0;

    while (elapsed_time < total_time_ms) {
        buzz(BUZZER_PIN, freq, duration);
        elapsed_time += duration;
        sleep_ms(1); // Espera 1ms entre cada chamada de buzz
    }
}

// Inicialização do Display SSD1306
void init_ssd1306() {
   
    i2c_init(I2C_PORT, 400 * 1000);

    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);


    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

// Iniacializador de entradas
void entradas(uint pino) {
    gpio_init(pino);

    gpio_set_dir(pino, GPIO_IN);

    gpio_pull_up(pino);
}

// Iniacializador de saídas
void saidas (uint pino) {
    gpio_init(pino);

    gpio_set_dir(pino,GPIO_OUT);
}

void contagem_regressiva () {
    int i;

    for(i = 5; i > 0; i--) {
        desenho_pio_red(matrix[i], VALOR_LED, pio, sm, R, G, B);
        sleep_ms(1000);
    }

    if(vermelho){
        gpio_put(LED_VERDE, false);
        gpio_put(LED_AZUL, false);
        gpio_put(LED_VERMELHO, true); // passagem do pedestre com necessidade
        desenho_pio_red(X, VALOR_LED, pio, sm, R, G, B);
        //sleep_ms(5000);
        vermelho = false; 
    } else if(verde) {
        desenho_pio_green(V, VALOR_LED, pio, sm, R, G, B);
        buzzer_positivo();
        pausar_timer = false;
        verde = true;
    }

}


bool repeating_timer_callback(struct repeating_timer *t) {
    
    if (pausar_timer) return true;
    
    switch (idx) {
        case 11: //verde
            gpio_put(LED_VERMELHO,false);
            gpio_put(LED_VERDE,true);
            desenho_pio_green(V, VALOR_LED, pio, sm, R, G, B);
            buzzp = true;
            idx = 12;
            break;
        case 12: //amarelo
            gpio_put(LED_VERMELHO,false);
            gpio_put(LED_VERMELHO,true);
            gpio_put(LED_VERDE,true);
            sinal = !sinal;
            (sinal == false) ? (idx = 11) : (idx = 13);
            break;
        case 13: //vermelho
            gpio_put(LED_VERMELHO,false);
            gpio_put(LED_VERDE,false);
            gpio_put(LED_VERMELHO,true);
            desenho_pio_red(X, VALOR_LED, pio, sm, R, G, B);

            buzzn = true;
            idx = 12;
            break;
    }
    return true;
}

static void gpio_irq_handler(uint gpio, uint32_t events) {
    
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    if (current_time - last_time > 600000){
        last_time = current_time; 
            if(gpio == BOTAO_A) {
                pausar_timer = true;
                frase_a = true;
                vermelho = true;
                ativar_contagem = true;
            } else if(gpio == BOTAO_B) {
                pausar_timer = true;
                frase_b = true;
                ativar_contagem = true;
                verde = true;
            }
    }
}

void buzzer_positivo() {
    buzz_for_duration(BUZZER_A, 1000, 100, 100);
    buzz_for_duration(BUZZER_A, 1500, 100, 100);
    buzz_for_duration(BUZZER_A, 2000, 100, 100);
}

void buzzer_negativo() {
    buzz_for_duration(BUZZER_A, 1500, 300, 2500);
}

int main()
{
    stdio_init_all();


    uart_init(UART_ID, BAUD_RATE);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Inicializando ADC
    adc_init();
    adc_gpio_init(VRX);

    // Inicializando ADC
    adc_init();
    adc_gpio_init(VRY);

    // Inicializando buzzer
    saidas(BUZZER_A);

    // Setando matriz de led's
    double r = 0.0, b = 0.0 , g = 0.0;
    bool ok;
    ok = set_sys_clock_khz(128000, false);
    pio = pio0;

    uint offset = pio_add_program(pio, &TAREFA_1_program);
    uint sm = pio_claim_unused_sm(pio, true);
    TAREFA_1_program_init(pio, sm, offset, OUT_PIN);

    uint32_t last_display_update = 0;
    uint32_t update_interval = 5000;

    // Inicializando botões
    entradas(BOTAO_A);
    entradas(BOTAO_B);

    // Inicializando leds
    saidas(LED_AZUL);
    saidas(LED_VERDE);
    saidas(LED_VERMELHO);

    // Inicializando display 
    init_ssd1306();

    // Setando o semáforo pra começar no vermelho
    gpio_put(LED_VERMELHO,true);
    desenho_pio_red(X, VALOR_LED, pio, sm, R, G, B);
    idx = 12;

    // Chamada pra callback com timer
    struct repeating_timer timer;
    add_repeating_timer_ms(5000, repeating_timer_callback, NULL, &timer);

    // Habilitando interrupções
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    while(true) {


        if(ativar_contagem) {
            contagem_regressiva();
                if(frase_a) {
                    puts("Passagem concedida\n");
                    sleep_ms(5000);
                    frase_a = false;
                } else if(frase_b) {
                    puts("Passagem finalizada com sucesso!\n");
                    frase_b = false;
                }
            ativar_contagem = false;
        }

        if(buzzp) {
            buzzer_positivo();
            buzzp = false;
        } else if(buzzn) {
            buzzer_negativo();
            buzzn = false;
        }
        
        uint32_t current_time = to_us_since_boot(get_absolute_time());
        
        adc_select_input(1);
        uint16_t vrx_value = adc_read();

        adc_select_input(0);
        uint16_t vry_value = adc_read();
        
        
        uint8_t conv_x = (128 * vrx_value) / 4096;
        uint8_t conv_y = (64 * vry_value) / 4096;
            
            if(conv_y > 31) { // Como o eixo y é invertido no display, usei esse fator de conversão 
                conv_y = conv_y - 31;
                conv_y = 31 - conv_y;
            } else { 
                conv_y = (conv_y - 31) * -1;
                conv_y = 31 + conv_y;
            }

        if (current_time - last_display_update >= update_interval) {
            last_display_update = current_time;
                if(vrx_value <= 2048 && vrx_value >= 1875 && vry_value <= 2048 && vry_value >= 1950) { // caso o joystick esteja centralizado, o quadrado permanece centralizado no display 
                    ssd1306_fill(&ssd, false);
                    ssd1306_draw_square(&ssd, 64, 32);
                    ssd1306_send_data(&ssd);
                }else{
                    ssd1306_fill(&ssd, false);
                    ssd1306_draw_square(&ssd, conv_x, conv_y);
                    ssd1306_send_data(&ssd);
                }
        } else {
            ssd1306_fill(&ssd, false);
            ssd1306_send_data(&ssd);
        }

    }
}
