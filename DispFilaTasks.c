#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include "blink.pio.h"
#include "hardware/pio.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27
#define LED_BLUE 12
#define LED_GREEN  11
#define tam_quad 10
#define BUZZER_PIN 21
#define NUM_PIXELS 25
#define MATRIXPIO 7
typedef struct
{
    uint16_t x_pos;
    uint16_t y_pos;
} joystick_data_t;

void beep_buzzer(int duration_ms, int freq) {
    uint slice_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint channel_buzzer = pwm_gpio_to_channel(BUZZER_PIN);
    
    // Configuração do PWM para o buzzer
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f); // Divisor de clock 1 (125MHz)
    
    // Calcula o wrap value baseado na frequência desejada
    uint32_t wrap = (125000000 / freq) - 1;
    if (wrap > 65535) wrap = 65535; // Limite máximo para 16-bit
    
    pwm_config_set_wrap(&config, wrap);
    pwm_init(slice_buzzer, &config, true);
    
    // Configura duty cycle para 50%
    pwm_set_chan_level(slice_buzzer, channel_buzzer, wrap / 2);
    
    // Aguarda a duração do beep
    sleep_ms(duration_ms);
    
    // Desliga o buzzer
    pwm_set_chan_level(slice_buzzer, channel_buzzer, 0);
    pwm_set_enabled(slice_buzzer, false);
}

double desenhoRED[25] = {1, 0, 0, 0, 1,
                        0, 1, 0, 1, 0,
                        0, 0, 1, 0, 0,
                        0, 1, 0, 1, 0,
                        1, 0, 0, 0, 1};

double desenhovazio[25] = {0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0};

uint32_t matrix_rgb(double b, double r, double g)
   {
     unsigned char R, G, B;
     R = r * 255 * 0.125;
     G = g * 255 * 0.125;
     B = b * 255 * 0.125;
     return (G << 24) | (R << 16) | (B << 8);
   }

   void desenho_pioRED(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){
   
    for (int16_t i = 0; i < NUM_PIXELS; i++) {
        if (i%2==0)
        {
            valor_led = matrix_rgb(0, desenho[24-i], 0.0);
            pio_sm_put_blocking(pio, sm, valor_led);

        }else{
            valor_led = matrix_rgb(0, desenho[24-i], 0.0);
            pio_sm_put_blocking(pio, sm, valor_led);
        }
    }
}

QueueHandle_t xQueueJoystickData;
QueueHandle_t xQueueJoystickDisplay;
QueueHandle_t xQueueJoystickMatrix;
QueueHandle_t xQueueJoystickGreen;
QueueHandle_t xQueueJoystickBlue;

void vJoystickTask(void *params)
{
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_init();

    joystick_data_t joydata;

    while (true)
    {
        adc_select_input(0); // GPIO 26 = ADC0
        joydata.y_pos = adc_read();

        adc_select_input(1); // GPIO 27 = ADC1
        joydata.x_pos = adc_read();

        xQueueSend(xQueueJoystickData, &joydata, 0); // Envia o valor do joystick para a fila
        xQueueSend(xQueueJoystickDisplay, &joydata, 0);
        xQueueSend(xQueueJoystickMatrix, &joydata, 0);
        xQueueSend(xQueueJoystickGreen, &joydata, 0);
        xQueueSend(xQueueJoystickBlue, &joydata, 0);
        vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura
    }
}

void vDisplayTask(void *params)
{
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    bool cor = true;
    joystick_data_t joydata;
    char buffer[32]; // Buffer para armazenar strings formatadas
    
    while (true)
    {
        if (xQueueReceive(xQueueJoystickDisplay, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Limpa a tela (preenche com preto)
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6); // Desenha uma string
            ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16);  // Desenha uma string
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            // Exibe os valores X e Y
            snprintf(buffer, sizeof(buffer), "X:%4d", joydata.x_pos);
            ssd1306_draw_string(&ssd, buffer, 10, 40);
            
            snprintf(buffer, sizeof(buffer), "Y:%4d", joydata.y_pos);
            ssd1306_draw_string(&ssd, buffer, 10, 50);  // 8 pixels abaixo

            // Atualiza o display
            ssd1306_send_data(&ssd);
    }
}
}

void vLedGreenTask(void *params)
{
    gpio_set_function(LED_GREEN, GPIO_FUNC_PWM);   // Configura GPIO como PWM
    uint slice = pwm_gpio_to_slice_num(LED_GREEN); // Obtém o slice de PWM
    pwm_set_wrap(slice, 100);                     // Define resolução (0–100)
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);     // Duty inicial
    pwm_set_enabled(slice, true);                 // Ativa PWM

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickGreen, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Brilho proporcional ao desvio do centro
            int16_t desvio_centro = (int16_t)joydata.x_pos - 2000;
            if (desvio_centro < 0)
                desvio_centro = -desvio_centro;
            uint16_t pwm_value = (desvio_centro * 100) / 2048;
            pwm_set_chan_level(slice, PWM_CHAN_B, pwm_value);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

void vLedBlueTask(void *params)
{
    gpio_set_function(LED_BLUE, GPIO_FUNC_PWM);   // Configura GPIO como PWM
    uint slice = pwm_gpio_to_slice_num(LED_BLUE); // Obtém o slice de PWM
    pwm_set_wrap(slice, 100);                     // Define resolução (0–100)
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);     // Duty inicial
    pwm_set_enabled(slice, true);                 // Ativa PWM

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickBlue, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Brilho proporcional ao desvio do centro
            int16_t desvio_centro = (int16_t)joydata.y_pos - 2048;
            if (desvio_centro < 0)
                desvio_centro = -desvio_centro;
            uint16_t pwm_value = (desvio_centro * 100) / 2048;
            pwm_set_chan_level(slice, PWM_CHAN_A, pwm_value);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

void MatrizTask(void *params) {
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    uint sm = pio_claim_unused_sm(pio, true);
    uint32_t valor_led;
    double r = 0.0, b = 0.0, g = 0.0;
    joystick_data_t joydata;

    blink_program_init(pio, sm, offset, MATRIXPIO);

    // Inicializa o pino do buzzer como saída
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);

    while(true) {
        if (xQueueReceive(xQueueJoystickMatrix, &joydata, pdMS_TO_TICKS(100))) {
            if (joydata.x_pos > 3500 || joydata.y_pos > 3500) {
                // Mostra o desenhoRED na matriz
                vTaskDelay(pdMS_TO_TICKS(1));
                desenho_pioRED(desenhoRED, valor_led, pio, sm, r, g, b);

                // Faz o buzzer apitar por 100ms a 1000Hz
                beep_buzzer(100, 1000);
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
                desenho_pioRED(desenhovazio, valor_led, pio, sm, r, g, b);
            }
        }
    }
}

// Modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    // Ativa BOOTSEL via botão
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();

    // Cria a fila para compartilhamento de valor do joystick
    xQueueJoystickData = xQueueCreate(5, sizeof(joystick_data_t));
    xQueueJoystickDisplay = xQueueCreate(5, sizeof(joystick_data_t));
    xQueueJoystickMatrix  = xQueueCreate(5, sizeof(joystick_data_t));
    xQueueJoystickGreen   = xQueueCreate(5, sizeof(joystick_data_t));
    xQueueJoystickBlue    = xQueueCreate(5, sizeof(joystick_data_t));

    // Criação das tasks
    xTaskCreate(vJoystickTask,  "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask,  "Display Task",  512, NULL, 1, NULL);
    xTaskCreate(vLedGreenTask, "LED red Task",  256, NULL, 1, NULL);
    xTaskCreate(vLedBlueTask,  "LED blue Task", 256, NULL, 1, NULL);
    xTaskCreate(MatrizTask,    "Matrix Task",   256, NULL, 1, NULL);
    // Inicia o agendador
    vTaskStartScheduler();
    panic_unsupported();
}
