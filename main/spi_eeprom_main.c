/* SPI Master Half Duplex EEPROM example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "spi_eeprom.h"

#define TIMER_DIVIDER   2
#define LED_PIN         GPIO_NUM_2
#define PIN_NUM_MOSI    GPIO_NUM_13
#define PIN_NUM_CLK     GPIO_NUM_14
#define PIN_NUM_CS      GPIO_NUM_27
#define PIN_NUM_LATCH   GPIO_NUM_25
#define PIN_NUM_FETH    GPIO_NUM_23
#define PIN_NUM_FETL    GPIO_NUM_22
spi_device_handle_t spi2;
esp_err_t ret;
// static const char TAG[] = "main";
const uint8_t transmitdata[3] = {0xaa,0xbb,0xcc};
uint8_t spiTx(const uint8_t *txData, uint16_t length);
typedef struct 
{
    uint8_t feth;
    uint8_t fetl;
}fet_state_t;

fet_state_t fetek = {
    .feth = 1,
    .fetl = 0
};

static bool IRAM_ATTR timer_group_isr_callback(void * args) {
    gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
    if(fetek.feth == 1)
    {
        fetek.feth = 0;
    }
    else
    {
        fetek.feth = 1;
    }
    if(fetek.fetl == 1)
    {
        fetek.fetl = 0;
    }
    else
    {
        fetek.fetl = 1;
    }
    if(fetek.feth == 1)
    {
        spiTx(transmitdata,3);
    }
    if(fetek.fetl == 1)
    {
        spiTx(transmitdata,3);
    }
        

    gpio_set_level(PIN_NUM_FETH,fetek.feth);
    gpio_set_level(PIN_NUM_FETL,fetek.fetl);
    return 0;
}
static void spiSelect(spi_transaction_t* t)
{

  gpio_set_level(PIN_NUM_CS,0);
}     

static void spiDeselect(spi_transaction_t* t)
{
   
   gpio_set_level(PIN_NUM_LATCH,0);
   gpio_set_level(PIN_NUM_LATCH,1);
   gpio_set_level(PIN_NUM_LATCH,0);
   gpio_set_level(PIN_NUM_CS,1);
}
void spiInitR()
{
    
    spi_bus_config_t buscfg={
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
    };
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 1000000,  // 1 MHz
        .mode = 1,                  //SPI mode 0
        .spics_io_num = -1,     
        .queue_size = 128,
		.flags = SPI_DEVICE_HALFDUPLEX,
        .pre_cb = spiSelect,
        .post_cb = spiDeselect,
        .input_delay_ns = 0
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi2));
    ESP_LOGI("spi", "spi init ok");
}


uint8_t spiTx(const uint8_t *txData, uint16_t length)
{  

  spi_transaction_t t = {
        .tx_buffer = txData,
        .length = 8*length,
        .flags = 0,
        .rxlength = 0

    };
    spi_device_polling_transmit(spi2, &t);
    return 0;
}
void app_main(void)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_NUM_LATCH, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_NUM_FETH, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_NUM_FETL, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    spiInitR();
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (5000));
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, NULL, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);
    ledc_timer_config_t ledtimerconf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER_3,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledtimerconf);
    ledc_channel_config_t ledchanconf = {
        .gpio_num =         26,
        .speed_mode =       LEDC_LOW_SPEED_MODE,
        .channel =          LEDC_CHANNEL_0,
        .intr_type =        LEDC_INTR_DISABLE,
        .timer_sel =        LEDC_TIMER_3,
        .duty =             4096,
        .hpoint =           0
    };
    ledc_channel_config(&ledchanconf);  
    
    
    while (1) {
        vTaskDelay(1);
    }
}