#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/i2s.h"
#include "esp_err.h"
#include "hal/i2s_types.h"

#define BCLK_PIN 25
#define LRC_PIN 26
#define DIN_PIN 22

static const adc1_channel_t adc_channel = ADC1_CHANNEL_6;

static const int i2s_num = I2S_NUM_0;
static uint8_t buff[128];

static short signal_mem[1];
static const float alpha = 0.97;

static esp_err_t init_i2s(){
    esp_err_t err;

    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = 16000,
        .bits_per_sample = 16,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = 1024,
        .use_apll = 1,
    };

    err = i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
    if(err != ESP_OK){
        return err;
    }

    i2s_pin_config_t pin_config = {
        .bck_io_num = BCLK_PIN,
        .ws_io_num = LRC_PIN,
        .data_out_num = DIN_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE,
    };
    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if(err != ESP_OK){
        return err;
    }

    return i2s_zero_dma_buffer(i2s_num);
}

static void init_adc(void){
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
}

u_int16_t preemphasis(uint16_t x){
    float x_em = 0;
    x_em = (short) x - alpha * signal_mem[0];
    signal_mem[0] = (short) x;
    return (uint16_t) x_em;
}

void app_main() {
    esp_err_t ret;
    const TickType_t xDelay = 0.0625 / portTICK_PERIOD_MS; // 16000 sample rates
    
    init_adc();
    ret = init_i2s();
    if(ret != ESP_OK){
        printf("err: %s", esp_err_to_name(ret));
        return;
    }
    
    signal_mem[0] = 0;
    int count = 0;
    size_t bytes_writen;
    while(1){
        uint16_t adc_val = 0; // for bit-depth 16
        adc_val = adc1_get_raw(adc_channel);

        // Pre-emphasis
        uint16_t signal = 0;
        signal = preemphasis(adc_val);

        signal = signal << 2; // Amplify 4 times

        memcpy(buff + count, &signal, 2);
        count += 2;
        vTaskDelay(xDelay);

        if(count == 128){
            count = 0;
            ret = i2s_write(i2s_num, (const void *)buff, sizeof(buff), &bytes_writen, portMAX_DELAY);
            if(ret != ESP_OK){
                printf("err: %s", esp_err_to_name(ret));
                break;
            }
        }
    }

    i2s_driver_uninstall(i2s_num);
}