#include <stdio.h>
#include <stdlib.h>

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2S_NUM I2S_NUM_0

#define I2S_BLCK 2
#define I2S_WS 3
#define I2S_DOUT 4
#define I2S_DIN 5

#define BUFF_SIZE 128

static i2s_chan_handle_t tx_chan;
static i2s_chan_handle_t rx_chan;

static esp_err_t init_i2s_mic(){
    esp_err_t ret;

    // Initial i2s in duplex mode
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ret = i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan);
    if(ret != ESP_OK){
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_32,
            .ws   = GPIO_NUM_33,
            .dout = GPIO_NUM_22,
            .din  = GPIO_NUM_34,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            }
        }
    };
    // Initializing the channels
    ret = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if(ret != ESP_OK){
        return ret;
    }

    ret = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if(ret != ESP_OK){
        return ret;
    }

    return ESP_OK;
}

static esp_err_t init_i2s_spk(){
    esp_err_t ret;

    // Initial i2s in duplex mode
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ret = i2s_new_channel(&chan_cfg, &tx_chan, NULL);
    if(ret != ESP_OK){
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_32,
            .ws   = GPIO_NUM_33,
            .dout = GPIO_NUM_25,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            }
        }
    };
    // Initializing the channels
    ret = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if(ret != ESP_OK){
        return ret;
    }

    return ESP_OK;
}

static esp_err_t i2s_read_task(){
    esp_err_t ret;
    printf("start to read\n");
    uint8_t * r_buf = (uint8_t *) calloc(1, BUFF_SIZE);
    size_t r_bytes = 0;

    ret = i2s_channel_enable(rx_chan);
    if(ret != ESP_OK){
        return ret;
    }
    printf("RX enable\n");

    while(1){
        printf("RX reading\n");
        ret = i2s_channel_read(rx_chan, r_buf, BUFF_SIZE, &r_bytes, portMAX_DELAY);
        if(ret == ESP_OK){
            printf("Read Task: i2s read %d bytes\n-----------------------------------\n", r_bytes);
            printf("[0] %x [1] %x [2] %x [3] %x\n[4] %x [5] %x [6] %x [7] %x\n\n",
                   r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5], r_buf[6], r_buf[7]);
        } else {
            printf("Read Task: i2s read failed\n");
            return ret;
        }
        printf("RX read finish\n");
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    free(r_buf);
    // vTaskDelete(NULL);
}

void app_main(void){
    esp_err_t ret;
    const TickType_t xDelay = 0.0625 * BUFF_SIZE / portTICK_PERIOD_MS; // 16000 sample rates

    ret = init_i2s();
    if(ret != ESP_OK){
        printf("err: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2s_channel_enable(rx_chan);
    if(ret != ESP_OK){
        printf("err: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2s_channel_enable(tx_chan);
    if(ret != ESP_OK){
        printf("err: %s", esp_err_to_name(ret));
        return;
    }


    uint8_t r_buf[BUFF_SIZE] = {0};
    size_t r_bytes = 0;
    while (1){
        vTaskDelay(xDelay);
        i2s_channel_read(rx_chan, r_buf, sizeof(r_buf), &r_bytes, portMAX_DELAY);
        i2s_channel_write(tx_chan, r_buf, sizeof(r_buf), &r_bytes, portMAX_DELAY);
    }

}
