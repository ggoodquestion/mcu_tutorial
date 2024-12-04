#include <stdio.h>
#include <stdlib.h>

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2S_BLCK 32
#define I2S_WS 33
#define I2S_DOUT 22
#define I2S_DIN 34

#define BUFF_SIZE 128

#define ALPHA 0.97

static int16_t r_buf[BUFF_SIZE];
static int16_t w_buf[BUFF_SIZE];
static float memory[1] = {0};

static i2s_chan_handle_t tx_chan;
static i2s_chan_handle_t rx_chan;

static void init_i2s(){
    // Initial i2s in duplex mode
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan));

    // Notice that GPIO num for bclk and ws should be able to PWM output
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BLCK,
            .ws   = I2S_WS,
            .dout = I2S_DOUT,
            .din  = I2S_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            }
        }
    };
    // Initializing the channels
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
}

void preemphasis(int16_t *xs, int16_t *y){
    float tmp = 0.0;
    for(int n = 0; n < BUFF_SIZE; n+=2){
        tmp = (float) xs[n] - ALPHA * memory[0];
        memory[0] = (float) xs[n];
        y[n] = (int16_t) tmp;
    }
}

void app_main(void){
    const TickType_t xDelay = 0.0625 * BUFF_SIZE / portTICK_PERIOD_MS; // 16000 sample rates

    init_i2s();
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));

    size_t r_bytes = 0;
    size_t w_bytes = 0;
    while (1){
        vTaskDelay(xDelay);
        i2s_channel_read(rx_chan, r_buf, sizeof(r_buf), &r_bytes, portMAX_DELAY);
        preemphasis(r_buf, w_buf);
        i2s_channel_write(tx_chan, w_buf, sizeof(w_buf), &w_bytes, portMAX_DELAY);
    }

}
