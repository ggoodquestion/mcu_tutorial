#include <stdio.h>
#include <stdlib.h>

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_private/i2s_platform.h"
#include "esp_private/spi_flash_os.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2S_NUM I2S_NUM_0

#define I2S_BLCK 32
#define I2S_WS 33
#define I2S_DOUT 22
#define I2S_DIN 34

#define BUFF_SIZE 1024

static uint8_t r_buf[BUFF_SIZE];
static int16_t w_buf[BUFF_SIZE];

static i2s_chan_handle_t tx_chan;
static i2s_chan_handle_t rx_chan;

static int is_idle = 1;
static size_t r_bytes = 0;
static size_t w_bytes = 0;

#define GET_DMA_BUFFERS_BY_OFFSET(base_addr, offset)   (uint8_t **)(*((uint32_t *)base_addr + offset))


static void i2s_print_data(void *args)
{
    /* Enable the TX channel */
    // ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    i2s_channel_write(tx_chan, r_buf, r_bytes, &w_bytes, portMAX_DELAY);
    vTaskDelete(NULL);
}

static void IRAM_ATTR i2s_data_process_and_write(i2s_chan_handle_t tx_chan){
    size_t offset = i2s_platform_get_dma_buffer_offset() / sizeof(int16_t);
    uint8_t **dma_buff = GET_DMA_BUFFERS_BY_OFFSET(tx_chan, offset);

    // disable cache and non-iram ISR handlers
    spi_flash_guard_get()->start();
    // write data into dma buffer directly, the data in dma buffer will be sent automatically
    for (int i=0; i < r_bytes; i++) {
        dma_buff[0][i] = r_buf[i];
    }
    // enable cache and non-iram ISR handlers
    spi_flash_guard_get()->end();
}


static bool IRAM_ATTR i2s_on_rx_callback(i2s_chan_handle_t rx_handle, i2s_event_data_t *event, void *user_ctx){
    // int *is_idle = (int *) user_ctx;
    // if(*is_idle == 1){
    //     // memcpy(r_buf, (uint8_t *)(event->data), r_bytes);
    // }
    // *is_idle = 0;
    r_bytes = event->size;
    // memcpy(r_buf, (uint8_t *)(event->data), r_bytes);
    // i2s_data_process_and_write(tx_chan);
    
    i2s_channel_write(tx_chan, event->data, r_bytes, &w_bytes, portMAX_DELAY);
    // xTaskCreate(i2s_print_data, "i2s_print_data", 4096, NULL, 5, NULL);   
    return false;
}


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

    // Configure & Register i2s interrupt callback
    i2s_event_callbacks_t cbs = {
        .on_recv = i2s_on_rx_callback,
        .on_recv_q_ovf = NULL,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx_chan, &cbs, &is_idle));
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
    const TickType_t xDelay = 0.0625 * BUFF_SIZE / portTICK_PERIOD_MS; // 16000 sample rates

    init_i2s();
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));

    // uint8_t r_buf[BUFF_SIZE] = {0};
    size_t w_bytes = 0;
    // while (1){
    //     if
    // }

}
