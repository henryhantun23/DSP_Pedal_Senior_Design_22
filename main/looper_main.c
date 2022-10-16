/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_partition.h"
#include "spi_flash_mmap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/dac.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define EXAMPLE_READ_LEN   32 // max 32 samples so far -> must be  << 520KB for SRAM
#define GET_UNIT(x)        ((x>>3) & 0x1)

#define ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1  //ESP32 only supports ADC1 DMA mode
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE1


// #define EXAMPLE_I2S_READ_LEN      (16 * 1024)
#define PARTITION_NAME   "storage"

#define SAMPLE_FREQ               (48000)
#define SAMPLE_BITS               (16)
//flash record size, for recording 5 seconds' data
#define FLASH_RECORD_LEN          (10)
#define FLASH_RECORD_SIZE         (SAMPLE_FREQ * SAMPLE_BITS / 8 * FLASH_RECORD_LEN)
#define FLASH_ERASE_SIZE          (FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE == 0) ? FLASH_RECORD_SIZE : FLASH_RECORD_SIZE + (FLASH_SECTOR_SIZE - FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE)
//sector size of flash
#define FLASH_SECTOR_SIZE         (0x1000)
//flash read / write address
#define FLASH_ADDR                (0x200000)

    static adc_channel_t channel[1] = {ADC_CHANNEL_7};

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        // .max_store_buf_size = 64,
        // .conv_frame_size = 16,
        .max_store_buf_size = EXAMPLE_READ_LEN * 32,
        .conv_frame_size = EXAMPLE_READ_LEN * 4,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 48000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        uint8_t unit = GET_UNIT(channel[i]);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_0;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void record_loop(void)
{
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    
    if (data_partition != NULL) {
        printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    } else {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", PARTITION_NAME);
        vTaskDelete(NULL);
    }

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN*2] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN*2);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));

    int adc_read_len = EXAMPLE_READ_LEN; // number of samples to take at a time; works only when 2 -> TODO: find a way to increase
    int flash_wr_size = 0;
    uint16_t* adc_read_buf = (uint16_t*) calloc(adc_read_len, sizeof(uint16_t)); // each sample is 2 bytes (12bits + 4bit overhead)

    ESP_ERROR_CHECK(adc_continuous_start(handle));
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint64_t start = esp_timer_get_time();

    while (flash_wr_size < FLASH_RECORD_SIZE) {
        ret = adc_continuous_read(handle, result, adc_read_len*2, &ret_num, ADC_MAX_DELAY); // adc_read_len must be divisible by 4
        // ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32, ret, ret_num); // load-bearing print statement
        // vTaskDelay(pdMS_TO_TICKS(10));
        if (ret == ESP_OK) {
            for (int i = 0; i < ret_num; i += 2) {
                adc_digi_output_data_t *p = (void*)&result[i];
                // ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %x", 1, p->type1.channel, p->type1.data);
                // uint16_t data = (uint16_t) p->val;
                // adc_read_buf[i]   = (char) data >> 8;
                // adc_read_buf[i+1] = (char) data & 0x0ff;
                adc_read_buf[i] = (uint16_t) p->val;
            }
            // ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32, ret, ret_num);
            // ESP_ERROR_CHECK(adc_continuous_stop(handle));
            ESP_ERROR_CHECK(esp_partition_write(data_partition, flash_wr_size, adc_read_buf, adc_read_len * 2));
            flash_wr_size += adc_read_len * 2;
            // ESP_ERROR_CHECK(adc_continuous_start(handle));
            // ESP_LOGI("TASK", "Flash write size %d\n", flash_wr_size);
            // ESP_LOGI("TASK", "Flash write size %d\n", FLASH_RECORD_SIZE);
            // ESP_LOGI("TASK", "Sound recording %d%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE("TASK", "Error timout");
            break;
        }
    }
    uint64_t end = esp_timer_get_time();
    ESP_LOGI("TASK", "Sound recording %d%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
    ESP_LOGI("TASK", "Actual recording time: %llu milliseconds\n", (end - start) / 1000);
    vTaskDelete(NULL);
}

uint8_t raw_val[FLASH_SECTOR_SIZE] = {0};
int rd_idx = 0;

/* Timer interrupt service routine */
static bool IRAM_ATTR on_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    int *head = (int *)user_data;

    /* DAC output ISR has an execution time of 4.4 us*/
    if (rd_idx >= FLASH_SECTOR_SIZE / 2)
    {
        rd_idx = 0;
    }

    dac_output_voltage(DAC_CHANNEL_1, *(head + rd_idx));
    return false;
}

void playback_loop(void)
{
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    
    if (data_partition != NULL) {
        printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    } else {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", PARTITION_NAME);
        vTaskDelete(NULL);
    }

    rd_idx = 0;
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        // .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
        .resolution_hz = SAMPLE_FREQ*8, // 48kHz sample rate
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(dac_output_enable(DAC_CHANNEL_1));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 24,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_timer_alarm_cb,
    };

    uint16_t *flash_read_buff = (uint16_t *)calloc(FLASH_SECTOR_SIZE/2, sizeof(uint16_t));
    int flash_wr_size = FLASH_RECORD_SIZE;

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, raw_val));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    while (1) {
        if (rd_idx == 0){
            ESP_ERROR_CHECK(gptimer_stop(gptimer));
            for (int rd_offset = 0; rd_offset < flash_wr_size; rd_offset += FLASH_SECTOR_SIZE)
            {
                esp_partition_read(data_partition, rd_offset, flash_read_buff, FLASH_SECTOR_SIZE);
                for (int i = 0; i < FLASH_SECTOR_SIZE/2; i += 1){
                    raw_val[i] = flash_read_buff[i] >> 4;
                    // printf("%d", raw_val[i]);
                }
            }
            ESP_ERROR_CHECK(gptimer_start(gptimer));
        }
    }
}

void erase_flash(void)
{
#if RECORD_IN_FLASH_EN
    printf("Erasing flash \n");
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                              ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL)
    {
        printf("partiton addr: 0x%08" PRIx32 "; size: %" PRIu32 "; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    }
    printf("Erase size: %d Bytes\n", FLASH_ERASE_SIZE);
    ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, 0, FLASH_ERASE_SIZE));
#else
    printf("Skip flash erasing...\n");
#endif
}

void live_audio(void)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    dac_output_enable(DAC_CHANNEL_1);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while(1) {

        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1) {
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                // ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32, ret, ret_num);
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (void*)&result[i];
                    // ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %x", 1, p->type1.channel, p->type1.data);
                    dac_output_voltage(DAC_CHANNEL_1, p->type1.data >> 4);
                }
                /**
                 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                 * usually you don't need this delay (as this task will block for a while).
                 */
                // vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}

esp_err_t app_main(void)
{
    live_audio();
    // record_loop();
    // playback_loop(); // TODO
    return ESP_OK;
}