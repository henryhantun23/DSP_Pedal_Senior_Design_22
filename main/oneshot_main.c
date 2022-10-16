/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/gptimer.h"
#include "esp_log.h"

const static char *TAG = "EXAMPLE";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_7

static int adc_raw[2][10];
static int voltage[2][10];
// static int adc_audio[2][10];
void example_adc_init(void);
void adc_read(void* arg);
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

/*---------------------------------------------------------------
        DAC General Macros
---------------------------------------------------------------*/
#define SAMPLE_FREQ            48000                                 // Sample frequency in Hz
#define TIMER_INTR_US          23                                    // Execution time of each ISR interval in micro-seconds
#define DAC_CHAN               DAC_CHANNEL_1           // DAC_CHANNEL_1 (GPIO25) by default

// _Static_assert(OUTPUT_POINT_NUM <= POINT_ARR_LEN, "The CONFIG_EXAMPLE_WAVE_FREQUENCY is too low and using too long buffer.");

// static int raw_val[POINT_ARR_LEN];                      // Used to store raw values
// static int volt_val[POINT_ARR_LEN];                    // Used to store voltage values(in mV)
// static const char *TAG = "wave_gen";

static int g_index = 0;

void example_adc_init(void)
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
        if (do_calibration1) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &voltage[0][0]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0][0]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1) {
        example_adc_calibration_deinit(adc1_cali_handle);
    }
}

DMA_ATTR int adc_val[1];
int adc_buf[16];
int dac_buf[16];

void adc_read(void* arg){
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    while(1){
        // ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_audio[0][0]));
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_audio[0][0]);
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_val[0]));
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_val[0]);
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}

adc_oneshot_unit_handle_t adc1_handle;

/* Timer interrupt service routine */
// ADC Timer
static bool IRAM_ATTR on_timer_alarm_adc(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, user_data);
    return false;
}

// DAC Timer
static bool IRAM_ATTR on_timer_alarm_dac(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    uint8_t dac_value = (int) user_data / 16;
    dac_output_voltage(DAC_CHAN, dac_value);
    // g_index++;
    return false;
}

void dac_init(void){
    g_index = 0;
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        // .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
        .resolution_hz = 48000*8, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(dac_output_enable(DAC_CHAN));

    // log_info();
    // prepare_data(OUTPUT_POINT_NUM);

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 8,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_timer_alarm_dac,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, adc_val));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    vTaskDelete(NULL);
}

void dac_write(void* arg){
    dac_init();
}

DMA_ATTR int val[0];
void trans_audio(void){
    
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    dac_output_enable(DAC_CHANNEL_1);

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &val[0]));
        val[0] /= 16;
        dac_output_voltage(DAC_CHANNEL_1, val[0]);
    }
}

void cont_adc_init(){
    adc_digi_pattern_config_t adc_digi_pat = {
        .atten = ADC_ATTEN_DB_11,
        .channel = ADC1_CHANNEL_7,
        .bit_width = 12,
    }
    
    adc_continuous_config_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 100,
        .pattern_num = 1,
        .adc_pattern = &adc_digi_pat,
        .sample_freq_hz = 48000 * 4,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,

    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config));
    ESP_ERROR_CHECK(adc_)

}

void app_main(void)
{
    trans_audio();
    // example_adc_init();
    // xTaskCreate(adc_read, "adc_read", 2*1024, NULL, 5, NULL);
    // xTaskCreate(dac_write, "dac_write", 2*1024, NULL, 5, NULL);
    // dac_init();
    //-------------ADC1 Init---------------//
    
    // adc_oneshot_unit_init_cfg_t init_config1 = {
    //     .unit_id = ADC_UNIT_1,
    // };
    // ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // //-------------ADC1 Config---------------//
    // adc_oneshot_chan_cfg_t config = {
    //     .bitwidth = ADC_BITWIDTH_12,
    //     .atten = ADC_ATTEN_DB_0,
    // };
    // ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    // g_index = 0;
    // gptimer_handle_t gptimer_dac = NULL;
    // gptimer_handle_t gptimer_adc = NULL;
    // gptimer_config_t timer_config_adc = {
    //     .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    //     .direction = GPTIMER_COUNT_UP,
    //     .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
    //     // .resolution_hz = SAMPLE_FREQ * config.bitwidth * 4,
    // };
    // gptimer_config_t timer_config_dac = {
    //     .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    //     .direction = GPTIMER_COUNT_UP,
    //     .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
    //     // .resolution_hz = SAMPLE_FREQ * 8 * 4,
    // };
    // ESP_ERROR_CHECK(gptimer_new_timer(&timer_config_dac, &gptimer_dac));
    // ESP_ERROR_CHECK(gptimer_new_timer(&timer_config_adc, &gptimer_adc));
    // ESP_ERROR_CHECK(dac_output_enable(DAC_CHAN));

    // gptimer_alarm_config_t alarm_config_adc = {
    //     .reload_count = 0,
    //     .alarm_count = 64,
    //     .flags.auto_reload_on_alarm = true,
    // };
    // gptimer_alarm_config_t alarm_config_dac = {
    //     .reload_count = 0,
    //     .alarm_count = 64,
    //     .flags.auto_reload_on_alarm = true,
    // };
    // gptimer_event_callbacks_t cb_dac = {
    //     .on_alarm = on_timer_alarm_dac,
    // };
    // gptimer_event_callbacks_t cb_adc = {
    //     .on_alarm = on_timer_alarm_adc,
    // };
    // ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_dac, &cb_dac, adc_val));
    // ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_dac, &alarm_config_dac));
    // ESP_ERROR_CHECK(gptimer_enable(gptimer_dac));
    // ESP_ERROR_CHECK(gptimer_start(gptimer_dac));

    // ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_adc, &cb_adc, adc_val));
    // ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_adc, &alarm_config_adc));
    // ESP_ERROR_CHECK(gptimer_enable(gptimer_adc));
    // ESP_ERROR_CHECK(gptimer_start(gptimer_adc));
    
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}