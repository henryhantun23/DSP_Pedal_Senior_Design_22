#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi_flash_mmap.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "driver/i2s.h"
#include "audio_example_file.h"
#include "esp_rom_sys.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

/*---------------------------------------------------------------
                            EXAMPLE CONFIG
---------------------------------------------------------------*/
static const char* TAG = "ad/da";
// reference voltage
#define V_REF   1100
// Select ADC1 channel (based on I/O) - ADC1 has I2S support
#define ADC1_TEST_CHANNEL (ADC1_CHANNEL_0)
// Name of flash partition to read/write
#define PARTITION_NAME   "storage"

//enable record sound and save in flash
#define RECORD_IN_FLASH_EN        (1)
//enable replay recorded sound in flash
#define REPLAY_FROM_FLASH_EN      (1)

//i2s number
#define EXAMPLE_I2S_NUM           (0)
//i2s sample rate
#define EXAMPLE_I2S_SAMPLE_RATE   (48000)
//i2s data bits
#define EXAMPLE_I2S_SAMPLE_BITS   (16)
//enable display buffer for debug
#define EXAMPLE_I2S_BUF_DEBUG     (0)
//I2S read buffer length
#define EXAMPLE_I2S_READ_LEN      (16 * 1024)
//I2S data format
// #define EXAMPLE_I2S_FORMAT        (I2S_CHANNEL_FMT_RIGHT_LEFT)
#define EXAMPLE_I2S_FORMAT        (I2S_CHANNEL_FMT_ALL_LEFT)
//I2S channel number
#define EXAMPLE_I2S_CHANNEL_NUM   ((EXAMPLE_I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0

//flash record size, for recording data
#define FLASH_RECORD_TIME         (5) // max recording time in seconds
#define FLASH_RECORD_SIZE         (EXAMPLE_I2S_CHANNEL_NUM * EXAMPLE_I2S_SAMPLE_RATE * EXAMPLE_I2S_SAMPLE_BITS / 8 * FLASH_RECORD_TIME)
#define FLASH_ERASE_SIZE          (FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE == 0) ? FLASH_RECORD_SIZE : FLASH_RECORD_SIZE + (FLASH_SECTOR_SIZE - FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE)
//sector size of flash
#define FLASH_SECTOR_SIZE         (0x1000)
//flash read / write address
#define FLASH_ADDR                (0x200000)

/*
 * @brief Init I2S driver
 */
void driver_i2s_init(void)
{
     int i2s_num = EXAMPLE_I2S_NUM;
     i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN | I2S_MODE_ADC_BUILT_IN,
        .sample_rate =  EXAMPLE_I2S_SAMPLE_RATE,
        .bits_per_sample = EXAMPLE_I2S_SAMPLE_BITS,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .channel_format = EXAMPLE_I2S_FORMAT,
        .intr_alloc_flags = 0,
        .dma_desc_num = 2,
        .dma_frame_num = 1024,
        .use_apll = 1,
     };
     //install and start i2s driver
     i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
     //init DAC pad
     i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
     //init ADC pad
     i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);
}

/**
 * @brief Scale data to 8bit for data from ADC.
 *        Data from ADC are 12bit width by default.
 *        DAC can only output 8 bit data.
 *        Scale each 12bit ADC data to 8bit DAC data.
 */
void example_i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t dac_value = 0;
#if (EXAMPLE_I2S_SAMPLE_BITS == 16)
    for (int i = 0; i < len; i += 2) {
        dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 4096;
    }
#else
    for (int i = 0; i < len; i += 4) {
        dac_value = ((((uint16_t)(s_buff[i + 3] & 0xf) << 8) | ((s_buff[i + 2]))));
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 4096;
    }
#endif
}

/*
 * @brief erase flash for recording
 */
void erase_flash(void)
{
    #if RECORD_IN_FLASH_EN
        printf("Erasing flash \n");
        const esp_partition_t *data_partition = NULL;
        data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
        if (data_partition != NULL) {
            printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
        }
        printf("Erase size: %d Bytes\n", FLASH_ERASE_SIZE);
        ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, 0, FLASH_ERASE_SIZE));
    #else
        printf("Skip flash erasing...\n");
    #endif
}

/*
 * @brief record audio to flash
 */
int flash_wr_size = 0;
void record_audio(void)
{
    printf("Started recording...");
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL) {
        printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    } else {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", PARTITION_NAME);
        vTaskDelete(NULL);
    }
    //1. Erase flash
    int i2s_read_len = EXAMPLE_I2S_READ_LEN;
    // int flash_wr_size = 0;
    size_t bytes_read;

    //2. Record audio from ADC and save in flash
    #if RECORD_IN_FLASH_EN
        char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
        uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
        i2s_adc_enable(EXAMPLE_I2S_NUM);
        while (flash_wr_size < FLASH_RECORD_SIZE) {
            //read data from I2S bus, in this case, from ADC.
            i2s_read(EXAMPLE_I2S_NUM, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
            //save original data from I2S(ADC) into flash.
            esp_partition_write(data_partition, flash_wr_size, i2s_read_buff, i2s_read_len);
            flash_wr_size += i2s_read_len;
            esp_rom_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
        }
        i2s_adc_disable(EXAMPLE_I2S_NUM);
        free(i2s_read_buff);
        i2s_read_buff = NULL;
        free(flash_write_buff);
        flash_write_buff = NULL;
    #endif
    // printf("Ended recording...");
    // return flash_wr_size;
}

/*
 * @brief playback audio from flash
 */
void playback_audio(void)
{
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
            ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL) {
        printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    } else {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", PARTITION_NAME);
        vTaskDelete(NULL);
    }

    int i2s_read_len = EXAMPLE_I2S_READ_LEN;
    size_t bytes_written;
    uint8_t* flash_read_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    uint8_t* i2s_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    
    #if REPLAY_FROM_FLASH_EN
        for (int rd_offset = 0; rd_offset < flash_wr_size; rd_offset += FLASH_SECTOR_SIZE) {
            //read I2S(ADC) original data from flash
            esp_partition_read(data_partition, rd_offset, flash_read_buff, FLASH_SECTOR_SIZE);
            //process data and scale to 8bit for I2S DAC.
            example_i2s_adc_data_scale(i2s_write_buff, flash_read_buff, FLASH_SECTOR_SIZE);
            //send data
            i2s_write(EXAMPLE_I2S_NUM, i2s_write_buff, FLASH_SECTOR_SIZE, &bytes_written, portMAX_DELAY);
            printf("playing: %d %%\n", rd_offset * 100 / flash_wr_size);
        }
    #endif
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
/*
 * @brief overdub audio - play then overwrite
 */
void overdub_audio(void)
{
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL) {
        printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    } else {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", PARTITION_NAME);
        vTaskDelete(NULL);
    }
    //1. Erase flash
    int i2s_read_len = EXAMPLE_I2S_READ_LEN;
    size_t bytes_read, bytes_written;
    uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    uint8_t* flash_read_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
    uint8_t* i2s_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    i2s_adc_enable(EXAMPLE_I2S_NUM);
    int rw_offset = 0;
    while(rw_offset < flash_wr_size){
        // Read from FLASH
        esp_partition_read(data_partition, rw_offset, flash_read_buff, FLASH_SECTOR_SIZE);
        // Scale data for DAC
        example_i2s_adc_data_scale(i2s_write_buff, flash_read_buff, FLASH_SECTOR_SIZE);
        // Write I2S to DAC
        i2s_write(EXAMPLE_I2S_NUM, i2s_write_buff, FLASH_SECTOR_SIZE, &bytes_written, portMAX_DELAY);
        // Read from ADC
        i2s_read(EXAMPLE_I2S_NUM, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
        // Overwrite current data in flash
        esp_partition_write(data_partition, rw_offset, i2s_read_buff, i2s_read_len);
        esp_rom_printf("Dubbing recording %u%%\n", rw_offset * 100 / FLASH_RECORD_SIZE);
        // Advance to next sector
        rw_offset += FLASH_SECTOR_SIZE;
    }
}

/*
 * @brief get input from USER
 */
void input_watch(void* arg)
{
    // TODO: Get USER input
    // TODO: Constant loop
    while(1){
        record_audio();
        playback_audio();
    }
    // while(1){
    //     overdub_audio();
    // }
    vTaskDelete(NULL);
}

void i2s_adc_dac_demo(void* arg)
{
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
            ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL) {
        printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    } else {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", PARTITION_NAME);
        vTaskDelete(NULL);
    }
    //"Erase"
    int i2s_read_len = EXAMPLE_I2S_READ_LEN;
    size_t bytes_read, bytes_written;
    
    //ADC & DAC
    uint8_t* i2s_read_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    uint8_t* i2s_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    
    while(1){
        i2s_adc_enable(EXAMPLE_I2S_NUM);
        i2s_read(EXAMPLE_I2S_NUM, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
        // example_disp_buf((uint8_t*) i2s_read_buff, 64);
        i2s_adc_disable(EXAMPLE_I2S_NUM);
        example_i2s_adc_data_scale(i2s_write_buff, i2s_read_buff, FLASH_SECTOR_SIZE);
        i2s_write(EXAMPLE_I2S_NUM, i2s_write_buff, FLASH_SECTOR_SIZE, &bytes_written, portMAX_DELAY);
    }
}

esp_err_t app_main(void)
{
    driver_i2s_init();
    esp_log_level_set("I2S", ESP_LOG_INFO);
    xTaskCreate(i2s_adc_dac_demo, "example_i2s_adc_dac", 1024 * 2, NULL, 5, NULL);
    // xTaskCreate(record_audio, "record_audio", 1024 * 2, NULL, 5, NULL);
    // xTaskCreate(playback_audio, "playback_audio", 1024 * 2, NULL, 5, NULL);
    // xTaskCreate(input_watch, "input_watch", 1024 * 2, NULL, 5, NULL);
    // erase_flash();
    // record_audio();
    // while(1)
    //     playback_audio();
    return ESP_OK;
}