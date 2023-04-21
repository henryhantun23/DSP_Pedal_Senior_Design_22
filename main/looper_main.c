/* General Imports */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

/* Debounce Button Import */
#include "button.h"
QueueHandle_t button_events;
button_event_t blank_event = {
    .pin = GPIO_NUM_7,
    .event = 0,
};
button_event_t down_event = {
    .pin = GPIO_NUM_7,
    .event = 1,
};
button_event_t up_event = {
    .pin = GPIO_NUM_7,
    .event = 2,
};

/* I2S Imports */
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "sdkconfig.h"

/* SPI Imports */
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "soc/spi_pins.h"
#include "wear_levelling.h"
#include "spi_flash_mmap.h"
#include "esp_rom_sys.h"
#include <assert.h>

/* LED Controller Imports */
#include "driver/ledc.h"

/* DEBUG Imports */
#include "esp_timer.h"

/* GPIO Controller Declarations */
#define INIT    (-1)
#define RECORD  (0)
#define PLAY    (1)
#define OVERDUB (2)
#define ERASE   (3)

/* LEDC Global Declarations */
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 10 bits
#define LEDC_DUTY (1023)                // PWM Duty, max: (2 ** resolution) - 1
#define LEDC_FREQUENCY (60)             // Frequency in Hertz

/* I2S/SPI Shared Global Declarations */
#define I2S_BUFF_LEN  2048   // I2S buffer length in bytes, 2048 seems to work best for whatever reason, 4096 glitches heavily but is required for erase_range
#define PCM_CHANNELS  2      // 1 for mono, 2 for stereo
#define I2S_FS        48000  // i2s sample frequency
#define I2S_BIT_DEPTH 16     // number of bits / sample
#define MAX_REC_TIME  30     // max number of seconds in a recording

static i2s_chan_handle_t tx_chan; // I2S tx channel handler
static i2s_chan_handle_t rx_chan; // I2S rx channel handler

/* SPI Global Declarations */
#define FLASH_FREQ_MHZ 40 // SPI frequency in MHz

#define HOST_ID SPI2_HOST                // aka HSPI_HOST for ESP 32
#define PIN_MOSI HSPI_IOMUX_PIN_NUM_MOSI // pin 13
#define PIN_MISO HSPI_IOMUX_PIN_NUM_MISO // pin 12
#define PIN_CLK HSPI_IOMUX_PIN_NUM_CLK   // pin 14
#define PIN_CS HSPI_IOMUX_PIN_NUM_CS     // pin 15
#define PIN_WP HSPI_IOMUX_PIN_NUM_WP     // pin 2
#define PIN_HD HSPI_IOMUX_PIN_NUM_HD     // pin 4
#define SPI_DMA_CHAN SPI_DMA_CH_AUTO     // auto select available DMA channels

#define FLASH_RECORD_SIZE (PCM_CHANNELS * I2S_FS * I2S_BIT_DEPTH / 8 * MAX_REC_TIME)
#define FLASH_ERASE_SIZE (FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE == 0) ? FLASH_RECORD_SIZE : FLASH_RECORD_SIZE + (FLASH_SECTOR_SIZE - FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE)
#define FLASH_SECTOR_SIZE (0x1000) // sector size of flash, 4KB

// Handle of the wear levelling library instance
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
static const char *TAG = "flash";
// Mount path for the partition
const char *base_path = "/extflash";
#define PARTITION_NAME "storage"

static esp_flash_t *example_init_ext_flash(void);
static const esp_partition_t *example_add_partition(esp_flash_t *ext_flash, const char *partition_label);
static void example_list_data_partitions(void);
static bool example_mount_fatfs(const char *partition_label);

int user_sel = -1;
int rd_length = 0;
int num_layer = 0;
int max_layer = 2;

static void i2s_init_std_duplex(void) {
    /* Create new i2s channel config */
    // i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_AUTO,
        .role = I2S_ROLE_MASTER,
        .dma_frame_num = 2048, // previously 2046
        .dma_desc_num = 3, // previously 4
        .auto_clear = true,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan));

    /* Set TX & RX config */
    // BYP_ADC pin 33
    // FMT_ADC & FMT_DAC pin 27
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << GPIO_NUM_27) | (1ULL << GPIO_NUM_33)),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_27, 1);
    gpio_set_level(GPIO_NUM_33, 1);

    i2s_std_config_t std_cfg = {
        // .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .clk_cfg = {.sample_rate_hz = 48000, .clk_src = I2S_CLK_SRC_APLL, .mclk_multiple = I2S_MCLK_MULTIPLE_384},
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_0, // master clock aka SCKI, source clock
            .bclk = GPIO_NUM_25,
            .ws = GPIO_NUM_26, // word select aka LRCK, L/R channel clock
            .dout = GPIO_NUM_22, // DATA_DAC
            .din = GPIO_NUM_32, // DATA_ADC; In duplex mode, bind output and input to the same gpio to loopback internally
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    /* Initialize the channels */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
}

/* Demo Task: Read from ADC */
static void i2s_read_task(void *args)
{
    uint8_t *r_buf = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    assert(r_buf); // Check if r_buf allocation success
    size_t r_bytes = 0;
    /* ATTENTION: The print and delay in the read task only for monitoring the data by human,
     * Normally there shouldn't be any delays to ensure a short polling time,
     * Otherwise the dma buffer will overflow and lead to the data lost */
    while (1)
    {
        /* Read i2s data */
        if (i2s_channel_read(rx_chan, r_buf, I2S_BUFF_LEN, &r_bytes, 1000) == ESP_OK)
        {
            printf("Read Task: i2s read %d bytes\n-----------------------------------\n", r_bytes);
            printf("[0] %x [1] %x [2] %x [3] %x\n[4] %x [5] %x [6] %x [7] %x\n\n",
                   r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5], r_buf[6], r_buf[7]);
        }
        else
        {
            printf("Read Task: i2s read failed\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    free(r_buf);
    vTaskDelete(NULL);
}

/* Demo Task: Write to DAC */
static void i2s_write_task(void *args)
{
    uint8_t *w_buf = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    assert(w_buf); // Check if w_buf allocation success

    /* Assign w_buf */
    for (int i = 0; i < I2S_BUFF_LEN; i += 8)
    {
        w_buf[i] = 0x12;
        w_buf[i + 1] = 0x34;
        w_buf[i + 2] = 0x56;
        w_buf[i + 3] = 0x78;
        w_buf[i + 4] = 0x9A;
        w_buf[i + 5] = 0xBC;
        w_buf[i + 6] = 0xDE;
        w_buf[i + 7] = 0xF0;
    }
    size_t w_bytes = 0;
    while (1){
        /* Write i2s data */
        if (i2s_channel_write(tx_chan, w_buf, I2S_BUFF_LEN, &w_bytes, 1000) == ESP_OK) {
            printf("Write Task: i2s write %d bytes\n", w_bytes);
        } else {
            printf("Write Task: i2s write failed\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    free(w_buf);
    vTaskDelete(NULL);
}

/* Demo Task: Read from ADC & Write to DAC */
static void i2s_live_task(void *args) {

    uint8_t *i_buf = (uint8_t*)calloc(1, I2S_BUFF_LEN);
    assert(i_buf);
    size_t r_bytes = 0;
    size_t w_bytes = 0;

    while(user_sel == INIT){
        i2s_channel_read(rx_chan, i_buf, I2S_BUFF_LEN, &r_bytes, 1000);
        i2s_channel_write(tx_chan, i_buf, I2S_BUFF_LEN, &w_bytes, 1000);
    }
    
    free(i_buf);
    vTaskDelete(NULL);
}

static void spi_flash_init(void) {
    // Set up SPI bus and initialize the external SPI Flash chip
    esp_flash_t *flash = example_init_ext_flash();
    if (flash == NULL)
    {
        return;
    }

    // Add the entire external flash chip as a partition
    const char *partition_label = "ext_flash";
    example_add_partition(flash, partition_label);

    // List the available partitions
    example_list_data_partitions();

    // Initialize FAT FS in the partition
    if (!example_mount_fatfs(partition_label))
    {
        return;
    }

    // Print FAT FS size information
    uint64_t bytes_total, bytes_free;
    esp_vfs_fat_info(base_path, &bytes_total, &bytes_free);
    ESP_LOGI(TAG, "FAT FS: %" PRIu64 " kB total, %" PRIu64 " kB free", bytes_total / 1024, bytes_free / 1024);
}

static esp_flash_t *example_init_ext_flash(void)
{
    const spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadhd_io_num = PIN_HD,
        .quadwp_io_num = PIN_WP,
    };

    const esp_flash_spi_device_config_t device_config = {
        .host_id = HOST_ID,
        .cs_id = 0,
        .cs_io_num = PIN_CS,
        .io_mode = SPI_FLASH_QIO,
        .freq_mhz = FLASH_FREQ_MHZ,
    };

    ESP_LOGI(TAG, "Initializing external SPI Flash");
    ESP_LOGI(TAG, "Pin assignments:");
    ESP_LOGI(TAG, "MOSI: %2d   MISO: %2d   SCLK: %2d   CS: %2d",
             bus_config.mosi_io_num, bus_config.miso_io_num,
             bus_config.sclk_io_num, device_config.cs_io_num);

    // Initialize the SPI bus
    ESP_LOGI(TAG, "DMA CHANNEL: %d", SPI_DMA_CHAN);
    ESP_ERROR_CHECK(spi_bus_initialize(HOST_ID, &bus_config, SPI_DMA_CHAN));

    // Add device to the SPI bus
    esp_flash_t *ext_flash;
    ESP_ERROR_CHECK(spi_bus_add_flash_device(&ext_flash, &device_config));

    // Probe the Flash chip and initialize it
    esp_err_t err = esp_flash_init(ext_flash);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize external Flash: %s (0x%x)", esp_err_to_name(err), err);
        return NULL;
    }

    // Print out the ID and size
    uint32_t id;
    ESP_ERROR_CHECK(esp_flash_read_id(ext_flash, &id));
    ESP_LOGI(TAG, "Initialized external Flash, size=%" PRIu32 " KB, ID=0x%" PRIx32, ext_flash->size / 1024, id);

    return ext_flash;
}

static const esp_partition_t *example_add_partition(esp_flash_t *ext_flash, const char *partition_label)
{
    ESP_LOGI(TAG, "Adding external Flash as a partition, label=\"%s\", size=%" PRIu32 " KB", partition_label, ext_flash->size / 1024);
    const esp_partition_t *fat_partition;
    const size_t offset = 0;
    ESP_ERROR_CHECK(esp_partition_register_external(ext_flash, offset, ext_flash->size, partition_label, ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, &fat_partition));

    // Erase space of partition on the external flash chip
    ESP_LOGI(TAG, "Erasing partition range, offset=%u size=%" PRIu32 " KB", offset, ext_flash->size / 1024);
    ESP_ERROR_CHECK(esp_partition_erase_range(fat_partition, offset, ext_flash->size));
    return fat_partition;
}

static void example_list_data_partitions(void)
{
    ESP_LOGI(TAG, "Listing data partitions:");
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);

    for (; it != NULL; it = esp_partition_next(it))
    {
        const esp_partition_t *part = esp_partition_get(it);
        ESP_LOGI(TAG, "- partition '%s', subtype %d, offset 0x%" PRIx32 ", size %" PRIu32 " kB",
                 part->label, part->subtype, part->address, part->size / 1024);
    }

    esp_partition_iterator_release(it);
}

static bool example_mount_fatfs(const char *partition_label)
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE};
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, partition_label, &mount_config, &s_wl_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return false;
    }
    return true;
}

static void i2s_record_task() {
    const esp_partition_t *data_partition = NULL;
    // data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    if (data_partition != NULL) {
        printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    } else {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", "storage");
        vTaskDelete(NULL);
    }
    
    uint8_t *r_buf = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    assert(r_buf);
    size_t r_bytes = 0;
    size_t w_bytes = 0;
    int rd_offset = 0;

    // uint64_t start = esp_timer_get_time();
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, LEDC_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));

    while (rd_offset < FLASH_RECORD_SIZE && user_sel == RECORD) {
        /* Read i2s data */
        if (i2s_channel_read(rx_chan, r_buf, I2S_BUFF_LEN, &r_bytes, 1000) == ESP_OK){
            i2s_channel_write(tx_chan, r_buf, I2S_BUFF_LEN, &w_bytes, 1000);
            esp_partition_write(data_partition, rd_offset, r_buf, I2S_BUFF_LEN);
            rd_offset += r_bytes;
        } else {
            printf("Read Task: i2s read failed\n");
        }

        if (rd_offset >= FLASH_RECORD_SIZE * 0.9 && ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_1) == 0)
        {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, LEDC_DUTY));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
        }
    }
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));

    // uint64_t end = esp_timer_get_time();
    // ESP_LOGI("TASK", "Actual recording time: %llu milliseconds\n", (end - start) / 1000);

    num_layer = 1;
    rd_length = rd_offset + (4096 - (rd_offset % 4096));
    // rd_length = rd_offset;
    free(r_buf);
    if (user_sel == RECORD){
        xQueueSend(button_events, &down_event, portMAX_DELAY);
        xQueueSend(button_events, &up_event, portMAX_DELAY);
    }
    vTaskDelete(NULL);
}

static void i2s_playback_task() {
    const esp_partition_t *data_partition = NULL;
    // data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    if (data_partition != NULL) {
        printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    } else {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", "storage");
        vTaskDelete(NULL);
    }

    uint8_t *w_buf = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    uint8_t *w_buf_l1 = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    uint8_t *w_buf_l2 = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    uint8_t *w_buf_l3 = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    uint8_t *w_buf_l4 = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    assert(w_buf);
    assert(w_buf_l1);
    assert(w_buf_l2);
    assert(w_buf_l3);
    assert(w_buf_l4);
    size_t w_bytes = 0;

    // uint64_t start = esp_timer_get_time();
    while (user_sel == PLAY){ // while mode is play
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, LEDC_DUTY)); // set Green to high duty
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1)); // apply duty
        size_t wr_offset = 0; // offset for writing each layer to DAC
        while(wr_offset < rd_length && user_sel == PLAY){ // while more file to play AND mode is play
            if (num_layer == 1){
                esp_partition_read(data_partition, wr_offset, w_buf, I2S_BUFF_LEN);
                /* Write i2s data to DAC */
                esp_err_t i2s_wr_e = i2s_channel_write(tx_chan, w_buf, I2S_BUFF_LEN, &w_bytes, 1000);
                if (i2s_wr_e != ESP_OK)
                    ESP_LOGE(TAG, "Read Task: i2s write failed - %s (0x%x)", esp_err_to_name(i2s_wr_e), i2s_wr_e);
            } else if (num_layer == 2){
                esp_partition_read(data_partition, wr_offset, w_buf_l1, I2S_BUFF_LEN);
                esp_partition_read(data_partition, wr_offset + rd_length, w_buf_l2, I2S_BUFF_LEN);
                
                for (int i = 0; i < I2S_BUFF_LEN; i+=2){
                    uint16_t l1 = (w_buf_l1[i+1] << 8) + w_buf_l1[i];
                    uint16_t l2 = (w_buf_l2[i+1] << 8) + w_buf_l2[i];
                    if (l1 == 0x0){
                        w_buf[i] = w_buf_l2[i];
                        w_buf[i+1] = w_buf_l2[i+1];
                    } else if (l2 == 0x0){
                        w_buf[i + 1] = w_buf_l1[i + 1];
                        w_buf[i] = w_buf_l1[i];
                    } else {
                        uint16_t sample = (l1 + l2);
                        w_buf[i] = (sample & 0x00ff);
                        w_buf[i + 1] = (sample & 0xff00) >> 8;
                    }
                }
                /* Write i2s data to DAC */
                esp_err_t i2s_wr_e = i2s_channel_write(tx_chan, w_buf, I2S_BUFF_LEN, &w_bytes, 1000);
                if (i2s_wr_e != ESP_OK)
                    ESP_LOGE(TAG, "Read Task: i2s write failed - %s (0x%x)", esp_err_to_name(i2s_wr_e), i2s_wr_e);
            }
            wr_offset += I2S_BUFF_LEN; // increment offset
        }
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
    }

    // uint64_t end = esp_timer_get_time();
    // ESP_LOGI("TASK", "Actual recording time: %llu milliseconds\n", (end - start) / 1000);
    free(w_buf_l1);
    free(w_buf_l2);
    // free(w_buf_l3);
    // free(w_buf_l4);
    vTaskDelete(NULL);
}

static void i2s_overdub_task(void *arg) {
    /* Locate External Flash Partition */
    const esp_partition_t *data_partition = NULL;
    // data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    if (data_partition != NULL) {
        printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    } else {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", "storage");
        vTaskDelete(NULL);
    }

    /* Initialize Buffers */
    // Reading from Flash
    uint8_t *r_buf = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    assert(r_buf);
    size_t r_bytes = 0;
    // Writing to Flash
    uint8_t *w_buf = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    assert(w_buf);
    size_t w_bytes = 0;

    // Time overdub loop
    uint64_t start_loop = esp_timer_get_time();
    uint64_t max_trans_time = 0;

    while (user_sel == OVERDUB && num_layer < max_layer){
        // Enable record light
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, LEDC_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));

        // Reset pointer to 0
        size_t od_offset = 0;
        // While not over record length AND mode is overdub
        while (od_offset < rd_length && user_sel == OVERDUB) {
            uint64_t start = esp_timer_get_time();
            // Flash -> I2S -> Audio Out
            esp_partition_read(data_partition, od_offset, w_buf, I2S_BUFF_LEN);
            esp_err_t i2s_wr_e = i2s_channel_write(tx_chan, w_buf, I2S_BUFF_LEN, &w_bytes, 1000);
            if (i2s_wr_e != ESP_OK)
            {
                ESP_LOGE(TAG, "Overdub Task: i2s write failed - %s (0x%x)", esp_err_to_name(i2s_wr_e), i2s_wr_e);
            }

            // uint64_t end = esp_timer_get_time();
            // if (end - start > max_trans_time)
            //     max_trans_time = end - start;

            // start = esp_timer_get_time();
            // Audio in -> I2S -> Flash
            if (i2s_channel_read(rx_chan, r_buf, I2S_BUFF_LEN, &r_bytes, 1000) == ESP_OK){
                esp_partition_write(data_partition, od_offset + rd_length, r_buf, I2S_BUFF_LEN);
            }

            // Verify transaction complete
            if (r_bytes != w_bytes) {
                ESP_LOGE(TAG, "Overdub Task: record and playback out of sync – read offset: %x, write offset: %x\n", r_bytes, w_bytes);
                r_bytes = r_bytes > w_bytes ? w_bytes : r_bytes;
                w_bytes = w_bytes > r_bytes ? r_bytes : w_bytes;
            }
            od_offset += I2S_BUFF_LEN;

            if (od_offset >= FLASH_RECORD_SIZE * 0.9 && ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_1) == 0)
            {
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, LEDC_DUTY));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
            }

            uint64_t end = esp_timer_get_time();
            if (end - start > max_trans_time)
                max_trans_time = end - start;
        }
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
        if (num_layer < max_layer) num_layer = num_layer + 1;
    }

    ESP_LOGI("OVERDUB", "Maximum Transaction Time: %llu milliseconds\n", max_trans_time / 1000);

    // End Timer
    uint64_t end_loop = esp_timer_get_time();
    ESP_LOGI("OVERDUB", "Actual overdub time: %llu milliseconds\n", (end_loop - start_loop) / 1000);
    printf("Current Layer is now: %d\n", num_layer);
    free(r_buf);
    free(w_buf);
    if (user_sel == OVERDUB){
        xQueueSend(button_events, &down_event, portMAX_DELAY);
        xQueueSend(button_events, &up_event, portMAX_DELAY);
    }
    vTaskDelete(NULL);
}

static void dummy_write(void *arg) {
    const esp_partition_t *data_partition = NULL;
    // data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "ext_flash");
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    if (data_partition != NULL) {
        printf("partiton addr: 0x%08"PRIx32"; size: %"PRIu32"; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    } else {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", "ext_flash");
        vTaskDelete(NULL);
    }

    ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, 0, data_partition->size));
    uint8_t *r_buf = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    assert(r_buf);
    size_t r_bytes = 0;
    int rd_offset = 0;
    uint8_t *w_buf = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    assert(w_buf);

    for (int i = 0; i < I2S_BUFF_LEN; i += 8)
    {
        r_buf[i]     = 0x12;
        r_buf[i + 1] = 0x34;
        r_buf[i + 2] = 0x56;
        r_buf[i + 3] = 0x78;
        r_buf[i + 4] = 0x9A;
        r_buf[i + 5] = 0xBC;
        r_buf[i + 6] = 0xDE;
        r_buf[i + 7] = 0xF0;
    }

    // FILE *f = fopen("/storage/dummy.txt", "wb");
    // if (f == NULL) {
    //     ESP_LOGE(TAG, "Failed to open file for writing");
    //     return;
    // }

    uint64_t start = esp_timer_get_time();

    for (size_t wr_offset = 0; wr_offset < FLASH_RECORD_SIZE; wr_offset += I2S_BUFF_LEN){
        // fwrite(r_buf, I2S_BUFF_LEN, 1, f);
        esp_partition_write(data_partition, wr_offset, r_buf, I2S_BUFF_LEN);
        printf("Bytes written: %d\n Total recorded: %d%%\n\n", wr_offset, wr_offset * 100 / FLASH_RECORD_SIZE);
        vTaskDelay(1);
        printf("[0] %x [1] %x [2] %x [3] %x\n[4] %x [5] %x [6] %x [7] %x\n\n",
                r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5], r_buf[6], r_buf[7]);
    }
    uint64_t end = esp_timer_get_time();
    ESP_LOGI("TASK", "Actual writing time: %llu milliseconds\n", (end - start) / 1000);
    // fclose(f);
    free(r_buf);
    vTaskDelete(NULL);
}

static void dummy_read(void)
{
    const esp_partition_t *data_partition = NULL;
    // data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "ext_flash");
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    if (data_partition != NULL)
    {
        printf("partiton addr: 0x%08" PRIx32 "; size: %" PRIu32 "; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    }
    else
    {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", "ext_flash");
        vTaskDelete(NULL);
    }

    uint8_t *w_buf = (uint8_t *)calloc(1, I2S_BUFF_LEN);
    assert(w_buf);

    uint64_t start = esp_timer_get_time();

    // FILE *f = fopen("/storage/dummy.wav", "rb");
    // if (f == NULL)
    // {
    //     ESP_LOGE(TAG, "Failed to open file for reading");
    //     return;
    // }

    // fseek(f, 0, SEEK_SET);

    for (size_t wr_offset = 0; wr_offset < FLASH_RECORD_SIZE; wr_offset += I2S_BUFF_LEN)
    {
        // fread(w_buf, I2S_BUFF_LEN, 1, f);
        esp_partition_read(data_partition, wr_offset, w_buf, I2S_BUFF_LEN);
        printf("Bytes read: %d\n Total played: %d%%\n\n", wr_offset, wr_offset * 100 / FLASH_RECORD_SIZE);
        for (int i = 0; i < I2S_BUFF_LEN; i+= 8){
            printf("[0] %x [1] %x [2] %x [3] %x\n[4] %x [5] %x [6] %x [7] %x\n\n",
               w_buf[i], w_buf[i+1], w_buf[i+2], w_buf[i+3], w_buf[i+4], w_buf[i+5], w_buf[i+6], w_buf[i+7]);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    uint64_t end = esp_timer_get_time();
    ESP_LOGI("TASK", "Actual reading time: %llu milliseconds\n", (end - start) / 1000);

    free(w_buf);
    vTaskDelete(NULL);
}

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_r_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = 19,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_r_channel));

    ledc_channel_config_t ledc_g_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = 20,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_g_channel));

    ledc_channel_config_t ledc_b_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = 21,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_b_channel));
}

TaskHandle_t live;
TaskHandle_t record;
TaskHandle_t playback;
TaskHandle_t overdub;

static void button_test(void) {
    gpio_config_t io_conf_bm = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << GPIO_NUM_8)),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf_bm);
    gpio_set_level(GPIO_NUM_8, 1);

    button_event_t ev;
    // QueueHandle_t button_events = button_init(GPIO_NUM_7);
    button_events = button_init(GPIO_NUM_7);

    // Default operation
    // user_sel defaults to -1
    printf("Looper not started... Digital passthrough...\n");
    xTaskCreate(i2s_live_task, "i2s_live_task", 4096, NULL, 5, &live);

    while(1)
    {
        if (xQueueReceive(button_events, &ev, 1000 / portTICK_PERIOD_MS) && user_sel != ERASE)
        {
            if (ev.event == BUTTON_DOWN){
                vTaskDelay(100 / portTICK_PERIOD_MS); // in theory to allow clean response
                printf("GPIO level: %d, User Selection: %d\n", gpio_get_level(GPIO_NUM_7), user_sel);
                switch(user_sel){
                    case RECORD: 
                        user_sel = PLAY;
                        break;
                    case PLAY: 
                        user_sel = OVERDUB;
                        break;
                    case OVERDUB: 
                        user_sel = PLAY;
                        break;
                    default: user_sel = RECORD;
                }
                printf("New selection value: %d\n", user_sel);
            
                /* User Options for Short Press */
                // Record (1st press) -> then play
                // Play (2n th press) -> then overdub
                // Overdub (2n+1 th press) -> then play
                if (user_sel == RECORD){
                    printf("Recording started...");
                    xTaskCreate(i2s_record_task, "i2s_record_task", 4096, NULL, 5, &record);
                    // vTaskDelay(200 / portTICK_PERIOD_MS); // in theory to allow clean shutdown
                } else if (user_sel == PLAY){ // play until said otherwise
                    // vTaskDelay(100 / portTICK_PERIOD_MS);
                    printf("Playback started...");
                    xTaskCreate(i2s_playback_task, "i2s_playback_task", 4096, NULL, 5, &playback);
                    // vTaskDelay(200 / portTICK_PERIOD_MS); // in theory to allow clean shutdown
                } else if (user_sel == OVERDUB){
                    printf("Overdub started...");
                    xTaskCreate(i2s_overdub_task, "i2s_overdub_task", 4096, NULL, 5, &overdub);
                //     vTaskDelay(200 / portTICK_PERIOD_MS); // in theory to allow clean shutdown
                }
            } else if (ev.event == BUTTON_HELD){
                // user_sel = ERASE;
                printf("Erasing layer...\n");
                i2s_channel_disable(tx_chan);
                i2s_channel_disable(rx_chan);
                /* ATTEMPTED CHANGES */
                // cancel all current tasks //
                if (record != NULL && eTaskGetState(record) != eDeleted)
                {
                    vTaskDelete(record);
                }
                printf("Stopped record...\n");
                if (playback != NULL && eTaskGetState(playback) != eDeleted)
                {
                    vTaskDelete(playback);
                }
                printf("Stopped playback...\n");
                if (overdub != NULL && eTaskGetState(overdub) != eDeleted)
                {
                    vTaskDelete(overdub);
                }
                printf("Stopped overdub...\n");

                printf("Clearing layer...\n");
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
                const esp_partition_t *data_partition = NULL;
                // data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");
                data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
                if (data_partition != NULL)
                {
                    printf("partiton addr: 0x%08" PRIx32 "; size: %" PRIu32 "; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
                }
                else
                {
                    ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", "storage");
                }

                // num_layer = num_layer - 1;

                printf("Num layers: %d\n", num_layer);
                if (num_layer <= 1 || user_sel == OVERDUB){
                    printf("Deleting first layer...\n");
                    ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, 0, data_partition->size));
                    i2s_channel_enable(tx_chan);
                    i2s_channel_enable(rx_chan);
                    // xTaskCreate(i2s_live_task, "i2s_live_task", 4096, NULL, 5, &live);
                    user_sel = INIT; // so that next button press -> record
                    num_layer = 0;
                } else if (num_layer > 1){
                    printf("Deleting nth layer...\n");
                    ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, rd_length * (num_layer-1), rd_length));
                    i2s_channel_enable(tx_chan);
                    i2s_channel_enable(rx_chan);
                    user_sel = RECORD; // so that next button press -> playback
                    num_layer -= 1;
                }
                printf("Layer cleared...\n");
                xQueueReset(button_events);
                ev.event = BUTTON_UP;
            }
        }
    }
}

void app_main(void) {
    ledc_init();
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, LEDC_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2));

    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    assert(partition != NULL);
    ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, partition->size));

    // vTaskDelay(2000 / portTICK_PERIOD_MS);

    /* Initialize External Flash */
    // spi_flash_init();
    // vTaskDelay(50 / portTICK_PERIOD_MS);

    /* init i2s_std driver in duplex mode following schematic */
    i2s_init_std_duplex();

    /* Enable the tx and rx channels before writing or reading data */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2));

    /* Demo Tasks */
    // xTaskCreate(i2s_read_task, "i2s_example_read_task", 4096, NULL, 5, NULL); // read from ADC via i2s
    // xTaskCreate(i2s_write_task, "i2s_example_write_task", 4096, NULL, 5, NULL); // write to DAC via i2s
    // xTaskCreate(i2s_live_task, "live_translate", 4096, NULL, 5, NULL); // live audio conversion
    // xTaskCreate(dummy_write, "dummy_write", 4096, NULL, 5, NULL);
    // vTaskDelay(6000 / portTICK_PERIOD_MS);
    // xTaskCreate(dummy_read, "dummy_read", 4096, NULL, 5, NULL);

    /* Actual Tasks */
    // xTaskCreate(io_controller, "io_controller", 1024, NULL, 5, NULL); // loop for controlling user IO
    // xTaskCreate(i2s_record_task, "i2s_record_task", 4096, NULL, 5, NULL); // record audio to flash
    // xTaskCreate(i2s_playback_task, "i2s_playback_task", 4096, NULL, 5, NULL); // play audio from flash
    // xTaskCreate(i2s_overdub_task, "i2s_overdub_task", 8192, NULL, 5, NULL); // overdub audio on flash

    button_test();

    // xTaskCreate(i2s_record_task, "i2s_record_task", 4096, NULL, 5, NULL); // record audio to flash
    // vTaskDelay((MAX_REC_TIME + 1) * 1000 / portTICK_PERIOD_MS);
    // // xTaskCreate(i2s_playback_task, "i2s_playback_task", 4096, NULL, 5, NULL); // play audio from flash
    // // vTaskDelay((MAX_REC_TIME + 1) * 1000 / portTICK_PERIOD_MS);
    // xTaskCreate(i2s_overdub_task, "i2s_overdub_task", 4096, NULL, 5, NULL); // play audio from flash
    // vTaskDelay((MAX_REC_TIME + 1) * 1000 / portTICK_PERIOD_MS);
}