#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "button.h"

#define TAG "BUTTON"

typedef struct
{
    uint8_t pin;
    bool inverted;
    uint16_t history;
    uint32_t down_time;
    uint32_t next_long_time;
} debounce_t;

debounce_t *d;
QueueHandle_t q;

#define MASK 0b1111000000111111
static bool button_rose(debounce_t *d)
{
    if ((d->history & MASK) == 0b0000000000111111)
    {
        d->history = 0xffff;
        return 1;
    }
    return 0;
}
static bool button_fell(debounce_t *d)
{
    if ((d->history & MASK) == 0b1111000000000000)
    {
        d->history = 0x0000;
        return 1;
    }
    return 0;
}
static bool button_down(debounce_t *d)
{
    if (d->inverted)
        return button_fell(d);
    return button_rose(d);
}
static bool button_up(debounce_t *d)
{
    if (d->inverted)
        return button_rose(d);
    return button_fell(d);
}

static uint32_t millis()
{
    return esp_timer_get_time() / 1000;
}

static void send_event(debounce_t d, int ev)
{
    button_event_t event = {
        .pin = d.pin,
        .event = ev,
    };
    xQueueSend(q, &event, portMAX_DELAY);
}

static void button_task(void *arg)
{
    while(1)
    {
        d->history = (d->history << 1) | gpio_get_level(d->pin);
        if (button_up(d))
        {
            d->down_time = 0;
            ESP_LOGI(TAG, "%d UP", d->pin);
            send_event(*d, BUTTON_UP);
        }
        else if (d->down_time && millis() >= d->next_long_time)
        {
            ESP_LOGI(TAG, "%d LONG", d->pin);
            d->next_long_time = d->next_long_time + CONFIG_ESP32_BUTTON_LONG_PRESS_REPEAT_MS;
            send_event(*d, BUTTON_HELD);
        }
        else if (button_down(d) && d->down_time == 0)
        {
            d->down_time = millis();
            ESP_LOGI(TAG, "%d DOWN", d->pin);
            d->next_long_time = d->down_time + CONFIG_ESP32_BUTTON_LONG_PRESS_DURATION_MS;
            send_event(*d, BUTTON_DOWN);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

QueueHandle_t button_init(unsigned long long pin_select)
{
    // Configure the pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1ULL << pin_select)),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);

    // Initialize global state and queue
    d = calloc(1, sizeof(debounce_t));
    q = xQueueCreate(CONFIG_ESP32_BUTTON_QUEUE_SIZE, sizeof(button_event_t));

    // Populate pin state
    ESP_LOGI(TAG, "Registering button input: %d", (int) pin_select);
    d->pin = pin_select;
    d->down_time = 0;
    d->inverted = false;
    if (d->inverted)
        d->history = 0xffff;

    // Spawn a task to monitor the pins
    xTaskCreate(&button_task, "button_task", CONFIG_ESP32_BUTTON_TASK_STACK_SIZE, NULL, 10, NULL);

    return q;
}