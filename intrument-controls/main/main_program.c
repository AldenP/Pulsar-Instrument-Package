/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"

#include "driver/gpio.h"

#define START_BUT   GPIO_NUM_35
#define STOP_BUT    GPIO_NUM_34

#define GREEN_LED   GPIO_NUM_0      // debug.
#define RED_LED     GPIO_NUM_13     // status

#define GPIO_IN_MASK (1ULL << START_BUT | 1ULL << STOP_BUT)
#define GPIO_OUT_MASK (1ULL << GREEN_LED | 1ULL << RED_LED)

#define ESP_INTR_FLAG_DEFAULT 0 //define flag for gpio ISRs

#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_2_5
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN                    2048

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[1] = {ADC_CHANNEL_0};//{ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3}; //don't forget size
#else
static adc_channel_t channel[2] = {ADC_CHANNEL_2, ADC_CHANNEL_3};
#endif

// TASKS
#define TASK_STACK_SIZE                     4096
static TaskHandle_t main_task_handle;
static const char *MAIN_TAG = "MAIN";

static TaskHandle_t adc_print_handle;
static const char *ADC_PRINT_TAG = "ADC-PRINT";
// vars for state
volatile bool adc_state = false; 
volatile bool led_state = false;
volatile bool red_led_state = false;

// LOCKS for concurrency
static portMUX_TYPE adc_lock = portMUX_INITIALIZER_UNLOCKED;  //spinlock for adc_state

// Struct to hold pointers to more than one kind of handle for ISR functions
typedef struct handlers_t {
    adc_continuous_handle_t *adc_handle;
    // gptimer_handle_t *gptimer;
} my_handlers_t;

static bool IRAM_ATTR adc_conv_ready_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(adc_print_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 8 * ADC_BUFFER_LEN,
        .conv_frame_size = ADC_BUFFER_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,    //frequency of conversion.
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(MAIN_TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(MAIN_TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(MAIN_TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static void IRAM_ATTR start_isr_handler(void* arg) {
    // start the continuous ADC reading. 
    // provide visual output
    led_state = !led_state;
    gpio_set_level(GREEN_LED, led_state);
    // get ADC handler from arg

    my_handlers_t *handles = (my_handlers_t*) arg;
    adc_continuous_handle_t *adc_handle = handles->adc_handle;

    esp_err_t ret;
    if (!adc_state) {
        // turn on / start the ADC, within critical section
        taskENTER_CRITICAL_ISR(&adc_lock);
        // ret = adc_continuous_start(*adc_handle);
        ESP_ERROR_CHECK(adc_continuous_start(*adc_handle)); //error here: abort from lock_acquire_generic
        adc_state = true;
        taskEXIT_CRITICAL_ISR(&adc_lock);
        xTaskResumeFromISR(adc_print_handle);
        // if (ret != ESP_OK) {
        //     // do something? - can't print in ISR!
        //     // return ret;  //void type
        // }

        gpio_set_level(RED_LED, 1); //turn red LED on.
    }
}
static void IRAM_ATTR stop_isr_handler(void* arg) {
    // stop the continuous ADC reading. 
    // provide visual output
    led_state = !led_state;
    gpio_set_level(GREEN_LED, led_state);   // to be ISR/IRAM safe, check config editor under GPIO.

    // get ADC handler from arg
    my_handlers_t *handles = (my_handlers_t*) arg;
    adc_continuous_handle_t *adc_handle = handles->adc_handle;

    esp_err_t ret;
    if (adc_state) {
        // turn off ADC
        taskENTER_CRITICAL_ISR(&adc_lock);
        // ret = adc_continuous_stop(*adc_handle);
        ESP_ERROR_CHECK(adc_continuous_stop(*adc_handle));
        adc_state = false;
        taskEXIT_CRITICAL_ISR(&adc_lock);

        // if (ret != ESP_OK) {
        //     return;
        // }
        gpio_set_level(RED_LED, 0);
    }
}

static void adc_print_task(void* args) {
    
    my_handlers_t * handles = (my_handlers_t*) args;
    adc_continuous_handle_t *adc_handle = handles->adc_handle;

    uint32_t byte_count = 0;

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[ADC_BUFFER_LEN] = {0};
    memset(result, 0xcc, ADC_BUFFER_LEN);
    ESP_LOGI(ADC_PRINT_TAG, "adc_print_task created and waiting.");
    while (1) {

        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

        while (1) {
            ret = adc_continuous_read(adc_handle, result, ADC_BUFFER_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                    /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                    if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                        // ESP_LOGI(MAIN_TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
                        printf("ch: %"PRIu32"; value: %"PRIx32"\n", chan_num, data); 
                    } else {
                        ESP_LOGW(ADC_PRINT_TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                    }
                }
                byte_count += ret_num;
                if (byte_count >= 16*1024) {
                    // stop ADC and suspend task
                    ESP_LOGI(ADC_PRINT_TAG, "byte count: %"PRIu32"", byte_count);

                    if (adc_state) {
                        adc_continuous_stop(adc_handle);
                        adc_state = false;
                        gpio_set_level(RED_LED, 0);
                        vTaskSuspend(adc_print_handle);
                    }
                }
                /**
                 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                 * usually you don't need this delay (as this task will block for a while).
                 */
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
            else {
                ESP_LOGW(ADC_PRINT_TAG, "bad ret value %d", ret);
                vTaskSuspend(adc_print_handle);
                break;
            }
        }
    }
}

// TaskHandle_t main_handle = NULL;

void app_main(void)
{
    esp_err_t ret;

    //gpio setup
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_IN_MASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    gpio_config(&io_conf);

    // io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUT_MASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    gpio_config(&io_conf);
    gpio_set_level(RED_LED, 0);
    gpio_set_level(GREEN_LED, 0);
    
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);    //enable ISRs to be added.


    main_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conv_ready_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    // ESP_ERROR_CHECK(adc_continuous_start(handle));

    my_handlers_t handles = {
        .adc_handle = &handle,
    };
    // Install ISRs for start and stop. 
    gpio_isr_handler_add(START_BUT, start_isr_handler, (void*)&handles);
    gpio_isr_handler_add(STOP_BUT, stop_isr_handler, (void*)&handles);

    // register a task
    xTaskCreate(adc_print_task, "ADC-PRINT", TASK_STACK_SIZE, (void*)&handles, tskIDLE_PRIORITY, (void*) &adc_print_handle);
    
    ESP_ERROR_CHECK(adc_continuous_start(handle));  //start the adc conversions
    adc_state = true;

    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // avoid watchdog.
        ESP_LOGI(MAIN_TAG, "adc_state: %d", adc_state);
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
