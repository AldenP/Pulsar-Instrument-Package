/*  Summer-Fall 2024 Senior Design Group 3
 *  Embedded Programming Restart
 *  David A Patenaude
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG   // permits DEBUG level logging in this file.
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
// includes for this project
#include "esp_adc/adc_continuous.h"     // for ADC in continuous mode
#include "driver/gpio.h"                // for gpio's
// #include "driver/ledc.h"                // LED PWM control for servos
// #include "driver/pcnt.h"                // (pulse counter) for rotary encoder 
// #include "driver/uart.h"                // UART communication for waveplate rotator


//      ----- Pin Assignments -----
#define START_BUT       GPIO_NUM_35     // starts the whole process
#define AUX_BUT        GPIO_NUM_34     // more of Auxilary button (AUX)

#define ROTARY_A        GPIO_NUM_32     // one of the rotary encoder/dial's inputs
#define ROTARY_B        GPIO_NUM_33
#define ROTARY_SWITCH   GPIO_NUM_4      // switch/press of dial

#define LCD_SDA         GPIO_NUM_10     // SDA pin for i2c LCD (20x04)
#define LCD_SCL         GPIO_NUM_9      // SCL pin for i2c LCD

#define SERVO_1         GPIO_NUM_25     // servo 1 for 1st fold mirror
#define SERVO_2         GPIO_NUM_26     // servo 2 for 2nd fold mirror

// TX/RX arbitarily chosen
#define WAVEPLATE_TX    GPIO_NUM_27     // TX for UART to control Waveplate Rotator
#define WAVEPLATE_RX    GPIO_NUM_14     // RX for UART to control Waveplate Rotator

#define GREEN_LED       GPIO_NUM_0      // General debug LED (button presses)
#define RED_LED         GPIO_NUM_2      // ADC on/off status LED
#define BLUE_LED        GPIO_NUM_15     // ADC overflow LED

// Convient Masks for inputs and outputs when configuraing GPIO.
#define GPIO_IN_MASK (1ULL << START_BUT | 1ULL << AUX_BUT)
#define GPIO_OUT_MASK (1ULL << GREEN_LED | 1ULL << RED_LED | 1ULL << BLUE_LED)

#define ESP_INTR_FLAG_DEFAULT 0     //define flag for gpio ISRs
// ----- -----

// Simplfy this section later if possible? 
#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_2_5        //FIX: set to 0dB on final implementation
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
// ----- -----

// ----- DEFINEs for Program -----
// --- Macros ---
#define KB_TO_BYTES(k)      k*1024          //to convert KB to bytes
// --- ---

// naively increase buffer size to permit longer sample duration.
#define ADC_BUFFER_LEN                  KB_TO_BYTES(8)
#define DEFAULT_ADC_FREQ                20000 // true default of 1,000,000 Hz
// ----- -----

// use channels 0-3 of ADC1 for the ESP32
static adc_channel_t channel[1] = {ADC_CHANNEL_0};//{ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3}; //don't forget size
static size_t num_adc_channels = sizeof(channel) / sizeof(adc_channel_t);

// ----- TASKS -----
// make this large enough to avoid task stack overflows. if too large, scale back, or set it per task 
#define TASK_STACK_SIZE                 KB_TO_BYTES(4)    //4kB
static TaskHandle_t main_task_handle = NULL;
static const char *MAIN_TAG = "MAIN-TASK";

static TaskHandle_t adc_print_handle = NULL;
static const char *ADC_PRINT_TAG = "ADC-PRINT";

static TaskHandle_t adc_task_handle = NULL;
static const char * ADC_TASK_TAG = "ADC-TASK";

static const char * PY_TAG = "BEGIN PY READ";

// ----- STATE VARIABLES -----
// volatile if value can change within an ISR context
volatile bool adc_state = false; 
volatile bool led_state = false;
volatile bool red_led_state = false;
volatile bool blue_led_state = false;
volatile bool pupil_path_state = false;     // essentially the state of the servos
// ----- -----
// --- Parameters ---
volatile uint32_t sample_frequency = DEFAULT_ADC_FREQ;    // default 1MHz frequency
volatile uint32_t sample_duration = 250;         // default 250ms 
volatile uint32_t bytes_read = 0;   //changes in an ISR context, make it volatile
// --- ---

// ----- LOCKS -----
// for concurrency issues, spinlocks should be used (easiest solution)
// static portMUX_TYPE adc_lock = portMUX_INITIALIZER_UNLOCKED;  //spinlock for adc_state

// ----- STRUCTURES -----
// structure to hold any handles (as pointers) that may be needed to be sent to functions
typedef struct my_handlers_t{
    adc_continuous_handle_t *adc_handle;
    // gptimer_handle_t *gptimer;
} my_handlers_t;        
// ----- -----

static bool IRAM_ATTR adc_conv_ready_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    // 1st, update number of bytes read from event data.
    // bytes_read += edata->size;  

    BaseType_t mustYield = pdFALSE;
    // Notify the printing task to retrieve and output the data
    vTaskNotifyGiveFromISR(adc_print_handle, &mustYield);

    return (mustYield == pdTRUE);
}
static bool IRAM_ATTR adc_pool_ovf_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    // set overflow LED on
    gpio_set_level(BLUE_LED, 1);
    blue_led_state = 1;
    //returns if a higher priority task is woken up...
    return pdFALSE; // I don't think any high-priority tasks woke up from this function...
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle) {
    // adc initialization function: first make a handle, then configure dig controller
    adc_continuous_handle_t handle = NULL;  // ? shouldn't this be malloc if we're outputting it to `out_handle`

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 8 * ADC_BUFFER_LEN,
        .conv_frame_size = ADC_BUFFER_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = sample_frequency,    // frequency of conversion controlled by global value
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;  //takes lower 3 bits (channel is uint8 - only 8 bits)
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(ADC_TASK_TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(ADC_TASK_TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(ADC_TASK_TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static void IRAM_ATTR start_isr_handler(void* arg) {
    // give notice to adc task, which initializes the adc module with sample frequency, and runs until all data read.
    // provide visual output of pressing button.
    led_state = !led_state;
    gpio_set_level(GREEN_LED, led_state);
    
    // get ADC handler from arg
    my_handlers_t *handles = (my_handlers_t*) arg;
    adc_continuous_handle_t *adc_handle = handles->adc_handle;

    // esp_err_t ret;
    BaseType_t mustYield = pdFALSE;
    // first see if ADC is off, and check that handle is null (both should follow each other)
    if (!adc_state && (*adc_handle == NULL)) {
        // notify the adc task to initialize adc module, and start sampling.
        vTaskNotifyGiveFromISR(adc_task_handle, &mustYield);
    }   //else it is already running, don't do anything.
}
static void IRAM_ATTR aux_isr_handler(void* arg) {
    // stop the continuous ADC reading. 
    // provide visual output
    led_state = !led_state;
    gpio_set_level(GREEN_LED, led_state);   // to be ISR/IRAM safe, check config editor under GPIO.

    // // get ADC handler from arg
    // my_handlers_t *handles = (my_handlers_t*) arg;
    // adc_continuous_handle_t *adc_handle = handles->adc_handle;
    // AUX functions: change menu shown on LCD. 

}

// Task to initialize adc with desired parameters, run the adc, delegate printing to other task, and when done, release resources.
static void adc_task(void* args) {
    // maybe useful to pin this task to opposite core that printing task will be on.
    ESP_LOGI(ADC_TASK_TAG, "ADC Task Created");
    my_handlers_t *myHandles = (my_handlers_t*) args;
    adc_continuous_handle_t *adc_handle = myHandles->adc_handle;

    while(1) {
        ESP_LOGI(ADC_TASK_TAG, "ADC Task Waiting");
        // wait until start button is pressed. 
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(ADC_TASK_TAG, "Beginning ADC setup...");
        ESP_LOGI(ADC_TASK_TAG, "Suspending main task (logging)");
        vTaskSuspend(main_task_handle);

        // start by getting some variables designed.
        *adc_handle = NULL; // set handle to NULL. 
        // run init function, global frequency variable will set adc frequency.
        continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), adc_handle);  //adc_handle is pointer itself
        ESP_LOGI(ADC_TASK_TAG, "Handle initialized. (waits 2 seconds)");
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        bytes_read = 0; //reset bytes read to zero.
        adc_continuous_evt_cbs_t cbs = {
            .on_conv_done = adc_conv_ready_cb,  //callback when conversion result is ready.
            .on_pool_ovf = adc_pool_ovf_cb,     //callback when conversion pool overflows.
        };
        ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(*adc_handle, &cbs, NULL));
        // adc handle has been configured and initialized
        
        gpio_set_level(BLUE_LED, 0);    // turn overflow LED off
        blue_led_state = 0;

        // FUTURE: start waveplate
        // give a 3 second countdown. 
        for (int16_t i = 3; i >= 1; i--) {
            ESP_LOGI(ADC_TASK_TAG, "Starting ADC...(%"PRId16")", i);
            vTaskDelay(1000/portTICK_PERIOD_MS);    //give a little delay
        }
        ESP_ERROR_CHECK(adc_continuous_start(*adc_handle));
        adc_state = true;   //does this need a critical section? 
        gpio_set_level(RED_LED, adc_state); // turn LED on

        // print task will be notified by adc_conv_ready_cb. Update byte count from ISR. wait here until resumed (after set bytes read)
        ESP_LOGI(ADC_TASK_TAG, "Suspending task (waiting to finish reading).");
        vTaskSuspend(NULL);
        
        // on resume, stop adc and deinit to free resources.
        ESP_ERROR_CHECK(adc_continuous_stop(*adc_handle));
        adc_state = false;  //update state var. 
        gpio_set_level(RED_LED, adc_state); // turn LED off. could just hard code 0. 
        
        // FUTURE: stop waveplate
        ESP_ERROR_CHECK(adc_continuous_deinit(*adc_handle));
        ESP_LOGI(ADC_TASK_TAG, "ADC deinitialized.");
        *adc_handle = NULL; //reset back to null

        ESP_LOGI(ADC_TASK_TAG, "Resuming main task (logging)");
        vTaskResume(main_task_handle);
    }
}

static void adc_print_task(void* args) {
    // Will need the adc_handle for reading the data. 
    my_handlers_t * handles = (my_handlers_t*) args;
    adc_continuous_handle_t *adc_handle = handles->adc_handle;  //it is a pointer, de-reference as needed.

    // uint32_t byte_count = 0; //moved to global => bytes_read

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[ADC_BUFFER_LEN] = {0};   //uint8 used because it is 1 byte. 
    memset(result, 0xcc, ADC_BUFFER_LEN);   

    ESP_LOGI(ADC_PRINT_TAG, "adc_print_task created and initialized");
    for(;;) {
        ESP_LOGI(ADC_PRINT_TAG, "Waiting for ADC Conversion Frame ...");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // wait for a conversion result to be done
        // determine the number of bytes to read.
        uint32_t bytes_to_read = (uint32_t)(2*num_adc_channels * (sample_duration/1000.0) * sample_frequency);    //may give float... cast to int, or round up 
        
        char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);   //not used (except in an else)

        while (1) {
            ret = adc_continuous_read(*adc_handle, result, ADC_BUFFER_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                ESP_LOGI(ADC_PRINT_TAG, "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
                printf("%s", PY_TAG);   // display tag for python.
                // adc_task will suspend any tasks that log. 
                // vTaskSuspendAll();  //suspends all tasks to avoid logs during printing to python (alternatively,  just suppress main)
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p); //uint32_t is much bigger than necessary
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                    /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                    if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                        // ESP_LOGI(MAIN_TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
                        // ideally, we can block other prints so that this is the only thing printing
                        
                        // printf("ch: %"PRIu32"; value: %"PRIx32"\n", chan_num, data); 

                        // print as few characters as possible. would like it to be 'raw' binary data
                        printf("%"PRIu32", %"PRIx32"\n", chan_num, data);
                        // fwrite();    // write a buffer of data to stdout stream. 
                    } else {
                        ESP_LOGW(ADC_PRINT_TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                    }
                }
                bytes_read += ret_num;
                // check if we read enough bytes.
                if (bytes_read >= bytes_to_read) {
                    // resume ADC task to stop ADC and clean up
                    ESP_LOGI(ADC_PRINT_TAG, "total bytes read: %"PRIu32"", bytes_read);
                    ESP_LOGI(ADC_PRINT_TAG, "sample bytes to read: %"PRIu32"", bytes_to_read);
                    vTaskResume(adc_task_handle);
                    vTaskDelay(1000/portTICK_PERIOD_MS); //give adc_task some time before breaking and waiting above
                    break;  // exit the inner while loop to wait for notify by adc_conv_done_cb
                }
                // xTaskResumeAll();   //resume all the other tasks, this one may now be preempted.
                // vTaskResume(main_task_handle);
                vTaskDelay(1);  // 1tick block to avoid watchdog timer.
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
            else {  //catch other return values and print it
                ESP_LOGW(ADC_PRINT_TAG, "bad ret value %d", ret);
                // vTaskSuspend(adc_print_handle); 
                vTaskDelay(1000/portTICK_PERIOD_MS);
                break;
            }
        }
    }
}

void app_main(void) {
    // esp_err_t ret;  // var to hold return values
    main_task_handle = xTaskGetCurrentTaskHandle();     // get task handle for main
    esp_log_level_set(MAIN_TAG, ESP_LOG_DEBUG);
    //---- gpio setup ----
    // -- gpio inputs
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_IN_MASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    gpio_config(&io_conf);
    // -- gpio outputs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUT_MASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    gpio_config(&io_conf);
    // set initial values of LEDs to 'off'
    gpio_set_level(RED_LED, 0);
    gpio_set_level(GREEN_LED, 0);
    gpio_set_level(BLUE_LED, 0);
    
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));    //enable ISRs to be added.
    // ----------
    adc_continuous_handle_t handle = NULL;  //needed in many places, hold it here for distribution.

    my_handlers_t handles = {
        .adc_handle = &handle,
    };
    // Install ISRs for start and aux. 
    gpio_isr_handler_add(START_BUT, start_isr_handler, (void*)&handles);
    gpio_isr_handler_add(AUX_BUT, aux_isr_handler, (void*)&handles);

    // register tasks
    xTaskCreatePinnedToCore(adc_task, "ADC-TASK", TASK_STACK_SIZE, (void*)&handles,  tskIDLE_PRIORITY, (void*)&adc_task_handle, 0);    
    // pin printing task to core 1, and limit number of tasks on that core. 
    xTaskCreatePinnedToCore(adc_print_task, "ADC-PRINT", TASK_STACK_SIZE * 8, (void*)&handles, tskIDLE_PRIORITY, (void*) &adc_print_handle, 1);
    // vTaskSuspend(adc_print_task);

    // Use this to monitor variables as debug level log. 
    while(1) {
        // 1000ms = 1s delay
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // avoid watchdog and spamming of logs.
        ESP_LOGD(MAIN_TAG, "adc_state: %d", adc_state);
        // ESP_LOGD(MAIN_TAG, "servo state: %d", pupil_path_state);
        ESP_LOGD(MAIN_TAG, "bytes_read: %"PRIu32"", bytes_read);
    }

    // ESP_ERROR_CHECK(adc_continuous_stop(handle));    //this is done in the adc-task
    // ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
