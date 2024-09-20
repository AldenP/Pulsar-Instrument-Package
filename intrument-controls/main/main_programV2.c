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
#include "driver/gptimer.h"             // for button debouncing
// #include "driver/mcpwm.h"               // Motor PWM control for servos
#include "driver/pulse_cnt.h"                // (pulse counter) for rotary encoder 
#include "lcd1602/lcd1602.h"            // LCD library

// #include "driver/uart.h"                // UART communication for waveplate rotator
#include "rotator_driver.h"             // Waveplate rotator driver functions

//      ----- Pin Assignments -----
#define START_BUT       GPIO_NUM_35     // starts the whole process
#define AUX_BUT         GPIO_NUM_34     // more of Auxilary button (AUX)

#define ROTARY_A        GPIO_NUM_32     // one of the rotary encoder/dial's inputs
#define ROTARY_B        GPIO_NUM_33
#define ROTARY_SWITCH   GPIO_NUM_25      // switch/press of dial

#define LCD_SDA         GPIO_NUM_10     // SDA pin for i2c LCD (20x04) [green wire]
#define LCD_SCL         GPIO_NUM_9      // SCL pin for i2c LCD

#define SERVO_1         GPIO_NUM_26     // servo 1 for 1st fold mirror
#define SERVO_2         GPIO_NUM_27     // servo 2 for 2nd fold mirror

// TX/RX arbitarily chosen - defined in rotator_driver.h
// #define WAVEPLATE_TX    GPIO_NUM_21     // TX for UART to control Waveplate Rotator
// #define WAVEPLATE_RX    GPIO_NUM_22     // RX for UART to control Waveplate Rotator

#define GREEN_LED       GPIO_NUM_0      // General debug LED (button presses)
#define RED_LED         GPIO_NUM_5      // ADC on/off status LED
#define BLUE_LED        GPIO_NUM_18     // ADC overflow LED

// Convient Masks for inputs and outputs when configuraing GPIO.
#define GPIO_IN_MASK (1ULL << START_BUT | 1ULL << AUX_BUT | 1ULL << ROTARY_SWITCH)
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
#define ROTARY_HIGH         4
#define ROTARY_LOW          -4

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

static TaskHandle_t adc_copy_handle = NULL;
static const char *ADC_PRINT_TAG = "ADC-COPY";

static TaskHandle_t adc_task_handle = NULL;
static const char * ADC_TASK_TAG = "ADC-TASK";

static TaskHandle_t lcd_task_handle = NULL;
static const char * LCD_TASK_TAG = "LCD-TASK";

static const char * PY_TAG = "BEGIN PY READ";
static const char * PY_END_TAG = "END PY SAMPLE";

// ----- STATE VARIABLES -----
// volatile if value can change within an ISR context
volatile bool adc_state = false; 
volatile bool led_state = false;
volatile bool red_led_state = false;
volatile bool blue_led_state = false;
volatile bool pupil_path_state = false;     // essentially the state of the servos
// --- menu variables ---
volatile uint16_t base_pos = 0;     // index of base array. shared between duration and frequency. reset to zero as needed.
volatile uint16_t lcd_curPos = 0;   // column position, based on base pos.

static char *menu[] = {"Sample Frequency:", "Sample Duration:", "Pupil Image?"};  //17, 16, 12 chars
static size_t menuSize = 3;
static char* BLANK_LINE = "                    ";   //20 spaces/characters
volatile int menuIndex = 1;  //change with 'stop button' (aux button) - make it volatile
int posMax[] = {5, 3, 0};  //freq, dur, pupil - inclusive
int posMin[] = {3, 0, 0};  //freq, dur, pupil - inclusive
int base[] = {1, 10, 100, 1000, 10000, 100000}; // for modifying the duration/frequency
size_t baseSize = sizeof(base) / sizeof(int);
static const char *menuUnit[] = { "kHz", "ms", ""}; //third unit is for a yes/no value.
static const char *servoStatus[] = {"OFF", " ON"};  //default is off, turn on with rot_switch when on this menu.
// --- ---
uint8_t adc_conv_buffer[ADC_BUFFER_LEN] = {0};   // result buffer for adc_copy_task
// ----- -----
// --- Parameters ---
uint32_t sample_frequency = DEFAULT_ADC_FREQ;    // default 1MHz frequency
uint32_t sample_duration = 250;         // default 250ms 
uint32_t bytes_read = 0;   // no longer changes in ISR context, removed volatile
// --- ---

// ----- LOCKS -----
// for concurrency issues, spinlocks should be used (easiest solution)
// static portMUX_TYPE adc_lock = portMUX_INITIALIZER_UNLOCKED;  //spinlock for adc_state
static portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;  //lock for sample parameters, and menu index

// ----- STRUCTURES -----
// structure to hold any handles (as pointers) that may be needed to be sent to functions
typedef struct my_handlers_t{
    adc_continuous_handle_t *adc_handle;
    gptimer_handle_t *gptimer;
} my_handlers_t;        
// ----- -----

// ----- ADC ISRs -----
static bool IRAM_ATTR adc_conv_ready_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    // 1st, update number of bytes read from event data.
    // bytes_read += edata->size;  
    BaseType_t mustYield = pdFALSE;
    // Notify the printing task to retrieve and output the data
    vTaskNotifyGiveFromISR(adc_copy_handle, &mustYield);
    return (mustYield == pdTRUE);
}
static bool IRAM_ATTR adc_pool_ovf_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    // set overflow LED on
    gpio_set_level(BLUE_LED, 1);
    blue_led_state = 1;
    //returns if a higher priority task is woken up...
    return pdFALSE; // I don't think any high-priority tasks woke up from this function...
}
// ----- -----
// Initializes ADC with desired frequency.
static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle) {
    // adc initialization function: first make a handle, then configure dig controller
    adc_continuous_handle_t handle = NULL;  // ? shouldn't this be malloc if we're outputting it to `out_handle`

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 16 * ADC_BUFFER_LEN,
        .conv_frame_size = ADC_BUFFER_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    taskENTER_CRITICAL(&param_lock);    // protect access to sample frequency
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = sample_frequency,    // frequency of conversion controlled by global value
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };
    taskEXIT_CRITICAL(&param_lock);

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

// ----- ISRs -----
static void IRAM_ATTR start_isr_handler(void* arg) {
    // give notice to adc task, which initializes the adc module with sample frequency, and runs until all data read.
        // get ADC handler from arg
    my_handlers_t *handles = (my_handlers_t*) arg;
    adc_continuous_handle_t *adc_handle = handles->adc_handle;
    // debounce scheme first.
    gptimer_handle_t *t_handle = handles->gptimer;
    gpio_intr_disable(ROTARY_SWITCH);
    // gptimer_set_raw_count(*t_handle, 0);
    gptimer_start(*t_handle);

    // provide visual output of pressing button.
    led_state = !led_state;
    gpio_set_level(GREEN_LED, led_state);
    
    // esp_err_t ret;
    BaseType_t mustYield = pdFALSE;
    // first see if ADC is off, and check that handle is null (both should follow each other)
    if (!adc_state && (*adc_handle == NULL)) {
        // notify the adc task to initialize adc module, and start sampling.
        vTaskNotifyGiveFromISR(adc_task_handle, &mustYield);
    }   //else it is already running, don't do anything.
    // void type doesn't need return...
}
static void IRAM_ATTR aux_isr_handler(void* arg) {
    // debounce scheme first.
    gptimer_handle_t *t_handle = ((my_handlers_t*)arg)->gptimer;
    gpio_intr_disable(ROTARY_SWITCH);
    // gptimer_set_raw_count(*t_handle, 0);
    gptimer_start(*t_handle);
    // provide visual output of button press
    led_state = !led_state;
    gpio_set_level(GREEN_LED, led_state);   // to be ISR/IRAM safe, check config editor under GPIO.

    // AUX functions: change menu shown on LCD. If in pupil state, do not change menu (no sampling while pupil imaging)?
    taskENTER_CRITICAL_ISR(&param_lock);    //just in case another task goes to access menuIndex.
    menuIndex += 1;
    if (menuIndex >= menuSize)
        menuIndex = 0;
    base_pos = posMin[menuIndex];   // make sure position isn't out of range when menu changes.
    taskEXIT_CRITICAL_ISR(&param_lock);

}
// Handler for the rotary switch interrupt
static void IRAM_ATTR rot_switch_isr_handler(void* args) {
    // Changes position/digit of sample duration/frequency
    led_state = !led_state;
    gpio_set_level(GREEN_LED, led_state);
    // debounce scheme first.
    gptimer_handle_t *t_handle = (gptimer_handle_t*) args;
    gpio_intr_disable(ROTARY_SWITCH);
    // gptimer_set_raw_count(*t_handle, 0);
    gptimer_start(*t_handle);
    // now update position
    // if on pupil imaging menu, pressing dial will change it's state
    if (menuIndex == 2) {
        // pupil_path_state = (pupil_path_state) ? false: true;    // flip value in pupil path.
        pupil_path_state = !pupil_path_state;   // does the same
    } else {
        // update position when not in pupil menu. 
        taskENTER_CRITICAL_ISR(&param_lock);    //don't want contention between locks, so maybe make another lock for pos. ?
        base_pos += 1;
        if (base_pos > posMax[menuIndex])   // base_pos can equal posMax[menuIndex]. 
            base_pos = posMin[menuIndex];
        taskEXIT_CRITICAL_ISR(&param_lock);
        // lcd_curPos +=1;  //variable unused!
    }
}
// ISR when pulse counter hits a watchpoint.
static bool IRAM_ATTR on_pcnt_watch(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_data) {
    // Change variable value
    // 'debounce' this interrupt? 
    BaseType_t higherWoken = pdFALSE;
    // send event data to main task. Perhaps off load above if it still doesn't work. 
    xQueueSendFromISR((QueueHandle_t) user_data, &(edata->watch_point_value), &higherWoken);
    return (higherWoken == pdTRUE); // 
}
static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t handle, const gptimer_alarm_event_data_t *edata, void* user_data) {
    // re-enable button interrupts, after stopping the timer
    gptimer_stop(handle);
    gpio_intr_enable(ROTARY_SWITCH);
    gpio_intr_enable(START_BUT);
    gpio_intr_enable(AUX_BUT);
    
    return pdFALSE; // just assume a higher priority task wasn't awoken. proper implementation could use a notification to main task? 
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
        if (pupil_path_state) {
            // flowchart has this constraint. technically data can still be acquired, but tracking would not have proper image
            ESP_LOGW(ADC_TASK_TAG, "Pupil imaging path on, cannot start ADC");
            continue;   // will go wait again
        }
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

        // FUTURE: start waveplate. Abort if status not OKAY
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
        // inform python that this sampling period has ended
        printf("%s\n", PY_END_TAG);
        // FUTURE: stop waveplate
        ESP_ERROR_CHECK(adc_continuous_deinit(*adc_handle));
        ESP_LOGI(ADC_TASK_TAG, "ADC deinitialized.");
        *adc_handle = NULL; //reset back to null

        ESP_LOGI(ADC_TASK_TAG, "Resuming main task (logging)");
        vTaskResume(main_task_handle);
    }
}
// Task to handle adc conversion result, whether it prints to python, or saves to SD card.
static void adc_copy_task(void* args) {
    // Will need the adc_handle for reading the data. 
    my_handlers_t * handles = (my_handlers_t*) args;
    adc_continuous_handle_t *adc_handle = handles->adc_handle;  //it is a pointer, de-reference as needed.

    // uint32_t byte_count = 0; //moved to global => bytes_read

    esp_err_t ret;
    uint32_t ret_num = 0;
    memset(adc_conv_buffer, 0xcc, ADC_BUFFER_LEN);   // clear the result buffer

    ESP_LOGI(ADC_PRINT_TAG, "adc_copy_task created and initialized");
    for(;;) {
        ESP_LOGI(ADC_PRINT_TAG, "Waiting for ADC Conversion Frame ...");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // wait for a conversion result to be done
        // determine the number of bytes to read.
        uint32_t bytes_to_read = (uint32_t)(2*num_adc_channels * (sample_duration/1000.0) * sample_frequency);    //may give float... cast to int, or round up 
        
        char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);   //not used (except in an else)

        while (1) {
            ret = adc_continuous_read(*adc_handle, adc_conv_buffer, ADC_BUFFER_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                ESP_LOGI(ADC_PRINT_TAG, "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
                printf("%s\n", PY_TAG);   // display tag for python.
                printf("Number of Bytes: %"PRIu32"\n", ret_num);    //pass over number of bytes/lines to read
                // adc_task will suspend any tasks that log. 
                // vTaskSuspendAll();  //suspends all tasks to avoid logs during printing to python (alternatively,  just suppress main)
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_conv_buffer[i];
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
                // vTaskSuspend(adc_copy_handle); 
                vTaskDelay(1000/portTICK_PERIOD_MS);
                break;
            }
        }
    }
}
// Task that updates the lcd periodically
static void lcd_task(void* args) {
    // get the lcd contex from args
    lcd1602_context lcd_ctx = (lcd1602_context) args; //lcd1602_context is a void* type. it gets casted to an lcd1602_t later. 
    lcd1602_clear(lcd_ctx);
    char strBuf[41] = {' '};    // buffer for formatting numbers into a string for the LCD
    size_t buf_size = sizeof(strBuf) / sizeof(char);
    ESP_LOGI(LCD_TASK_TAG, "LCD task started successfully");
    ESP_LOGI(LCD_TASK_TAG, "strBuf: %s; ", strBuf);

    while(1) {
        // change menu based on menu index
        if (menuIndex == 0) {           //frequency
            lcd1602_set_cursor(lcd_ctx, 0, 1);
            lcd1602_string(lcd_ctx, menu[menuIndex]);
            // format numerical data into string
            memset(strBuf, 32, buf_size);    // clear string (with spaces), then format it
            snprintf(strBuf, buf_size, "  %4ld %s", sample_frequency / 1000, menuUnit[menuIndex]);  //be cognizant if no null-termination

            lcd1602_set_cursor(lcd_ctx, 1, 2);
            lcd1602_string(lcd_ctx, strBuf);
            // reposition cursor to current unit position
            lcd1602_set_cursor(lcd_ctx, 1, (5-base_pos));
        } else if (menuIndex == 1) {    //duration
            lcd1602_set_cursor(lcd_ctx, 0, 1);
            lcd1602_string(lcd_ctx, menu[menuIndex]);
            // format numerical data into string
            memset(strBuf, 32, buf_size);    // clear string, then format it
            snprintf(strBuf, buf_size, "  %4ld %s", sample_duration, menuUnit[menuIndex]);

            lcd1602_set_cursor(lcd_ctx, 1, 2);
            lcd1602_string(lcd_ctx, strBuf);
            // reposition cursor to current unit position
            lcd1602_set_cursor(lcd_ctx, 1, (5-base_pos));
        } else if (menuIndex == 2) {    //pupil image?
            lcd1602_set_cursor(lcd_ctx, 0, 1);
            lcd1602_string(lcd_ctx, menu[menuIndex]);
            int pos;
            if (pupil_path_state)   pos = 1;
            else    pos = 0;
            // format numerical data into string
            memset(strBuf, 32, buf_size);    // clear string, then format it
            snprintf(strBuf, buf_size, "  %3s %s", servoStatus[pos], menuUnit[menuIndex]);

            lcd1602_set_cursor(lcd_ctx, 1, 2);
            lcd1602_string(lcd_ctx, strBuf);   // display formatted string to LCD
            // reposition cursor to current unit position
            lcd1602_set_cursor(lcd_ctx, 1, (5-base_pos));
        }
        // lcd1602_set_display(lcd_ctx, true, true, true);
        // lcd1602_set_mode(lcd_ctx, false, false);
        
        ESP_LOGI(LCD_TASK_TAG, "strBuf: %s", strBuf);
        // if the 3rd and 4th lines are used later, the above will write over it (maybe). Soln: reduce size of strBuf.
        // if (adc_state) => print "ADC running" or "Sampling", else "ADC off"/"Standby"
        // wait a reasonable time before refreshing/updating LCD
        vTaskDelay(250/portTICK_PERIOD_MS); // 250ms delay
    }
}

void app_main(void) {
    // esp_err_t ret;  // var to hold return values
    main_task_handle = xTaskGetCurrentTaskHandle();     // get task handle for main
    esp_log_level_set(MAIN_TAG, ESP_LOG_DEBUG);
    // ---- gpio setup ----
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
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;    //external pullups implemented

    gpio_config(&io_conf);
    // set initial values of LEDs to 'off'
    gpio_set_level(RED_LED, 0);
    gpio_set_level(GREEN_LED, 0);
    gpio_set_level(BLUE_LED, 0);
    
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));    //enable ISRs to be added.
    // ----------
    // --- GPTimer / Debounce Timer ---
    #pragma region
    ESP_LOGI(MAIN_TAG, "Initializing GPTimer peripheral for debouncing");
    gptimer_handle_t timer_handle = NULL;
    gptimer_config_t timer_conf = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .intr_priority = 0,
        .resolution_hz = 10*1000,   //10kHz = 0.1ms ticks
        // .flags.intr_shared
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_conf, &timer_handle));
    // setup the alarm for timer
    gptimer_alarm_config_t timer_alarm_conf = {
        .alarm_count = 75*10,   // 75ms * 10ticks/ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true   // reloads alarm to reload count immediately after alarm. Will do this in ISR
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &timer_alarm_conf));
    // callback function
    gptimer_event_callbacks_t timer_cbs_conf =  {
        .on_alarm = on_timer_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &timer_cbs_conf, (void*)NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer_handle));
    ESP_LOGI(MAIN_TAG, "GPTimer initialized and enabled");
    #pragma endregion
    // --- Waveplate UART ---
    // use functions defined in rotator_driver.h

    // --- I2C LCD ---
    #pragma region 
    ESP_LOGI(MAIN_TAG, "Setting up I2C bus for LCD");
    i2c_master_bus_handle_t i2c_master_bus = {0};
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, //common value for this
        .i2c_port = I2C_NUM_0,  //uses one of 2 internal i2c ports
        .sda_io_num = LCD_SDA,
        .scl_io_num = LCD_SCL,
        .flags.enable_internal_pullup = true,   //not suitable for high frequency. External pullup rec. in that case
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_master_bus));

    i2c_lowlevel_config lowConf = {
        .bus = &i2c_master_bus
    };
    // Example uses a pointer for the handle
    lcd1602_context lcd_ctx = lcd1602_init(0x27, true, &lowConf);
    // clear lcd and print hello world! in center
    // lcd1602_set_mode(lcd_ctx, true, false);    // L->R will move cursor left to right when updating.
    lcd1602_reset(lcd_ctx);     // tabula rasa  - blank slate! nope. in code it runs set_mode with L->R true...
    lcd1602_clear(lcd_ctx);   //part of init
    // lcd1602_set_backlight(lcd_ctx, true);   // in case reset makes this false.
    // lcd1602_set_display(lcd_ctx, true, true, true);    //cursor enabled for rotary encoder, optionally blink

    lcd1602_home(lcd_ctx);
    lcd1602_string(lcd_ctx, "   Hello World!!!   From ESP32");
    // line 3 = from line 1, it is col 20-39.
    // lcd1602_set_cursor(lcd_ctx, 0, 19);
    // lcd1602_string(lcd_ctx, " 0,20 Line 3");   // positioned correctly.
    // line 4 = from line 2, it is col 20-39
    lcd1602_set_cursor(lcd_ctx, 1, 19);
    lcd1602_string(lcd_ctx, " 1,20 Line 4");

    lcd1602_set_display(lcd_ctx, true, true, true);    //cursor enabled for rotary encoder, optionally blink
    lcd1602_set_mode(lcd_ctx, false, false);
    vTaskDelay(1000/portTICK_PERIOD_MS);    // let the hello world display for a second.
    // interestingly, it doesn't scroll until later...
    #pragma endregion
    // --- Rotary Encoder ---
    #pragma region
    ESP_LOGI(MAIN_TAG, "Installing pcnt (pulse count) unit for dial");  //peripheral for the rotary encoder dial
    pcnt_unit_config_t unit_config = {
        .high_limit = ROTARY_HIGH,
        .low_limit = ROTARY_LOW,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(MAIN_TAG, "PCNT: setting glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(MAIN_TAG, "Installing pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ROTARY_A,      
        .level_gpio_num = ROTARY_B,   //use -1 if not used
    };  //Channel A: Watches edge of 'A', and level of 'B'.
    pcnt_channel_handle_t pcnt_chan_a = NULL;   //channel for 1 direction
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ROTARY_B,
        .level_gpio_num = ROTARY_A,
    };  //Channel B: watches edge of 'B', and level of 'A'. 
    pcnt_channel_handle_t pcnt_chan_b = NULL;   //channel for other direction
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    
    ESP_LOGI(MAIN_TAG, "PCNT: setting edge actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(MAIN_TAG, "PCNT: add watch points and register callbacks");
    int watch_points[] = {ROTARY_LOW, ROTARY_HIGH}; //zero watchpoint adds extra event. 
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t pcnt_cbs = {
        .on_reach = on_pcnt_watch,
    };
    QueueHandle_t pcnt_queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &pcnt_cbs, pcnt_queue));

    ESP_LOGI(MAIN_TAG, "Enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(MAIN_TAG, "Clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(MAIN_TAG, "Start pcnt unit");  // well that would do it!
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    // Counts for rotary encoder
    int pulse_count = 0;
    int event_count = 0;    //passed by reference to xQueueReceive
    #pragma endregion
    // --- MCPWM ---

    adc_continuous_handle_t handle = NULL;  //needed in many places, hold it here for distribution.

    my_handlers_t handles = {
        .adc_handle = &handle,
        .gptimer = &timer_handle,
    };
    // Install GPIO ISRs
    gpio_isr_handler_add(START_BUT, start_isr_handler, (void*)&handles);
    gpio_isr_handler_add(AUX_BUT, aux_isr_handler, (void*)&handles);
    gpio_isr_handler_add(ROTARY_SWITCH, rot_switch_isr_handler, (void*)&timer_handle);

    // register tasks
    xTaskCreatePinnedToCore(adc_task, "ADC-TASK", TASK_STACK_SIZE, (void*)&handles,  tskIDLE_PRIORITY, (void*)&adc_task_handle, 0);    
    // pin printing task to core 1, and limit number of tasks on that core. 
    xTaskCreatePinnedToCore(adc_copy_task, "ADC-COPY", TASK_STACK_SIZE, (void*)&handles, tskIDLE_PRIORITY, (void*) &adc_copy_handle, tskNO_AFFINITY);
    // vTaskSuspend(adc_copy_task);
    xTaskCreate(lcd_task, "LCD-TASK", TASK_STACK_SIZE, lcd_ctx, tskIDLE_PRIORITY, &lcd_task_handle);
    
    // Use this task (main) to monitor variables as debug level log. 
    while(1) {
        // 1000ms = 1s delay
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // avoid watchdog and spamming of logs.
        ESP_LOGD(MAIN_TAG, "adc_state: %d", adc_state);
        // ESP_LOGD(MAIN_TAG, "servo state: %d", pupil_path_state);
        ESP_LOGD(MAIN_TAG, "bytes_read: %"PRIu32"", bytes_read);
        ESP_LOGD(MAIN_TAG, "menuIndex: %d", menuIndex);
        ESP_LOGD(MAIN_TAG, "base_pos: %"PRIu16"", base_pos);
        // report PCNT count
        if (xQueueReceive(pcnt_queue, &event_count, 0)) {
            ESP_LOGI(MAIN_TAG, "PCNT Event Count: %d", event_count);
            // adjust the relevent parameter
            taskENTER_CRITICAL(&param_lock);    // protect sample params from concurrency issues! if this is problem, relocate closer
            if (menuIndex == 0) {
                if (event_count > 0)
                    sample_frequency += base[base_pos];
                else    //includes if event count == 0 !
                    sample_frequency -= base[base_pos];
            } else if (menuIndex == 1) {
                if (event_count > 0)
                    sample_duration += base[base_pos];
                else
                    sample_duration -= base[base_pos];
            } // else if (menuIndex == 2) {
                // use the rotary switch button for changing the pupil imaging.
                // if ()
            // }
            taskEXIT_CRITICAL(&param_lock);
        } // else  {  // else it waits anyway.
        //     ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
        //     ESP_LOGI(MAIN_TAG, "PCNT Current Count: %d", pulse_count);
        // }
    }
    lcd1602_deinit(lcd_ctx);
    // ESP_ERROR_CHECK(adc_continuous_stop(handle));    //this is done in the adc-task
    // ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
