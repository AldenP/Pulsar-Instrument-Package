/*  Summer-Fall 2024 Senior Design Group 3
 *  Embedded Programming Restart
 *  David A Patenaude
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#define LOG_LOCAL_LEVEL         ESP_LOG_DEBUG   // permits DEBUG level logging in this file.
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// includes for this project
#include "esp_adc/adc_continuous.h"     // for ADC in continuous mode
#include "driver/gpio.h"                // for gpio's
#include "driver/gptimer.h"             // for button debouncing
#include "driver/mcpwm_prelude.h"       // Motor PWM control for servos
#include "driver/pulse_cnt.h"           // (pulse counter) for rotary encoder 
#include "lcd1602/lcd1602.h"            // LCD library
#include "driver/uart.h"                // UART communication for waveplate rotator
#include "waveplate_uart.h"             // Functions to setup and communicate with rotator mount
// #include "rotator_driver.h"             // Waveplate rotator driver functions
// SD card includes
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

//      ----- Pin Assignments -----
#define START_BUT       GPIO_NUM_35     // starts the whole process
#define AUX_BUT         GPIO_NUM_34     // more of Auxilary button (AUX)

#define ROTARY_A        GPIO_NUM_33     // one of the rotary encoder/dial's inputs. left is B, right is A to satisfy below.
#define ROTARY_B        GPIO_NUM_32     // switch for proper directional increments (clock wise = + increase)
#define ROTARY_SWITCH   GPIO_NUM_25      // switch/press of dial

#define LCD_SDA         GPIO_NUM_10     // SDA pin for i2c LCD (20x04) [green wire]
#define LCD_SCL         GPIO_NUM_9      // SCL pin for i2c LCD

#define SERVO_1         GPIO_NUM_26     // servo 1 for 1st fold mirror  (green wire) 
#define SERVO_2         GPIO_NUM_27     // servo 2 for 2nd fold mirror  (blue wire)

// TX/RX arbitarily chosen - defined in rotator_driver.h / waveplate_uart.h
// #define WAVEPLATE_TX    GPIO_NUM_21     // TX for UART to control Waveplate Rotator
// #define WAVEPLATE_RX    GPIO_NUM_22     // RX for UART to control Waveplate Rotator

// SD card pins - also 3.3V and GND connection. (3.3V is important)
#define SD_CMD          GPIO_NUM_15     // brown wire
#define SD_CLK          GPIO_NUM_14     // white wire
#define SD_DETECT       GPIO_NUM_23     // gray wire - when high, SD card is inserted. Hardwire an LED --> loading effect drops voltage to ~1.7V
// --- Data Lines ---
#define SD_DAT0         GPIO_NUM_2      // blue wire
#define SD_DAT1         GPIO_NUM_4      // green wire
#define SD_DAT2         GPIO_NUM_12     // yellow wire
#define SD_DAT3         GPIO_NUM_13     // orange wire
// - SD Params -
#define SD_LINE_WIDTH   4               // want 4-line/bit width, but doesn't work, so 1-line it is (for now)
#define SD_FREQUENCY    SDMMC_FREQ_HIGHSPEED    // 40MHz fastest, default is 20MHz, and probing (slowest) is 400kHz
// --- LEDs ---
#define GREEN_LED       GPIO_NUM_0      // General debug LED (button presses)
#define RED_LED         GPIO_NUM_5      // ADC on/off status LED
#define BLUE_LED        GPIO_NUM_18     // ADC overflow LED

// Convient Masks for inputs and outputs when configuraing GPIO.
#define GPIO_IN_MASK    (1ULL << START_BUT | 1ULL << AUX_BUT | 1ULL << ROTARY_SWITCH) // | 1ULL << SD_DETECT)
#define GPIO_OUT_MASK   (1ULL << GREEN_LED | 1ULL << RED_LED | 1ULL << BLUE_LED)

#define ESP_INTR_FLAG_DEFAULT 0     //define flag for gpio ISRs
// ----- -----

// Simplfy this section later, if possible? 
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
#define DEBOUNCE_TIME_MS    300     // time to wait, in milliseconds, before re-enabling button interrupts
// naively increase buffer size to permit longer sample duration.
#define ADC_BUFFER_LEN                  KB_TO_BYTES(16)
#define DEFAULT_ADC_FREQ                20000 // true default of 1,000,000 Hz

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define SD_MOUNT "/sdcard"  //mount path for SD card
#define FORMAT_IF_MOUNT_FAILS   true    // this will format the card for the 1st time (if mounting fails).
#define USE_SD_CARD             true    // used with #ifdef to either print ADC to file (on SD card) or to python/console.
// define max # of files and unit size? 

// ----- -----

// use channels 0-3 of ADC1 for the ESP32
static adc_channel_t channel[1] = {ADC_CHANNEL_0};//{ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3}; //don't forget size
static size_t num_adc_channels = sizeof(channel) / sizeof(adc_channel_t);

// ----- TASKS -----
// make this large enough to avoid task stack overflows. if too large, scale back, or set it per task 
#define TASK_STACK_SIZE                 KB_TO_BYTES(8)    //8kB
static TaskHandle_t main_task_handle = NULL;
static const char *MAIN_TAG = "MAIN-TASK";

static TaskHandle_t adc_copy_handle = NULL; //copy to SD card
static const char *ADC_COPY_TAG = "ADC-COPY";
#define COPY_TASK_PRIORITY      5

static TaskHandle_t adc_task_handle = NULL;
static const char * ADC_TASK_TAG = "ADC-TASK";
#define ADC_TASK_PRIORITY       2

static TaskHandle_t adc_transfer_handle = NULL; // transfer from SD card to PC over UART
static const char * ADC_TRANS_TAG = "ADC-TRANSFER";
#define ADC_TRANSFER_PRIORITY   tskIDLE_PRIORITY
// static TaskHandle_t lcd_task_handle = NULL;
// static const char * LCD_TASK_TAG = "LCD-TASK";

static TaskHandle_t monitor_handle = NULL;
static const char * MONITOR_TASK_TAG = "MONITOR";
#define MONITOR_TASK_PRIORITY   tskIDLE_PRIORITY

static TaskHandle_t servo_task_handle = NULL;
static const char* SERVO_TASK_TAG = "SERVO";
#define SERVO_TASK_PRIORITY     1

static const char* SD_TAG = "SD-TRANSFER";

static const char * PY_TAG = "BEGIN PY READ";       // begin reading a segment of ADC_BUFFER_LEN
static const char * PY_END_TAG = "END PY SAMPLE";   // tag to save partial segments into 1 file 
static const char * PY_DATA = "PY METADATA";        // tag to tell python here is metadata (sample frequency and duration)

// ----- STATE VARIABLES -----
// volatile if value can change within an ISR context
volatile bool adc_state = false; 
volatile bool led_state = false;            //green (debug) LED state 
volatile bool red_led_state = false;        // the status LED
volatile bool blue_led_state = false;       // the overflow LED
volatile bool pupil_path_state = false;     // essentially the state of the servos
static int servo_angle = 0;                 // angle of servos 
// --- menu variables ---
volatile uint16_t base_pos = 0;     // index of base array. shared between duration and frequency. reset to zero as needed.
// volatile uint16_t lcd_curPos = 0;   // column position, based on base pos, dependent on menu index. 

// menu strings extended to 20 characters to overwrite any text past the ':'
static char *menu[] = {"Sample Frequency:  ", "Sample Duration:   ", "Pupil Image?       "};  //17, 16, 12 chars
static size_t menuSize = 3;
static char* BLANK_LINE = "                    ";   //20 spaces/characters
volatile int menuIndex = 1;  //change with 'stop button' (aux button) - make it volatile
int posMax[] = {5, 3, 0};  //freq, dur, pupil - inclusive
int posMin[] = {3, 0, 0};  //freq, dur, pupil - inclusive
int base[] = {1, 10, 100, 1000, 10000, 100000}; // for modifying the duration/frequency
static size_t baseSize = sizeof(base) / sizeof(int);
static const char *menuUnit[] = { "kHz", "ms ", "   "}; //third unit is for a yes/no value.
static const char *servoStatus[] = {"OFF", " ON"};  //default is off, turn on with rot_switch when on this menu.
// --- ---
uint8_t adc_conv_buffer[ADC_BUFFER_LEN] = {0};   // result buffer for adc_copy_task; should it be static?

static uint16_t sample_num = 0; // number to set directory name for samples
#define SAMPLE_LOG_DIR      "/SAMP_LOG"     // folder to hold logs for each sample
// FILE * sample_files[10];    // arary of FILE pointers (*) used to store list of files from a sample session.
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

typedef struct my_pcnt_t {
    pcnt_unit_handle_t *pcnt_handle;
    QueueHandle_t *event_queue;
} my_pcnt_t;

typedef struct file_data {
    char* file_path;
    uint32_t result_size;
    // can add other fields of data as time progresses
} file_data;
// ----- -----
static inline uint32_t example_angle_to_compare(int angle); // prototype
// ----- 
/**
 * Function that returns (via out pointer) a path for the next file to create to store a result buffer.
 * Uses a global static integer to provide a unique file number each time.
 * Short Filenames (SFN) follows 8.3 = 8-character name (dot) 3-character extension
 * (8-char per directory)
 * @param out: pointer to a character buffer where the file path will be written to.
 * @param fileNum: number at the end of file name (i.e. buffer number)
*/ 
static esp_err_t get_file_path(char * out, /*uint16_t ch_num,*/ uint16_t fileNum) {
    // static uint16_t file_num = 0;
    // esp_cpu_cycle_count_t cycles = esp_cpu_get_cycle_count();   // problem: this will change every time function is called. (can't be static either)
    // if (resetFileCount) {
    //     file_num = 0;
    // }
    // char strBuf[64] = {'\0'};
    // assign to the buffer provided (out points to the 1st array element)
    // path is /sdmcard/(cycles since reset / 100,000 [to reduce the number/path length])/(filenumber).dat
    // snprintf(out, 63, "%s/SAMPLE%02u/ch%"PRIu16"_%"PRIu16".dat", SD_MOUNT, sample_num, ch_num, fileNum); //increment after
    snprintf(out, 63, "%s/SAMPLE%02u/%08u.dat", SD_MOUNT, sample_num, fileNum); //increment after
    struct stat st;
    if (stat(out, &st) == 0) {
        // file already exists! return error, as we do not want to overwrite data.
        // this shouldn't happen unless MCU is reset and so sample_num resets to zero. Could increment sample_num until it works?
        // increment sample_num here, or where function is called? New problem would be ensuring that the folder exists.
        return ESP_OK;  // QoL FIX: prevent data from being overwritten? 
    }
    return ESP_OK;
}
/**
 * Function to open a text file (.txt) for logging information, or storing metadata.
 * @param file_path: Path of the logging file, or file to store data
 * @param data: data to append to the end of given file
 * @returns Returns either ESP_OK or ESP_FAILS if file fails to open
 */
static esp_err_t append_log_file(char *file_path, char* data) {
    FILE *f = fopen(file_path, "a");  // append mode (to end of file). Will create a file if needed
    if (f == NULL) {
        return ESP_FAIL;
    }
    // append data to file
    fprintf(f, "%s\n", data);
    fclose(f);  // close file to save
    return ESP_OK;
}
/**
 * Function that reads the next line from an already opened file. Reads character by character until LF encountered
 * @param fp: FILE pointer to a file opened with reading permission, non-binary.
 * @param out: pointer to the string output
 * @param max_length: max length of string output
 * @return  
 *  ESP_OK if line read successfully, ESP_FAIL if an error occurs (no error handling, so no return), 
 *  ESP_TIMEOUT if max_length reached (still ok),
 *  ESP_ERR_INVALID_STATE if end-of-file (EOF) is reached.
 */
static esp_err_t read_line_file(FILE* fp, char* out, size_t max_length) {
    //size_t num_read = 0;
    size_t i = 0;
    // while( (*(out+i) = fgetc(fp)) != '\n' && i++ < max_length -1) i++;    // one liner!
    for (i = 0; i < max_length -1; i++) {
        int ch = fgetc(fp);
        if (ch == '\n') {
            // stop b/c of newline. b/c using a file, file cursor keeps position.
            break;
        }   // could combine into one if statement, but if need to distinguish for debug...
        if (ch == 0)    // null char
            break;
        if (ch == EOF)  //EOF is -1
            break;
        // if current character is none of these, assign it to our string
        *(out+i) = ch;
    }
    *(out+i) = 0;   // null-termination. overwrites the newline.

    if (i >= max_length)
        return ESP_ERR_TIMEOUT;
    if (feof(fp)) 
        return ESP_ERR_INVALID_STATE;
    return ESP_OK;
}
// /**
//  * @brief Opens file given and reads contents to out buffer. Not implemented
//  * @param file_path: path of the file to read
//  * @param out: output buffer to read file too
//  */
// static esp_err_t read_file(char* file_path, char *out) {
//     FILE * f = fopen(file_path, "r");
//     if (f == NULL) {
//         return ESP_FAIL;
//     }
//     // fread(out, )
//     return ESP_OK;
// }

/**
 * Sees if a folder/directory exists at the given path. If not, creates a new directory (mkdir)
 * @param dir_path: Path to the directory for an existence check
 * @param make_dir: Boolean to create directory if it doesn't already exist
 * @returns Returns ESP_OK if directory (or file) exists. ESP_ERR_NOT_FOUND if path doesn't exist
 */
static esp_err_t check_dir(char *dir_path, bool make_dir) {
    struct stat st;
    if (stat(dir_path, &st) != 0) {
        // the path doesn't exist
        if (make_dir) {
            mkdir(dir_path, ACCESSPERMS);   // make directory, if desired
            return ESP_OK;
        }   // else return not found error
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;  // path exists
}
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
        .max_store_buf_size = 2 * ADC_BUFFER_LEN,   // be careful: can run out of mem if not careful. 
        .conv_frame_size = ADC_BUFFER_LEN,
    };
    // For debugging no_mem errors, print out how much heap memory is available
    ESP_LOGI(MAIN_TAG, "(Before) Free heap: %"PRIu32" B; Free internal heap: %"PRIu32" B", esp_get_free_heap_size(), esp_get_free_internal_heap_size());
    if (esp_log_level_get(MAIN_TAG) == ESP_LOG_VERBOSE)
        heap_caps_dump_all();   // prints all info on allocated heaps!
    
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
    
    ESP_LOGI(MAIN_TAG, "(After) Free heap: %"PRIu32" B; Free internal heap: %"PRIu32" B", esp_get_free_heap_size(), esp_get_free_internal_heap_size());
    if (esp_log_level_get(MAIN_TAG) == ESP_LOG_VERBOSE)
        heap_caps_dump_all();   // prints all info on allocated heaps!

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
    if (gpio_get_level(START_BUT) == 1) {
        return; // not the proper time to take interrupt
    }   // else continue with debounce scheme
    // get ADC handler from arg
    my_handlers_t *handles = (my_handlers_t*) arg;
    adc_continuous_handle_t *adc_handle = handles->adc_handle;
    // debounce scheme first.
    gptimer_handle_t *t_handle = handles->gptimer;
    gpio_intr_disable(START_BUT);   // logical error: had wrong GPIO tag
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
    if (gpio_get_level(AUX_BUT) == 1) {
        return;
    }   // else continue to debounce
    gptimer_handle_t *t_handle = ((my_handlers_t*)arg)->gptimer;
    gpio_intr_disable(AUX_BUT); // also had logical error (had ROTARY_SWITCH instead)
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
    if (gpio_get_level(ROTARY_SWITCH) == 1) {
        return; //        
    }   // (else) when the level is low, then debounce
    led_state = !led_state; 
    gpio_set_level(GREEN_LED, led_state);   // visually indicated button push registered
    // debounce scheme first. (seems to need a long timer. )
    gptimer_handle_t *t_handle = (gptimer_handle_t*) args;  //get timer handle from args
    gpio_intr_disable(ROTARY_SWITCH);   // prevent other interrupts
    // gptimer_set_raw_count(*t_handle, 0);
    gptimer_start(*t_handle);
    
    // now update position
    // if on pupil imaging menu, pressing dial will change it's state
    BaseType_t mustYield = pdFALSE;
    if (menuIndex == 2) {
        // pupil_path_state = (pupil_path_state) ? false: true;    // flip value in pupil path.
        pupil_path_state = !pupil_path_state;   // does the same
        // update servo position, but can't do a delay here... just notify task. 
        vTaskNotifyGiveFromISR(servo_task_handle, &mustYield);
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
// ----- Task Functions -----
// Task to initialize adc with desired parameters, run the adc, delegate printing to other task, and when done, release resources.
static void adc_task(void* args) {
    // maybe useful to pin this task to opposite core that printing task will be on.
    ESP_LOGI(ADC_TASK_TAG, "ADC Task Created");
    my_handlers_t *myHandles = (my_handlers_t*) args;
    adc_continuous_handle_t *adc_handle = myHandles->adc_handle;

    while(1) {
        ESP_LOGI(ADC_TASK_TAG, "ADC Task Waiting");
        // wait until start button is pressed. 
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // should put task into blocked state, getting no CPU time. 
        if (pupil_path_state) {
            // flowchart has this constraint. technically data can still be acquired, but tracking would not have proper image
            ESP_LOGW(ADC_TASK_TAG, "Pupil imaging path on, cannot start ADC");
            continue;   // will go wait again
        }
        // also check SD card detect pin
        // check if sample folder exists, if not, create it
        char tmpBuf[64] = {0};
        snprintf(tmpBuf, 64, "%s/SAMPLE%02u/", SD_MOUNT, sample_num);
        // created function to check existence of folder, and create it, if desired.
        if (check_dir(tmpBuf, true) != ESP_OK) {
            // only occurs if 'make_dir' is false.  
            ESP_LOGE(ADC_TASK_TAG, "Could not create folder (%s) for samples (does not exist).", tmpBuf);
            continue;
        }
        // overwrite/clear past log files
        snprintf(tmpBuf, 64, "%s%s/samp%03u.txt", SD_MOUNT, SAMPLE_LOG_DIR, sample_num);
        if (check_dir(tmpBuf, false) == ESP_OK) {
            // file exists, remove it (FUTURE: rename it? ie, append '.old' to file?)
            unlink(tmpBuf); // unlink == remove
            ESP_LOGW(ADC_TASK_TAG, "Removed previous log file (%s)", tmpBuf);
        }

        ESP_LOGI(ADC_TASK_TAG, "Beginning ADC setup...");
        ESP_LOGI(ADC_TASK_TAG, "Suspending monitor task (logging)");
        vTaskSuspend(monitor_handle);
        // suspend other tasks as necessary, or suppress logging somehow

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
        // tell python the sample frequency and duration
        printf("%s\nfreq: %"PRIu32"; duration: %"PRIu32"\n", PY_DATA, sample_frequency, sample_duration);   //in Hz and ms
        
        gpio_set_level(BLUE_LED, 0);    // turn overflow LED off
        blue_led_state = 0;

        // FUTURE: start waveplate. Abort if status not OKAY
        // assume that the waveplate has been pre-configured (TX 0sv32, TX 0sj00000000, TX 0ho1 [home])
        // send "fw" to run waveplate continuously

        // give a 3 second countdown. (perhaps before the waveplate, when that gets implemented) 
        for (int16_t i = 3; i >= 1; i--) {
            ESP_LOGI(ADC_TASK_TAG, "Starting ADC...(%"PRId16")", i);
            vTaskDelay(1000/portTICK_PERIOD_MS);    //give a little delay
        }
        ESP_ERROR_CHECK(adc_continuous_start(*adc_handle));
        adc_state = true;   //does this need a critical section? 
        gpio_set_level(RED_LED, adc_state); // turn LED on

        // copy task will be notified by adc_conv_ready_cb. Update byte count from ISR. wait here until resumed (after set bytes read)
        ESP_LOGI(ADC_TASK_TAG, "Suspending task (waiting to finish reading).");
        vTaskSuspend(NULL);
        
        // on resume, stop adc and deinit to free resources.
        ESP_ERROR_CHECK(adc_continuous_stop(*adc_handle));
        adc_state = false;  //update state var. 
        gpio_set_level(RED_LED, adc_state); // turn LED off. could just hard code 0. 
        #if !USE_SD_CARD
        // inform python that this sampling period has ended
        printf("%s\n", PY_END_TAG);
        #endif
        // FUTURE: stop waveplate
        // send "st" to stop waveplate rotation. Is there a reason to reverse ("bw") back to home? 

        ESP_ERROR_CHECK(adc_continuous_deinit(*adc_handle));
        ESP_LOGI(ADC_TASK_TAG, "ADC deinitialized.");
        *adc_handle = NULL; //reset back to null

    #if USE_SD_CARD
        // notify ADC transfer task
        xTaskNotifyGive(adc_transfer_handle);
        // wait for transfer to complete!   (2 options: use notification, or suspend this task [riskier?])
        vTaskSuspend(NULL);
        // Update sample_num after data has been transfered to PC
        sample_num++; 
    #endif

        ESP_LOGI(ADC_TASK_TAG, "Resuming monitor task (logging)");
        vTaskResume(monitor_handle);
    }
}
// Task to handle adc conversion result, whether it prints to python, or saves to SD card.
static void adc_copy_task(void* args) {
    // Will need the adc_handle for reading the data. 
    my_handlers_t * handles = (my_handlers_t*) args;
    adc_continuous_handle_t *adc_handle = handles->adc_handle;  //it is a pointer, de-reference as needed.
    // will also need the handle for the SD card (or have it global)
    // uint32_t byte_count = 0; //moved to global => bytes_read

    esp_err_t ret;
    uint32_t ret_num = 0;
    memset(adc_conv_buffer, 0xcc, ADC_BUFFER_LEN);   // clear the result buffer

    ESP_LOGI(ADC_COPY_TAG, "adc_copy_task created and initialized");
    for(;;) {
        ESP_LOGI(ADC_COPY_TAG, "Waiting for ADC Conversion Frame ...");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // wait for a conversion result to be done
        // determine the number of bytes to read.
        uint32_t bytes_to_read = (uint32_t)(2*num_adc_channels * (sample_duration/1000.0) * sample_frequency);    //may give float... cast to int, or round up 
        // reset file number and cycle number for this new sampling period
        uint16_t file_num = 0;
        // esp_cpu_cycle_count_t cycle_num = esp_cpu_get_cycle_count() / 10000;
        char file_path[64] = {'\0'};
        char data_file[64] = {'\0'};
        char temp_data[128] = { 0 };

        // char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);   //not used (except in an else)
        while (1) {
            ret = adc_continuous_read(*adc_handle, adc_conv_buffer, ADC_BUFFER_LEN, &ret_num, 1);
            if (ret == ESP_OK) {
                ESP_LOGI(ADC_COPY_TAG, "return val is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
                #if USE_SD_CARD
                // Get a file path to write results to, write to it, then close file.
                while (get_file_path(file_path, file_num) != ESP_OK) {
                    ESP_LOGE(ADC_COPY_TAG, "Error getting file path (%s). File Exists!", file_path);
                    // sample_num++;   // update sample number
                    // before breaking, must resume adc_task, otherwise infinite loop (will perpetually read data)
                    vTaskResume(adc_task_handle);   
                    vTaskDelay(1000/portTICK_PERIOD_MS);
                    break;  // breaks this loop, data will not be saved to a file!
                }
                file_num += 1; // increment the file num for next time
                FILE * f = fopen(file_path, "wb");  // open for binary writing
                if (f == NULL) {
                    ESP_LOGE(ADC_COPY_TAG, "Error openning file (%s)", file_path);
                    perror("Error Description");   // prints error message after failed FILE operation
                    break;
                }
                // // fwrite or fprintf the number of bytes to read then newline. alternatively, make a metadata file that presides over multiple files
                // // fwrite result buffer to a file (will write data as binary, therefore is most optimal!)
                fprintf(f, "size:%010lu\n", ret_num);  // number of bytes of data in file (total size: 5+10+1 = 16 bytes total)
                fwrite(adc_conv_buffer, ret_num, sizeof(adc_conv_buffer[0]), f);    // write the result buffer to the file!
                fclose(f);  // close file to flush changes to it. 
                // Add file_path and bytes written to a data file.
                snprintf(data_file, 64, "%s%s/samp%03u.txt", SD_MOUNT, SAMPLE_LOG_DIR, sample_num);
                snprintf(temp_data, 128, "File: %s\nBytes Written: %0lu", file_path, ret_num);
                if (append_log_file(data_file, temp_data) == ESP_FAIL) {
                    ESP_LOGE(ADC_COPY_TAG, "Error writing to log_file (%s)", data_file);
                    perror("Error Description");   // prints error message after failed FILE operation
                    break;
                };
                // log file updated
                #else   // USE_SD_CARD == FALSE, so print to console
                printf("%s\n", PY_TAG);   // display tag for python.
                printf("Number of Bytes: %"PRIu32"\n", ret_num);    //pass over number of bytes/lines to read
                // adc_task will suspend any tasks that log. 
                // Relocate this loop to transfer task which will read in data from SD file and format for output to python.
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
                        ESP_LOGW(ADC_COPY_TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                    }
                }
                #endif // end USE_SD_CARD
                // in either case, tally the number of bytes read so far. 
                bytes_read += ret_num;
                // check if we read enough bytes.
                if (bytes_read >= bytes_to_read) {
                    // printf("PY STOP");   // python error was from an internal buffer filling up, not requiring a termination key word
                    // resume ADC task to stop ADC and clean up
                    ESP_LOGI(ADC_COPY_TAG, "total bytes read: %"PRIu32"", bytes_read);
                    ESP_LOGI(ADC_COPY_TAG, "minimum sample bytes to read: %"PRIu32"", bytes_to_read);
                    vTaskResume(adc_task_handle);
                    vTaskDelay(1500/portTICK_PERIOD_MS); //give adc_task some time before breaking and waiting above
                    break;  // exit the inner while loop to wait for notify by adc_conv_done_cb
                }
                // xTaskResumeAll();   //resume all the other tasks, this one may now be preempted.
                // vTaskResume(main_task_handle);
                vTaskDelay(1);  // 1tick block to avoid watchdog timer.
            } else if (ret == ESP_ERR_TIMEOUT) {
                // timeout occured
                ESP_LOGW(ADC_COPY_TAG, "Timeout error from adc_read_continuous: %s", esp_err_to_name(ret));
                break;
            }
            else {  //catch other return values and print it
                ESP_LOGW(ADC_COPY_TAG, "bad return value: %s", esp_err_to_name(ret));
                // vTaskSuspend(adc_copy_handle); 
                vTaskDelay(1000/portTICK_PERIOD_MS);
                break;
            }
        }
    }
}
// Task to transfer adc data from file to PC
static void adc_transfer_task(void* args) {
    esp_err_t ret;
    uint8_t result[ADC_BUFFER_LEN]; // holder for results array read from file. -- may be able to reuse adc_conv_buffer to save on heap memory!
    char strBuf[128] = { 0 };   // buffer to hold data that is read in from the data file
    char data_file[64] = { 0 }; // buffer to hold the data file to be openned.
    uint32_t ret_num = 0;   // number bytes from file

    ESP_LOGI(ADC_TRANS_TAG, "adc_transfer_task initialized and beginning...");
    while(1) {
        ESP_LOGI(ADC_TRANS_TAG, "Task waiting for notification from adc_task");
        // Wait for notification to transfer data from SD card file to PC (via UART)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // snprintf(data_file, 64, "%s/%s/samp%03u.txt", SD_MOUNT, SAMPLE_LOG_DIR, sample_num);
        // snprintf(temp_data, 64, "File: %s\nBytes Written: %"PRIu32"\n", file_path, ret_num);
        // Read in the files that we need to transfer to PC from the sample log file
        snprintf(data_file, 64, "%s%s/samp%03u.txt", SD_MOUNT, SAMPLE_LOG_DIR, sample_num);
        FILE * f = fopen(data_file, "r");
        if (f == NULL) {
            ESP_LOGE(ADC_TRANS_TAG, "Error openning log file: %s", data_file);
            perror(NULL);   // prints error message after failed FILE operation
            clearerr(f);
            // before waiting in notify, resume adc_task to avoid requiring a hard reset
            vTaskResume(adc_task_handle);
            continue;   // loops to wait for notify
        }
        while (1) { // if actual # of files written is needed (equal to # of buffers needed), then it can be hypothesized 
            // contains records of 2 lines: 1st line gives "File: <file_name>", 2nd gives "Bytes Written: <bytes wrote>" for each file!
            ret = read_line_file(f, strBuf, 128);   // 1st line from data_file
            if (ret != ESP_OK) {
                ESP_LOGW(ADC_TRANS_TAG, "Problem reading 1st line (could be EOF). strBuf: %s", strBuf); // can check EOF with feof(FILE*)
                perror("Error Description");   // prints error message after failed FILE operation
                clearerr(f);
                break;
            }
            // now split the line based on ':' delimiter, using strtok function (in string.h)
            char* token = strtok(strBuf, " :"); //subsequent calls with NULL string will give next token.
            token = strtok(NULL, " :"); // should give file_name
            // check that token isn't NULL
            if (token == NULL) {
                ESP_LOGE(ADC_TRANS_TAG, "Error tokenizing 1st line ('%s')", strBuf);
                break;
            }
            // store file name or open file right away (latter chosen)
            FILE *sample_file = fopen(token, "rb");
            if (sample_file == NULL) {
                ESP_LOGE(ADC_TRANS_TAG, "Error openning sample file (%s)", token);
                perror("Error Description");   // prints error message after failed FILE operation
                clearerr(sample_file);
                // vTaskResume(adc_task_handle);   // resume the adc_task since error occured. 
                break;   // goes and waits at notify statement.
            }   //else continue on
            // read second line
            ret = read_line_file(f, strBuf, 128);   // 2nd line from data_file
            if (ret != ESP_OK) {
                ESP_LOGW(ADC_TRANS_TAG, "Problem reading 2nd line. strBuf: %s", strBuf);
                break;
            }
            token = strtok(strBuf, " :");   // split string on spaces and colons.   (Bytes)
            token = strtok(NULL, " :");     // second string after split (Written:)
            token = strtok(NULL, " :");     // 3rd string   (<number>)
            sscanf(token, "%lu", &ret_num); // convert string into a 32-bit unsigned int.
            if (ret_num == 0)   {
                ESP_LOGE(ADC_TRANS_TAG, "Error parsing number of bytes. token string: %s; integer: %lu", token, ret_num);
                break;
            } else if (ret_num >= ADC_BUFFER_LEN) {
                ESP_LOGW(ADC_TRANS_TAG, "ret_num (%lu) is larger than buffer size. Result will be truncated to buffer length", ret_num);
                ret_num = ADC_BUFFER_LEN;   // set return num to buffer size. 
            }
            // now read in result array from sample_file. First line gives size again... either remove it, or make it a fixed # of bytes
            fread(strBuf, 16, sizeof(char), sample_file);
            ESP_LOGD(ADC_TRANS_TAG, "First line of sample_file: %s", strBuf);

            fread(result, ret_num, sizeof(uint8_t), sample_file);   // reads file into result container.
            // Now begin printing to Python!
            printf("%s\n", PY_TAG);   // display tag for Python.
            printf("Number of Bytes: %"PRIu32"\n", ret_num);    //pass over number of bytes/lines to read
            // adc_task will suspend any tasks that log. => with relocation, this function should now take care of this!
            // Relocated loop from copy_task to transfer task which will read in data from SD file and format for output to python.
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p); //uint32_t is much bigger than necessary
                uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                    // printf("ch: %"PRIu32"; value: %"PRIx32"\n", chan_num, data); 
                    // print as few characters as possible for speed, although not as important with this implementation
                    printf("%"PRIu32", %"PRIx32"\n", chan_num, data);
                    // fwrite();    // write a buffer of data to stdout stream. 
                } else {
                    ESP_LOGW(ADC_TRANS_TAG, "Invalid data [%"PRIu32"_%"PRIx32"]", chan_num, data);
                }
            }
            fclose(sample_file);    // release resources
        }   // while for # of buffer files
        fclose(f);  // close the file to release resources
        // inform python that this sampling period has ended
        printf("%s\n", PY_END_TAG);
        vTaskResume(adc_task_handle);   // resume the adc_task that is waiting for this to finish 
    }
}
// Task to periodically display debug or info messages on variable states, etc. Also updates vars from PCNT events.
static void monitor_var_task(void* args) {
    ESP_LOGI(MONITOR_TASK_TAG, "Variable Monitoring Task started");
    my_pcnt_t *user_data = (my_pcnt_t*) args;
    // Counts for rotary encoder
    int pulse_count = 0;
    int event_count = 0;    //passed by reference to xQueueReceive
    while(1) {
        // 1000ms = 1s delay, queue waits for 1s. 
        // vTaskDelay(1000 / portTICK_PERIOD_MS);  // avoid watchdog and spamming of logs.

        // report PCNT count; problem: will only update every second because of delay above. 
        if (xQueueReceive(*(user_data->event_queue), &event_count, 1000/portTICK_PERIOD_MS)) { // blocks for 1s waiting to see if update to counter
            ESP_LOGI(MONITOR_TASK_TAG, "PCNT Event Count: %d", event_count);
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
            }
            taskEXIT_CRITICAL(&param_lock);
        } else  {  // else print some debugging logs
            ESP_ERROR_CHECK(pcnt_unit_get_count(*(user_data->pcnt_handle), &pulse_count));
            ESP_LOGI(MONITOR_TASK_TAG, "PCNT Current Count: %d", pulse_count);
            // debug logs
            // ESP_LOGD(MONITOR_TASK_TAG, "adc_state: %d", adc_state);  // visually shown with LED
            ESP_LOGI(MONITOR_TASK_TAG, "servo state: %d", pupil_path_state);
            ESP_LOGD(MONITOR_TASK_TAG, "bytes_read: %"PRIu32"", bytes_read);
            ESP_LOGD(MONITOR_TASK_TAG, "menuIndex: %d", menuIndex);     // visually shown on LCD, but good as debug. 
            ESP_LOGD(MONITOR_TASK_TAG, "base_pos: %"PRIu16"", base_pos);    
        }
    }
}
// Task to service servo updates
static void servo_task(void* args) {
    // needs comparator to call mcpwm_comparator
    mcpwm_cmpr_handle_t *comparator = (mcpwm_cmpr_handle_t*) args;

    ESP_LOGI(SERVO_TASK_TAG, "Servo task entering loop");
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // wait for notification from ISR
        if (pupil_path_state) {
            ESP_LOGI(SERVO_TASK_TAG, "Pupil Image ON, servos set to max");
            servo_angle = SERVO_MAX_DEGREE;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*comparator, example_angle_to_compare(servo_angle)));
            vTaskDelay(pdMS_TO_TICKS(500));    // give servo time to get to position
        } else {
            ESP_LOGI(SERVO_TASK_TAG, "Pupil Image OFF, servos set to min");
            servo_angle = SERVO_MIN_DEGREE;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*comparator, example_angle_to_compare(servo_angle)));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

static inline uint32_t example_angle_to_compare(int angle)  {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

// Function that takes a path (/sdmount/<file_path>) and writes the result data buffer to the file
// static esp_err_t sd_write_result_file(const char * path, char * data) {
//     ESP_LOGI(SD_TAG, "Opening file %s", path);  // if there's a need for speed, then these should be commented out!
//     FILE *f = fopen(path, "wb");    // binary write mode is faster than formatting in ASCII. 
//     if (f == NULL) {
//         ESP_LOGE(SD_TAG, "Failed to open file for writing");
//         return ESP_FAIL;
//     }
//     // write the result buffer line by line? 
//     // fwrite(f, sizeof(*data), )
//     fclose(f);
//     ESP_LOGI(SD_TAG, "File written");
//     return ESP_OK;
// }

void app_main(void) {
    esp_err_t ret;  // var to hold return values

    main_task_handle = xTaskGetCurrentTaskHandle();     // get task handle for main
    // Set log level to allow display of debug-level logging
    esp_log_level_set("*", ESP_LOG_VERBOSE);  // enables debug logs globally (for debugging SD card)
    esp_log_level_set(MAIN_TAG, ESP_LOG_DEBUG);
    esp_log_level_set(MONITOR_TASK_TAG, ESP_LOG_DEBUG);
    esp_log_level_set("sdmmc_req", ESP_LOG_DEBUG);
    esp_log_level_set("sdmmc_cmd", ESP_LOG_DEBUG);
    // ---- gpio setup ----
    // -- gpio inputs
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_IN_MASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;        // external recommended because internal too weak for most devices

    gpio_config(&io_conf);
    // Change default settings for SD card detect.
    // gpio_pullup_dis(SD_DETECT);     // disable the default pullup for SD_detect
    // gpio_pulldown_en(SD_DETECT);    // active high, means it must be GND for off (pulldown). -- handled by breakout board
    // -- gpio outputs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUT_MASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;    //external pullups implemented because internal too low voltage

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
        .alarm_count = DEBOUNCE_TIME_MS*10,   // time in ms * 10ticks/ms
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
    #pragma region 
    // use functions defined in rotator_driver.h
    // set velocity ("sv") to 50-70% of max. so 0x32 to 0x48 would be sent as ASCII ('3' + '2', etc. )
    // set jog step size to 0 for continuous ("sj")
    init_waveplate_uart();  // initialize the UART for the waveplate
    send_waveplate_command("ho", 1, 1);
    if (!read_waveplate_response()) {
        ESP_LOGE(MAIN_TAG, "Error returning waveplate to home position");
        // return?
    }
    send_waveplate_command("sv", 50, 2);    // set speed to 50%
    // read response, continue if OK
    if (!read_waveplate_response()) {
        ESP_LOGE(MAIN_TAG, "Error setting velocity for waveplate");
    }
    send_waveplate_command("sj", 0, 8);     // set jog size to 0 for continuous motion
    if (!read_waveplate_response()) {
        ESP_LOGE(MAIN_TAG, "Error setting jog step");
    }

    #pragma endregion
    // // --- I2C LCD ---
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
    // lcd1602_reset(lcd_ctx);     // tabula rasa  - blank slate! nope. in code it runs set_mode with L->R true...

    lcd1602_home(lcd_ctx);  // print hello world! in 'center'
    lcd1602_string(lcd_ctx, "   Hello World!!!   From ESP32");  // ... From ESP32 wraps onto 3rd line (row 2)
    // lcd1602_set_cursor(lcd_ctx, 0, 19);  // line 3 = from line 1, it is col 20-39.
    // lcd1602_string(lcd_ctx, " 0,20 Line 3");
    lcd1602_set_cursor(lcd_ctx, 1, 19);     // line 4 = from line 2, it is col 20-39
    lcd1602_string(lcd_ctx, " 1,20 Line 4");

    lcd1602_set_display(lcd_ctx, true, true, true);    //cursor enabled for rotary encoder, optionally blink
    lcd1602_set_mode(lcd_ctx, false, false);
    vTaskDelay(1000/portTICK_PERIOD_MS);    // let the hello world display for a second.
    // interestingly, it doesn't scroll until later... (LCD task was preempted by scheduler, and so only 1 character was getting set at a time!)
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
    QueueHandle_t pcnt_queue = xQueueCreate(20, sizeof(int));   // size limits how many events can occur in the time that it takes to process an event
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &pcnt_cbs, pcnt_queue));

    ESP_LOGI(MAIN_TAG, "Enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(MAIN_TAG, "Clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(MAIN_TAG, "Start pcnt unit");  // well that would do it!
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    ESP_LOGI(MAIN_TAG, "PCNT unit started.");
    // structure to send to monitor task
    my_pcnt_t monitor_data = {0};
    monitor_data.event_queue = &pcnt_queue;
    monitor_data.pcnt_handle = &pcnt_unit;

    #pragma endregion
    // --- MCPWM ---
    #pragma region
    ESP_LOGI(MAIN_TAG, "MCPWM: Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
   
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(MAIN_TAG, "MCPWM: Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(MAIN_TAG, "MCPWM: Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator1 = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_1,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator1));
    
    mcpwm_gen_handle_t generator2 = NULL;
    generator_config.gen_gpio_num = SERVO_2;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator2));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(MAIN_TAG, "MCPWM: Set generator action on timer and compare event");
    // go high on counter empty - servo 1
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator1,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold - servo 1
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator1,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    // go high on counter empty - servo 2
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold  - servo 2
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(MAIN_TAG, "MCPWM: Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    #pragma endregion
    // --- SD Card / SDMMC ---
    #pragma region
    ESP_LOGI(MAIN_TAG, "Waiting to initialize SD card (2s). Ensure clock wire is secured.");
    vTaskDelay(2000/portTICK_PERIOD_MS);

    esp_vfs_fat_sdmmc_mount_config_t mount_conf = {
        .format_if_mount_failed = FORMAT_IF_MOUNT_FAILS,
        .max_files = 8,    // max # of open files
        .allocation_unit_size = KB_TO_BYTES(16)   //bigger is better for large file R/W. cost is overhead on small files
    };
    sdmmc_card_t * card_handle;
    const char mount_point[] = SD_MOUNT;    // "/sdcard"
    ESP_LOGI(MAIN_TAG, "Initializing SD card");
    ESP_LOGI(MAIN_TAG, "Using SDMMC peripheral");
    // default host handle will set max frequency to 20MHz and 4 bit mode.
    // this is sufficent, (10 *10^6 B/s, vs. 8 *10^6 B/s) 
    sdmmc_host_t host_handle = SDMMC_HOST_DEFAULT();
    host_handle.max_freq_khz = SD_FREQUENCY; // if 40MHz is possible/ needed

    // power is supplied via breakout SD card board (3.3V pullups))
    // Configure the sdmmc slot using default values. 
    sdmmc_slot_config_t slot_conf = SDMMC_SLOT_CONFIG_DEFAULT();
    // slot_conf.gpio_cd = SD_DETECT;  // we will use the SD detect to prevent ADC from running if no place to put data!
    slot_conf.width = SD_LINE_WIDTH;    // configurable line width
    // External pullups are used, so internal pullups unnecessary.
    slot_conf.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    ESP_LOGI(MAIN_TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host_handle, &slot_conf, &mount_conf, &card_handle);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(MAIN_TAG, "Failed to mount filesystem. ");
        } else {
            ESP_LOGE(MAIN_TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(MAIN_TAG, "Filesystem mounted!");
    sdmmc_card_print_info(stdout, card_handle); // prints info on the SD card connected
    // ESP_LOGW(MAIN_TAG, "SD card not yet implemented, filesystem NOT mounted!");
    #pragma endregion

    // create log file folder, if it doesn't already exist
    char tmpBuf[64] = {0};
    sprintf(tmpBuf, "%s%s/", SD_MOUNT, SAMPLE_LOG_DIR);
    struct stat st;
    if (stat(tmpBuf, &st) != 0) {
        // then an error occured, likely a dir not found
        mkdir(tmpBuf, ACCESSPERMS);
        ESP_LOGI(ADC_TASK_TAG, "Directory created for sample logs: %s", tmpBuf);
    } // else stat will have info on the folder.

    adc_continuous_handle_t handle = NULL;  //needed in many places, hold it here for distribution.

    my_handlers_t handles = {
        .adc_handle = &handle,
        .gptimer = &timer_handle,
    };
    // Install GPIO ISRs
    gpio_isr_handler_add(START_BUT, start_isr_handler, (void*)&handles);
    gpio_isr_handler_add(AUX_BUT, aux_isr_handler, (void*)&handles);
    gpio_isr_handler_add(ROTARY_SWITCH, rot_switch_isr_handler, (void*)&timer_handle);

    // register tasks: adc_task, adc_copy (to SD card), adc_transfer (from SD card to PC/python), monitor_vars (logging), servo (control)
    xTaskCreatePinnedToCore(adc_task, "ADC-TASK", TASK_STACK_SIZE, (void*)&handles, ADC_TASK_PRIORITY, (void*)&adc_task_handle, 1);    
    // pin printing task to core 1, and limit number of tasks on that core (better to not be pinned. Pin other tasks instead). 
    xTaskCreatePinnedToCore(adc_copy_task, "ADC-COPY", TASK_STACK_SIZE, (void*)&handles, COPY_TASK_PRIORITY, (void*) &adc_copy_handle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(adc_transfer_task, "ADC-TRANSFER", TASK_STACK_SIZE*5, (void*)NULL, ADC_TRANSFER_PRIORITY, &adc_transfer_handle, 1);
    // vTaskSuspend(adc_copy_task);
    // Task that updates the lcd periodically. Cannot be preempted by other tasks when setting LCD display. Will scroll if interrupted.
    // xTaskCreate(lcd_task, "LCD-TASK", TASK_STACK_SIZE, lcd_ctx, 2, &lcd_task_handle);   // perhaps scrolling effect is caused by getting kicked off of CPU. increased priority
    // Task for updating variables from PCNT events, and printing log messages.
    xTaskCreatePinnedToCore(monitor_var_task, "MONITOR-TASK", TASK_STACK_SIZE, (void*)&monitor_data, MONITOR_TASK_PRIORITY, &monitor_handle, 0);
    // servo task creation
    xTaskCreatePinnedToCore(servo_task, "SERVO-TASK", TASK_STACK_SIZE, (void*)&comparator, SERVO_TASK_PRIORITY, &servo_task_handle, tskNO_AFFINITY);
    
    // set angle of servos to 'off' state
    servo_angle = SERVO_MIN_DEGREE;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(servo_angle)));
    //Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
    vTaskDelay(pdMS_TO_TICKS(500));

    // Vars for LCD printing
    lcd1602_clear(lcd_ctx);
    char strBuf[41] = {' '};    // buffer for formatting numbers into a string for the LCD
    size_t buf_size = sizeof(strBuf) / sizeof(char);
    ESP_LOGI(MAIN_TAG, "strBuf: %s;", strBuf);

    // Use main task as LCD_task. Off load monitoring to a different task.
    while(1) {
        // if (adc_state) => print "ADC running" or "Sampling", else "ADC off"/"Standby"
        // placed before other section because cursor would be in different location. 
        if (adc_state) { 
            // lcd1602_set_cursor(lcd_ctx, 1, 19);// lcd1602_string(lcd_ctx, BLANK_LINE);    // this causes the text to flash by updating...
            snprintf(strBuf, 22, " %-20s", "ADC Running...");
            lcd1602_set_cursor(lcd_ctx, 1, 19);
            lcd1602_string(lcd_ctx, strBuf);
        } else {
            snprintf(strBuf, 22, " %-20s", "ADC Off / Standby");
            lcd1602_set_cursor(lcd_ctx, 1, 19);
            lcd1602_string(lcd_ctx, strBuf);
        }
        // check if SD card is inserted or not, if not, display a message, else ... temporarily display one? 
        // if (gpio_get_level(SD_DETECT) == 0) {
        //     snprintf(strBuf, 22, " %-20s", "No SD Card Inserted!");
        //     lcd1602_set_cursor(lcd_ctx, 0, 19);
        //     lcd1602_string(lcd_ctx, strBuf);
        // }  else {   //else clear that line!
        //     snprintf(strBuf, 22, " %s", BLANK_LINE);
        //     lcd1602_set_cursor(lcd_ctx, 0, 19);
        //     lcd1602_string(lcd_ctx, strBuf);
        // } 
        // change menu based on menu index, and display the appropriate value
        if (menuIndex == 0) {           //frequency
            lcd1602_set_cursor(lcd_ctx, 0, 1);
            lcd1602_string(lcd_ctx, menu[menuIndex]);
            // format numerical data into string
            memset(strBuf, 32, buf_size);    // clear string (with spaces), then format it
            snprintf(strBuf, buf_size, "  %4ld %-11s", sample_frequency / 1000, menuUnit[menuIndex]);  //be cognizant if no null-termination

            lcd1602_set_cursor(lcd_ctx, 1, 2);
            lcd1602_string(lcd_ctx, strBuf);
            // reposition cursor to current unit position
            lcd1602_set_cursor(lcd_ctx, 1, (10-base_pos));  // range of pos for freq. is 3-5 (incl). at min, want cursor at 7. 
        } else if (menuIndex == 1) {    //duration
            lcd1602_set_cursor(lcd_ctx, 0, 1);
            lcd1602_string(lcd_ctx, menu[menuIndex]);
            // format numerical data into string
            memset(strBuf, 32, buf_size);    // clear string, then format it
            snprintf(strBuf, buf_size, "  %4ld %-11s", sample_duration, menuUnit[menuIndex]);

            lcd1602_set_cursor(lcd_ctx, 1, 2);
            lcd1602_string(lcd_ctx, strBuf);
            // reposition cursor to current unit position
            lcd1602_set_cursor(lcd_ctx, 1, (7-base_pos));
        } else if (menuIndex == 2) {    //pupil image?
            lcd1602_set_cursor(lcd_ctx, 0, 1);
            lcd1602_string(lcd_ctx, menu[menuIndex]);
            int pos;
            if (pupil_path_state)   pos = 1;
            else    pos = 0;
            // format numerical data into string
            memset(strBuf, 32, buf_size);    // clear string, then format it
            snprintf(strBuf, buf_size, "  %4s %-11s", servoStatus[pos], menuUnit[menuIndex]);

            lcd1602_set_cursor(lcd_ctx, 1, 2);
            lcd1602_string(lcd_ctx, strBuf);   // display formatted string to LCD
            // reposition cursor to current unit position
            lcd1602_set_cursor(lcd_ctx, 1, (7-base_pos));
        }      
        // ESP_LOGI(MAIN_TAG, "strBuf: %s", strBuf);   // can change to debug-level later
        // if the 3rd and 4th lines are used later, the above will write over it (maybe). Soln: reduce size of strBuf. Actual Soln: not necessary!     
        // wait a reasonable time before refreshing/updating LCD
        vTaskDelay(250/portTICK_PERIOD_MS); // 250ms delay make smaller if want faster response to user input. 
    }
    // unmount the sd card
    esp_vfs_fat_sdcard_unmount(mount_point, card_handle);
    ESP_LOGI(MAIN_TAG, "Card unmounted");
    lcd1602_deinit(lcd_ctx);
    ESP_LOGI(MAIN_TAG, "LCD deinitialized");
}
