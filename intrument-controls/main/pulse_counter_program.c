/*  Summer-Fall 2024 Senior Design Group 3
 *  Embedded Programming Restart #2 - Using PCNT instead of ADC
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
// NVS
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"

//      ----- Pin Assignments -----
#pragma region 
#define START_BUT       GPIO_NUM_35     // starts the whole process
#define AUX_BUT         GPIO_NUM_34     // more of Auxilary button (AUX)

#define ROTARY_A        GPIO_NUM_33     // one of the rotary encoder/dial's inputs. left is B, right is A to satisfy below.
#define ROTARY_B        GPIO_NUM_32     // switch for proper directional increments (clock wise = + increase)
#define ROTARY_SWITCH   GPIO_NUM_25      // switch/press of dial

// pins changed for ESP32WROOM32 chip/dev board - 23 24?
#define LCD_SDA         GPIO_NUM_10     // SDA pin for i2c LCD (20x04) [green wire]
#define LCD_SCL         GPIO_NUM_9      // SCL pin for i2c LCD

#define SERVO_1         GPIO_NUM_26     // servo 1 for 1st fold mirror  (green wire) 
#define SERVO_2         GPIO_NUM_27     // servo 2 for 2nd fold mirror  (blue wire)

// TX/RX arbitarily chosen - defined in rotator_driver.h / waveplate_uart.h
// #define WAVEPLATE_TX    GPIO_NUM_21     // TX for UART to control Waveplate Rotator
// #define WAVEPLATE_RX    GPIO_NUM_22     // RX for UART to control Waveplate Rotator

// Former ADC pins, now used with PCNT
#define PULSE_PIN_1     GPIO_NUM_36
#define PULSE_PIN_2     GPIO_NUM_37
#define PULSE_PIN_3     GPIO_NUM_38
#define PULSE_PIN_4     GPIO_NUM_39

#define PULSE_HIGH_COUNT        20000   // Counters are 16-bit signed
#define PULSE_UNIT_STR(unit)    #unit   // macro to get number/string/identifier from unit handle?
// SD card pins - also 3.3V and GND connection. (3.3V is important) [these are fixed pins, included here for convenience]
#define SD_CMD          GPIO_NUM_15     // brown wire
#define SD_CLK          GPIO_NUM_14     // white wire
#define SD_DETECT       GPIO_NUM_19     // gray wire - when high, SD card is inserted. Hardwire an LED --> loading effect drops voltage to ~1.7V
// --- Data Lines ---
#define SD_DAT0         GPIO_NUM_2      // blue wire
#define SD_DAT1         GPIO_NUM_4      // green wire
#define SD_DAT2         GPIO_NUM_12     // yellow wire
#define SD_DAT3         GPIO_NUM_13     // orange wire
// - SD Params -
#define SD_LINE_WIDTH   4               // want 4-line/bit width, but doesn't work, so 1-line it is (for now)
#define SD_FREQUENCY    SDMMC_FREQ_DEFAULT    // 40MHz fastest, default is 20MHz, and probing (slowest) is 400kHz
    // problem with SD card could be impedence mismatching
// --- LEDs ---
#define GREEN_LED       GPIO_NUM_0      // General debug LED (button presses)
#define RED_LED         GPIO_NUM_5      // ADC on/off status LED
#define BLUE_LED        GPIO_NUM_18     // ADC overflow LED
#define SD_LED          GPIO_NUM_23     // SD detect LED

// Convient Masks for inputs and outputs when configuraing GPIO.
#define GPIO_IN_MASK    (1ULL << START_BUT | 1ULL << AUX_BUT | 1ULL << ROTARY_SWITCH) // | 1ULL << SD_DETECT)
#define GPIO_OUT_MASK   (1ULL << GREEN_LED | 1ULL << RED_LED | 1ULL << BLUE_LED || 1ULL << SD_LED)

#define ESP_INTR_FLAG_DEFAULT 0     //define flag for gpio ISRs
#pragma endregion
// ----- -----

// ----- DEFINEs for Program -----
// --- Macros ---
#define KB_TO_BYTES(k)      k*1024          //to convert KB to bytes
// --- ---
#define ROTARY_HIGH         4
#define ROTARY_LOW          -4
#define DEBOUNCE_TIME_MS    300     // time to wait, in milliseconds, before re-enabling button interrupts
// naively increase buffer size to permit longer sample duration.
// #define ADC_BUFFER_LEN                  KB_TO_BYTES(16)
#define DEFAULT_SAMPLE_FREQ                50000 // true default of 500,000 Hz (500kHz)
#define DEFAULT_DURATION                250   //ms

#define PULSE_DATA_BUF_LEN      6*512
// Servos
#pragma region 
// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

// Angles for the servos when pupil is set to ON or OFF. Servo1 = Front, Servo2 = Back
#define SERVO1_ON  63
#define SERVO1_OFF  0
#define SERVO2_ON  -60
#define SERVO2_OFF  0
#pragma endregion

#define SD_MOUNT "/sdcard"  //mount path for SD card
#define FORMAT_IF_MOUNT_FAILS   true    // this will format the card for the 1st time (if mounting fails).
#define USE_SD_CARD             false    // used with #ifdef to either print ADC to file (on SD card) or to python/console.
#define USE_RAW_SD              true
// define max # of files and unit size? 

#define UART_READ_BUF           512    // buffer size for reading from PC UART channel
static QueueHandle_t pc_uart_queue;
// ----- -----

// ----- TASKS -----
#pragma region 
// make this large enough to avoid task stack overflows. if too large, scale back, or set it per task 
#define TASK_STACK_SIZE                 KB_TO_BYTES(8)    //8kB
static TaskHandle_t main_task_handle = NULL;
static const char *MAIN_TAG = "MAIN-TASK";

static TaskHandle_t copy_sample_handle = NULL;      //copy to SD card
static const char *SAMPLE_COPY_TAG = "SAMP-COPY";
#define COPY_TASK_PRIORITY      5

static TaskHandle_t start_task_handle = NULL;       // starts sampling
static const char * START_TASK_TAG = "SAMPLE-START";
#define START_TASK_PRIORITY       ((UBaseType_t) 1U)

static TaskHandle_t sample_transfer_task_handle = NULL; // transfer from SD card to PC over UART
static const char * TRANSFER_TAG = "SAMP-TRANSFER";
#define SAMPLE_TRANSFER_PRIORITY   tskIDLE_PRIORITY

static TaskHandle_t monitor_handle = NULL;
static const char * MONITOR_TASK_TAG = "MONITOR";
#define MONITOR_TASK_PRIORITY   tskIDLE_PRIORITY

static TaskHandle_t uart_monitor_handle = NULL;
static const char * UART_MON_TAG = "UART-RX";
#define UART_RX_TASK_PRIORITY   0
#define PC_UART_NUM             UART_NUM_0
#define UART_PATTERN_CHAR       '\n'
#define PATTERN_CHR_NUM         (1)

static TaskHandle_t servo_task_handle = NULL;
static const char* SERVO_TASK_TAG = "SERVO";
#define SERVO_TASK_PRIORITY     1

static const char* SD_TAG = "SD-TRANSFER";
static const char* NVS_TAG = "NVS";
#pragma endregion

static const char * PY_TAG = "BEGIN PY READ";       // begin reading a segment of ADC_BUFFER_LEN
static const char * PY_END_TAG = "END PY SAMPLE";   // tag to save partial segments into 1 file 
static const char * PY_DATA = "PY METADATA";        // tag to tell python here is metadata (sample frequency and duration)

// ----- STATE VARIABLES -----
// volatile if value can change within an ISR context
volatile bool sample_state = false; 
volatile bool led_state = false;            //green (debug) LED state 
volatile bool red_led_state = false;        // the status LED
volatile bool blue_led_state = false;       // the overflow LED
volatile bool pupil_path_state = false;     // essentially the state of the servos
// static int servo_angle = 0;                 // angle of servos 
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
// ----- -----
// --- Parameters ---
uint32_t sample_frequency = DEFAULT_SAMPLE_FREQ;    // default 1MHz frequency
uint32_t sample_duration = 250;         // default 250ms 
volatile uint32_t bytes_read = 0;   // no longer changes in ISR context, removed volatile. except volatile is used more than just from ISRs
volatile size_t end_sample_buf_idx = 0;

// Queue Handle for copy task
QueueHandle_t copy_queue = NULL;

// Handles for the pcnt counters. Global so that we don't need to pass them around all the time.
static pcnt_unit_handle_t counter_handle1 = NULL;      // 2 channels per handle, but interrupts are unit-based
static pcnt_unit_handle_t counter_handle2 = NULL;
static pcnt_unit_handle_t counter_handle3 = NULL;
static pcnt_unit_handle_t counter_handle4 = NULL;

// Timer handles - I think static is okay for these
static gptimer_handle_t segment_timer = NULL;
static gptimer_handle_t duration_timer = NULL;
static gptimer_handle_t debounce_timer = NULL;

// NVS Global handle
static nvs_handle_t my_nvs_handle;

// SD raw access helper variables
#define DEFAULT_STARTING_SECTOR     8*512
static uint32_t sample_start_sector = DEFAULT_STARTING_SECTOR;  // holds the starting sector address for a sample. changes as more samples created
volatile uint32_t sample_sector = DEFAULT_STARTING_SECTOR;    // the sector address that changes during execution. sample_start_sector stays same during sample run.
volatile uint32_t sample_num = 0; // number to set directory or file name for samples. potentially change to 32-bit to use with current NVS functions
#define SAMPLE_NUM_NVS      "sample_num"
#define SAMPLE_START_NVS    "start_sector"   // ah, limited to 16 characters (including \0). 
// SD card initialization vars
static sdmmc_host_t host_g;
static sdmmc_slot_config_t slot_config_g;
// SPI Flash FATFS setup (to store information on each sample)
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE; // Handle of the wear levelling library instance
const char *base_path = "/spiflash";    // Mount path for the internal flash partition
const char *FLASH_TAG = "FLASH-IO";
#define EX_FILE_LEN     256

#define SUPPRESS_LEVEL  ESP_LOG_DEBUG       // level to use for the 'suppress' command. Monitor always at WARN level. 
volatile bool logs_suppressed = false;      // bool to tell if logs are suppressed or not 
#define SAMPLE_LOG_DIR      "/SAMP_LOG"     // UNUSED - folder to hold logs for each sample
// FILE * sample_files[10];    // arary of FILE pointers (*) used to store list of files from a sample session.

// ----- LOCKS -----
// for concurrency issues, spinlocks should be used (easiest solution)
// static portMUX_TYPE adc_lock = portMUX_INITIALIZER_UNLOCKED;  //spinlock for adc_state
static portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;  //lock for sample parameters, and menu index

// ----- STRUCTURES -----
#pragma region 
// Structure holds the comparators for the two servos, so that they can be controlled separately (one CW [clockwise], other CCW)
typedef struct my_servos_t {
    mcpwm_cmpr_handle_t *comp1;
    mcpwm_cmpr_handle_t *comp2;
} my_servos_t;  
// Holds data for the rotary encoder
typedef struct my_pcnt_t {
    pcnt_unit_handle_t *pcnt_handle;
    QueueHandle_t *event_queue;
} my_pcnt_t;
// structure to hold any handles (as pointers) that may be needed to be sent to functions
typedef struct my_handlers_t{
    // adc_continuous_handle_t *adc_handle;
    gptimer_handle_t *gptimer;
    my_servos_t * servos;
    sdmmc_card_t * card;
} my_handlers_t;
// typedef struct file_data {
//     char* file_path;
//     uint32_t result_size;
//     // can add other fields of data as time progresses
// } file_data;
typedef struct pulse_data {
    uint64_t timer_count;
    int32_t counts[4];
} pulse_data;
typedef struct count_event_data {
    pcnt_unit_handle_t handle;  // type is a pointer already
    int32_t count;
} count_event_data;
#pragma endregion
// ----- -----
// --- Prototypes (as needed) ---
static inline uint32_t example_angle_to_compare(int angle);
static void handle_command(const char* command, void * args);
static void suppress_logs(esp_log_level_t general_level);
static void unsuppress_logs(esp_log_level_t global_level);
// ----- 
/**
 * @brief Gets an unsigned, 32-bit integer from NVS with error handling included
 * 
 * @returns ESP_OK if integer retrieved successfully; ESP_ERR_NVS_NOT_FOUND - value isn't initialized yet; or another error
 */
esp_err_t get_nvs_uint(char* var_name, uint32_t * var) {
    esp_err_t err = nvs_get_u32(my_nvs_handle, var_name, var);
    switch (err) {
        case ESP_OK:
            ESP_LOGI(NVS_TAG, "Integer from NVS: %s = %"PRIu32"", var_name, *var);
            // printf("%s = %" PRIu32 "\n", var_name, *var);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(NVS_TAG, "The NVS value (%s) is not initialized yet!\n", var_name);
            break;
        default :
            ESP_LOGE(NVS_TAG, "Error (%s) reading NVS value!\n", esp_err_to_name(err));
    }
    return err;
}
/**
 * @brief Gets a 32-bit integer from NVS with error handling included
 * 
 * @returns ESP_OK if integer retrieved successfully; ESP_ERR_NVS_NOT_FOUND - value isn't initialized yet; or another error
 */
esp_err_t get_nvs_int(char* var_name, int32_t * var) {
    esp_err_t err = nvs_get_i32(my_nvs_handle, var_name, var);
    switch (err) {
        case ESP_OK:
            ESP_LOGI(NVS_TAG, "Integer from NVS: %s = %"PRId32"", var_name, *var);
            // printf("%s = %" PRId32 "\n", var_name, *var);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(NVS_TAG, "The NVS value (%s) is not initialized yet!\n", var_name);
            break;
        default :
            ESP_LOGE(NVS_TAG, "Error (%s) reading NVS key!\n", esp_err_to_name(err));
    }
    return err;
}
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
    snprintf(out, 63, "%s/SAMPLE%02lu/%08u.dat", SD_MOUNT, sample_num, fileNum); //increment after
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
    FILE *f = fopen(file_path, "ab");  // append mode (to end of file). Will create a file if needed
    if (f == NULL) {
        return ESP_FAIL;
    }
    // append data to file
    fprintf(f, "%s\n", data);
    fclose(f);  // close file to save
    return ESP_OK;
}
/**
 * Function to create a new text file (.txt) for logging information, or storing metadata. 
 * Overwrites file if one already exists
 * @param file_path: Path of the logging file, or file to store data
 * @param data: data to write to the given file
 * @returns Returns either ESP_OK or ESP_FAILS if file fails to open
 */
static esp_err_t write_log_file(char *file_path, char* data) {
    FILE *f = fopen(file_path, "wb");  // write mode (to overwrite existing logs). Will create a file if needed
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
 * @param fp: FILE pointer to a file opened with reading permission, and binary data.
 * @param out: pointer to the string output
 * @param max_length: max length of string output
 * @return  
 *  ESP_OK if line read successfully, ESP_FAIL if an error occurs (no error handling, so no return), 
 *  ESP_TIMEOUT if max_length reached (still ok),
 *  ESP_ERR_INVALID_STATE if end-of-file (EOF) is reached.
 */
static esp_err_t read_line_file(FILE* fp, char * out, size_t max_length) {
    //size_t num_read = 0;
    size_t i = 0;
    // while( (*(out+i) = fgetc(fp)) != '\n' && i++ < max_length -1) i++;    // one liner!
    // *cough* strchr(string, '\n') will return pointer/position of first occurance of '\n'...
    for (i = 0; i < max_length -1; i++) {
        int ch = fgetc(fp);
        // printf("in 'read_line_file': ch: %c, 0x%X\n", ch, ch);
        if (ch == '\n') {
            // stop b/c of newline. b/c using a file, file cursor keeps position.
            break;
        }   // could combine into one if statement, but if need to distinguish for debug...
        else if (ch == 0)    // null char
            break;
        else if (ch == EOF)  //EOF is -1
            break;
        else // if current character is none of these, assign it to our string
            *(out+i) = ch;
    }
    *(out+i) = 0;   // null-termination. overwrites the newline.

    if (i >= max_length)
        return ESP_ERR_TIMEOUT;
    if (feof(fp)) 
        return ESP_ERR_INVALID_STATE;
    return ESP_OK;
}
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

// ----- -----

// Run this first to configure the SD slot parameters
void init_sd_config(sdmmc_host_t *out_host, sdmmc_slot_config_t *out_slot_config, int freq_khz) {
    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz
    // (range 400kHz - 40MHz for SDMMC, 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    ESP_LOGI(SD_TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = freq_khz;

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    // TODO: add card detect (CD)
    slot_config.gpio_cd = SD_DETECT;
    // Set bus width to use:
    slot_config.width = 4;

    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    // slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    *out_host = host;
    *out_slot_config = slot_config;
}
// use this to init the SD card for raw access
esp_err_t init_sd_card(sdmmc_card_t **out_card) {
    esp_err_t ret = ESP_OK;
    sdmmc_card_t* card = (sdmmc_card_t *)malloc(sizeof(sdmmc_card_t));
    if (card == NULL) {
        ESP_LOGE(SD_TAG, "Failed to allocate sdmmc_card_t structure");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    // Initialize the interface
    ret = sdmmc_host_init();
    ESP_ERROR_CHECK(ret);
    ret = sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot_config_g);
    ESP_ERROR_CHECK(ret);

    ret = sdmmc_card_init(&host_g, card);
    if (ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "Failed to initialize SD card (%s)", esp_err_to_name(ret));
        ESP_LOGE(SD_TAG, "If you were using SDMMC and switched to SPI reinsert the SD card or power cycle the board");
        free(card);
        ESP_ERROR_CHECK(ret);
    }
    ESP_LOGI(SD_TAG, "SD card mounted - raw access");
    sdmmc_card_print_info(stdout, card);

    *out_card = card;

    return ret;
}
// Denitializes the card, and SDMMC peripheral
void deinit_sd_card(sdmmc_card_t **card) {
    // Unmount SD card
    sdmmc_host_deinit();

    free(*card);
    *card = NULL;
}
// ----- ISRs -----
static void IRAM_ATTR start_isr_handler(void* arg) {
    // give notice to adc task, which initializes the adc module with sample frequency, and runs until all data read.
    if (gpio_get_level(START_BUT) == 1) {
        return; // not the proper time to take interrupt
    }   // else continue with debounce scheme
    // get ADC handler from arg
    my_handlers_t *handles = (my_handlers_t*) arg;
    // adc_continuous_handle_t *adc_handle = handles->adc_handle;
    // debounce scheme first.
    // gptimer_handle_t *t_handle = handles->gptimer;
    gpio_intr_disable(START_BUT);   // logical error: had wrong GPIO tag
    // gptimer_set_raw_count(*t_handle, 0);
    gptimer_start(debounce_timer);

    // provide visual output of pressing button.
    led_state = !led_state;
    gpio_set_level(GREEN_LED, led_state);
    
    // esp_err_t ret;
    BaseType_t mustYield = pdFALSE;
    // first see if ADC is off, and check that handle is null (both should follow each other)
    if (!sample_state) {
        // notify the adc task to initialize adc module, and start sampling.
        vTaskNotifyGiveFromISR(start_task_handle, &mustYield);
    }   //else it is already running, don't do anything.
    // void type doesn't need return...
}
static void IRAM_ATTR aux_isr_handler(void* arg) {
    // debounce scheme first.
    if (gpio_get_level(AUX_BUT) == 1) {
        return;
    }   // else continue to debounce
    // gptimer_handle_t *t_handle = ((my_handlers_t*)arg)->gptimer;
    gpio_intr_disable(AUX_BUT); // also had logical error (had ROTARY_SWITCH instead)
    // gptimer_set_raw_count(*t_handle, 0);
    gptimer_start(debounce_timer);
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
    // gptimer_handle_t *t_handle = (gptimer_handle_t*) args;  //get timer handle from args
    gpio_intr_disable(ROTARY_SWITCH);   // prevent other interrupts
    // gptimer_set_raw_count(*t_handle, 0);
    gptimer_start(debounce_timer);
    
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
static bool IRAM_ATTR on_counter_watch(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_data) {
    BaseType_t higherWoken = pdFALSE;
    // send event data to a handler
    count_event_data send = {
        .count = (edata->watch_point_value),
        .handle = unit,
    };
    xQueueSendFromISR((QueueHandle_t) user_data, &send, &higherWoken);
    return (higherWoken == pdTRUE);
}
static bool IRAM_ATTR on_debounce_alarm(gptimer_handle_t handle, const gptimer_alarm_event_data_t *edata, void* user_data) {
    // re-enable button interrupts, after stopping the timer
    ESP_ERROR_CHECK(gptimer_stop(handle));
    gpio_intr_enable(ROTARY_SWITCH);
    gpio_intr_enable(START_BUT);
    gpio_intr_enable(AUX_BUT);
    
    return pdFALSE; // just assume a higher priority task wasn't awoken. proper implementation could use a notification to main task? 
}  
/**
 * ISR / Callback when segment alarm is triggered. Perform the following tasks:
 * - Read PCNTs
 * - Read duration timer's count
 * - Save these into 1 structure
 * - Place into buffer to export to SD card (send to another task)
 */
static bool IRAM_ATTR on_segment_alarm(gptimer_handle_t handle, const gptimer_alarm_event_data_t *edata, void* user_data) {
    BaseType_t awoke = pdFALSE;
    #pragma region 
    // int32_t pulse_counts[4] = {0};
    // ESP_ERROR_CHECK(pcnt_unit_get_count(counter_handle1, &pulse_counts[0]));
    // ESP_ERROR_CHECK(pcnt_unit_get_count(counter_handle2, &pulse_counts[1]));
    // ESP_ERROR_CHECK(pcnt_unit_get_count(counter_handle3, &pulse_counts[2]));
    // ESP_ERROR_CHECK(pcnt_unit_get_count(counter_handle4, &pulse_counts[3]));
     
    // uint64_t time = 0;
    // ESP_ERROR_CHECK(gptimer_get_raw_count(handle, &time));    
    // pulse_data data = { 
    //     .counts = {
    //         pulse_counts[0],
    //         pulse_counts[1],
    //         pulse_counts[2],
    //         pulse_counts[3]
    //     },
    //     .timer_count = time,
    // };
    // send data to copy task
    // xQueueSendFromISR(copy_queue, &data, &awoke);
    #pragma endregion
    vTaskNotifyGiveFromISR(copy_sample_handle, &awoke);
    return (awoke == pdTRUE);
}
/**
 * ISR / Callback when duration alarm is triggered. Performs these tasks:
 * - Stop segment_timer
 * - Stop duration timer
 * - Notify transfer task to export the data [from sample task]
 */
static bool IRAM_ATTR on_duration_alarm(gptimer_handle_t handle, const gptimer_alarm_event_data_t *edata, void* user_data) {
    BaseType_t awoke = pdFALSE;
    gptimer_stop(segment_timer);
    gptimer_stop(duration_timer);

    vTaskNotifyGiveFromISR(start_task_handle, &awoke);
    // vTaskResume(start_task_handle);
    return (awoke == pdTRUE); 
}
// ----- Task Functions -----
/**
 * Task function for handling the start of a sampling session
 */
static void sample_start_task(void* args) {
    // Initialize helpers here
    esp_err_t ret = ESP_OK;
    // counter handles are global, gptimer handles as well

    char log_file[128] = {0};   // buffer to store file path for log
    char temp_data[256] = {0};  // buffer to store data to write to log file

    ESP_LOGD(START_TASK_TAG, "sample_start_task initialized.");
    while (1) {
        ESP_LOGI(START_TASK_TAG, "Start sample task waiting for notification");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // wait for notify from start button.
        if (pupil_path_state) {
            // flowchart has this constraint. technically data can still be acquired, but tracking would not have proper image
            ESP_LOGW(START_TASK_TAG, "Pupil imaging path on, cannot start sampling");
            continue;   // will go wait again
        }
        // steps: prepare any data handling, clear PCNTs, start waveplate, config timers, and enable PCNTs
        ESP_LOGI(START_TASK_TAG, "Preparing data management...");   // these logs serve as checkpoints
        // Get sample_num and sample_start_sector from NVS
        ret = get_nvs_uint(SAMPLE_NUM_NVS, &sample_num);
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            // set sample num
            ESP_LOGW(START_TASK_TAG, "Setting sample_num to 0");  //warning or debug level?
            sample_num = 0;
        } else if (ret != ESP_OK) {
            // some other error occured, break out?
            ESP_LOGE(START_TASK_TAG, "NVS get error, aborting ADC conversion");
            continue;
        }
        
        ret = get_nvs_uint(SAMPLE_START_NVS, &sample_start_sector);
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(START_TASK_TAG, "Setting sample_starting_sector to default");
            sample_start_sector = DEFAULT_STARTING_SECTOR;
        } else if (ret != ESP_OK) {
            ESP_LOGE(START_TASK_TAG, "NVS get error, aborting ADC conversion");
            // realistically, we can just set it to default, but I guess it's to avoid data overwritting
            continue;
        }
        sample_sector = sample_start_sector; 
        
        // --- anything else?
        
        suppress_logs(ESP_LOG_INFO);    // suppress the logs!
        // esp_log_level_set(SAMPLE_COPY_TAG, ESP_LOG_DEBUG);
        esp_log_level_set(TRANSFER_TAG, ESP_LOG_DEBUG);

        ESP_LOGI(START_TASK_TAG, "Clearing PCNT counts...");
        pcnt_unit_clear_count(counter_handle1);
        pcnt_unit_clear_count(counter_handle2);
        pcnt_unit_clear_count(counter_handle3);
        pcnt_unit_clear_count(counter_handle4);
        
        // Configure GPTimer alarms (resolution is 10MHz [seg] and 10kHz [dur])
        ESP_LOGI(START_TASK_TAG, "Configuring timer alarms...");
        gptimer_alarm_config_t alarm_conf = {
            .alarm_count = (10000000/sample_frequency), /* integer division */
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true,
        };
        ret = gptimer_set_alarm_action(segment_timer, &alarm_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(START_TASK_TAG, "Error setting segment timer alarm action: %s", esp_err_to_name(ret));
            continue;
        }

        alarm_conf.alarm_count = 10*sample_duration;    // dur is in ms. 10 ticks per ms.
        alarm_conf.flags.auto_reload_on_alarm = false;
        ret = gptimer_set_alarm_action(duration_timer, &alarm_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(START_TASK_TAG, "Error setting duration timer alarm action: %s", esp_err_to_name(ret));
            continue;
        }
        ESP_LOGI(START_TASK_TAG, "Enabling Timers");
        
        // explicitly set the timer counts
        gptimer_set_raw_count(duration_timer, 0);
        gptimer_set_raw_count(segment_timer, 0);
        // enable the timers. They are ready now.
        ESP_ERROR_CHECK(gptimer_enable(duration_timer));
        ESP_ERROR_CHECK(gptimer_enable(segment_timer));

        // TODO: send Waveplate commands. configured in main, so just go HOME then BW
        ESP_LOGI(START_TASK_TAG, "Starting waveplate rotation");
        send_waveplate_command("ho", 0, 1); // home
        vTaskDelay(10/portTICK_PERIOD_MS);
        read_waveplate_response();

        send_waveplate_command("bw", 0,0);  // rotate backwards (CCW)
        vTaskDelay(10/portTICK_PERIOD_MS);
        read_waveplate_response();

        uint64_t tic = esp_cpu_get_cycle_count();
        // Start pulse counters
        ESP_LOGI(START_TASK_TAG, "Starting pulse counters");
        ESP_ERROR_CHECK(pcnt_unit_start(counter_handle1));
        ESP_ERROR_CHECK(pcnt_unit_start(counter_handle2));
        ESP_ERROR_CHECK(pcnt_unit_start(counter_handle3));
        ESP_ERROR_CHECK(pcnt_unit_start(counter_handle4));

        // Start sampling (start timers)
        sample_state = true;
        red_led_state = true;
        gpio_set_level(RED_LED, red_led_state);

        ret = gptimer_start(segment_timer);
        if (ret != ESP_OK) {
            ESP_LOGE(START_TASK_TAG, "Error starting segment timer! (%s)", esp_err_to_name(ret));
            send_waveplate_command("st", 0, 0); // stop waveplate to prevent it from spinning indefinitely!
            vTaskDelay(100/portTICK_PERIOD_MS);
            read_waveplate_response();
            continue;
        }
        
        ret = gptimer_start(duration_timer);
        if (ret != ESP_OK) {
            ESP_LOGE(START_TASK_TAG, "Error starting duration timer! (%s)", esp_err_to_name(ret));
            send_waveplate_command("st", 0, 0); // stop waveplate to prevent it from spinning indefinitely!
            vTaskDelay(100/portTICK_PERIOD_MS);
            read_waveplate_response();
            continue;
        }
        uint64_t toc = esp_cpu_get_cycle_count();
        ESP_LOGD(START_TASK_TAG, "Elapsed clocks from waveplate start to timer starts: %"PRIu64"", toc-tic);
        // Suspend this task to save resources (and wait)
        // vTaskSuspend(NULL);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // wait for duration timer to awaken this task!

        sample_state = false;
        red_led_state = false;
        gpio_set_level(RED_LED, red_led_state);
        ESP_LOGI(START_TASK_TAG, "Sampling finished.");
        // xTaskNotifyGive(copy_sample_handle);    // notify copy that sample is over
        vTaskSuspend(NULL);    // wait until copy finished

        // After re-awakening, disable timers so they can be adjusted later
        gptimer_disable(segment_timer); // assumes timer is stopped!
        gptimer_disable(duration_timer);

        // stop PCNTs, they will be cleared at start of this task
        ESP_ERROR_CHECK(pcnt_unit_stop(counter_handle1));
        ESP_ERROR_CHECK(pcnt_unit_stop(counter_handle2));
        ESP_ERROR_CHECK(pcnt_unit_stop(counter_handle3));
        ESP_ERROR_CHECK(pcnt_unit_stop(counter_handle4));

        // stop waveplate
        send_waveplate_command("st", 0, 0);
        vTaskDelay(100/portTICK_PERIOD_MS);
        read_waveplate_response();

        // Facilitate data transfer
        // save sample_start_sector in a file with sample num. requires total number of sectors written
        #pragma region 
        uint32_t num_sectors = sample_sector - sample_start_sector;
        snprintf(log_file, 128, "%s/samp%03lu.txt", base_path, sample_num);
        snprintf(temp_data, 256, "sample_num: %lu\nstart_sector: %lu\nnum_sectors: %lu\nstop index: %u\nsample_freq: %lu\nsample_duration: %lu\n",
                sample_num, sample_start_sector, num_sectors, /* end - start = total sectors written */
                end_sample_buf_idx, sample_frequency, sample_duration);     // now includes the frequency and duration. Important metadata!
        
        ESP_LOGD(START_TASK_TAG, "log_file: %s\ntemp_data:\n%s", log_file, temp_data);     // log debug information
        // if (num_sectors*512 != bytes_read) {
        //     ESP_LOGW(START_TASK_TAG, "Failed sanity check: sectors = %lu, sectors*512 = %lu vs. bytes read = %lu",
        //                  num_sectors, num_sectors*512, bytes_read);
        //     unsuppress_logs(SUPPRESS_LEVEL);
        //     continue;
        // }
        ret = write_log_file(log_file, temp_data);   // log/write the information to the file. (overwrite if necessary)
        if (ret == ESP_FAIL) {
            ESP_LOGE(START_TASK_TAG, "Failed to open and write to log file (%s). Sample sector address not saved!", log_file);
            unsuppress_logs(SUPPRESS_LEVEL);
            continue;
        }

        // else it was successful
        ESP_LOGI(START_TASK_TAG, "Sample #%lu address saved to flash successfully", sample_num);
        // update sample_start_sector for next time
        sample_start_sector = sample_sector + 512*2;    // give a little buffer between 
        sample_sector = sample_start_sector;
        // store sample_start_sector into NVS
        nvs_set_u32(my_nvs_handle, SAMPLE_START_NVS, sample_start_sector);
        sample_num++; 
        nvs_set_u32(my_nvs_handle, SAMPLE_NUM_NVS, sample_num); // store sample_num into NVS for next sample run.
        ret = nvs_commit(my_nvs_handle);  // commit the changes
        if (ret != ESP_OK) {
            ESP_LOGE(START_TASK_TAG, "Error committing sector information to nvs! (%s)", esp_err_to_name(ret));
            continue;
        }
        ESP_LOGD(START_TASK_TAG, "Sample variables saved to NVS successfully");
        #pragma endregion
        
        ESP_LOGI(START_TASK_TAG, "Delaying transfer task to read logs (5s)");
        vTaskDelay(5000/portTICK_PERIOD_MS);

        // inform/start transfer task
        xTaskNotifyGive(sample_transfer_task_handle);
        vTaskSuspend(NULL); // transfer task will resume!

        unsuppress_logs(ESP_LOG_DEBUG);    // un-suppress the logs!
        // restore logs if not suppressed.
        if (!logs_suppressed) {
            ESP_LOGI(START_TASK_TAG, "Resuming monitor logging");
            esp_log_level_set(MONITOR_TASK_TAG, ESP_LOG_DEBUG);
        }
    }
}
/**
 * Task to handle data from timer ISR. Performs:
 * - takes data and puts into buffer
 * - when buffer fills, writes it to SD card
 * If using buffer proves too difficult, just write individual data structures to a single sector on SD card
 */
static void sample_copy_task(void* args) {
    // needs queue handle and SD card handle from args
    my_handlers_t* handles = (my_handlers_t*) args;
    // gptimer_handle_t* dur_timer = handles->gptimer;
    // TODO: add queue handle to my_handles? no, just make it global. ([TODO] initialized in main)
    // pulse_data q_receive = { 0 };
    ESP_LOGD(SAMPLE_COPY_TAG, "Free heap memory: %lu", esp_get_free_heap_size());
    pulse_data *buffer = (pulse_data*) heap_caps_malloc(PULSE_DATA_BUF_LEN * sizeof(pulse_data), MALLOC_CAP_8BIT);
    // pulse_data* buffer = (pulse_data*) calloc(PULSE_DATA_BUF_LEN, sizeof(pulse_data)); // 20B per struct. 
    if (buffer == NULL) {
        ESP_LOGE(SAMPLE_COPY_TAG, "Error allocating buffer for pulse data!");
        return;
    }
    // size_t buf_len = 12*512;    // might make this into a macro definition instead
    bool runOnce = false;
    size_t buf_idx = 0;
    while(1) {
        // wait until data is received from on_segment_alarm (via notification)
        ESP_LOGD(SAMPLE_COPY_TAG, "Copy task waiting for notification from timer...");
        // configTASK_NOTIFICATION_ARRAY_ENTRIES    // hmm, it equals 1, so only 0 is valid? Requires different solution I guess. 
        if (ulTaskNotifyTake(pdTRUE, 1000/portTICK_PERIOD_MS) ) {
            ESP_LOGD(SAMPLE_COPY_TAG, "Getting data to store");
            int32_t pulse_counts[4] = {0};
            ESP_ERROR_CHECK(pcnt_unit_get_count(counter_handle1, &pulse_counts[0]));
            ESP_ERROR_CHECK(pcnt_unit_get_count(counter_handle2, &pulse_counts[1]));
            ESP_ERROR_CHECK(pcnt_unit_get_count(counter_handle3, &pulse_counts[2]));
            ESP_ERROR_CHECK(pcnt_unit_get_count(counter_handle4, &pulse_counts[3]));
            
            uint64_t time = 0;
            ESP_ERROR_CHECK(gptimer_get_raw_count(duration_timer, &time));    
            ESP_LOGD(SAMPLE_COPY_TAG, "Counts: %lu, %lu, %lu, %lu; Time: %llu", pulse_counts[0], pulse_counts[1],
                pulse_counts[2],pulse_counts[3], time); // verify what data comes in. 
            // pulse_data data = { 
            //     .counts = {
            //         pulse_counts[0],
            //         pulse_counts[1],
            //         pulse_counts[2],
            //         pulse_counts[3]
            //     },
            //     .timer_count = time,
            // };
            if (buf_idx < PULSE_DATA_BUF_LEN) {
                buffer[buf_idx].counts[0] = pulse_counts[0]; // should be a copy, and not pass by ref. 
                buffer[buf_idx].counts[1] = pulse_counts[1];
                buffer[buf_idx].counts[2] = pulse_counts[2];
                buffer[buf_idx].counts[3] = pulse_counts[3];
                buffer[buf_idx].timer_count = time;
                buf_idx++;
            }  // if doesn't work, initialize it like data above
            else {
                // write buffer to SD card and hope it is fast enough!
                ESP_LOGI(SAMPLE_COPY_TAG, "Writing buffer to SD card");
                // use sample_start_sector to write buffer to SD card via raw access
                size_t num_sectors = PULSE_DATA_BUF_LEN / 512; // sector size is 512 B. 
                ESP_ERROR_CHECK(sdmmc_write_sectors(handles->card, buffer, sample_sector, num_sectors));
                sample_sector += num_sectors; //update sector start for next buffer (global variable)
                // transfer is complete, store the data from this request
                buf_idx = 0;
                buffer[buf_idx].counts[0] = pulse_counts[0]; // should be a copy, and not pass by ref. 
                buffer[buf_idx].counts[1] = pulse_counts[1];
                buffer[buf_idx].counts[2] = pulse_counts[2];
                buffer[buf_idx].counts[3] = pulse_counts[3];
                buffer[buf_idx].timer_count = time;
                buf_idx++;
            }
            runOnce = true;     // must be notified once to be true
        } // here if it return 0, ie no notifications in last second
        else if (!sample_state && runOnce) {    // must be notified once, and the sample_state must be false. sample_state is true before this task is notified.
            runOnce = false;    // prevent next time until new data acquired. 
            ESP_LOGI(SAMPLE_COPY_TAG, "Copying leftover data to SD card");
            // after one second, check if sampling has stopped to write rest of the buffer to SD card
            // needs to be coordinated with other tasks, use notification instead
            // export current buffer size to SD card

            // buf_idx is incremented after data is written to that index. Thus, no need to add one here. (buf_idx = 0 means no extra data)
            size_t num_sectors = ((buf_idx)*sizeof(pulse_data) +511) / 512; // round data up to next sector. 
            if (num_sectors < 1) {
                ESP_LOGD(SAMPLE_COPY_TAG, "setting num_sectors to 1 (too small)");
                num_sectors = 1;
            }
            ESP_ERROR_CHECK(sdmmc_write_sectors(handles->card, buffer, sample_sector, num_sectors));
            sample_sector += num_sectors; //update sector start for next buffer (global variable)
            end_sample_buf_idx = buf_idx;   // store the ending buffer index to terminate output later to prevent garbage being sent to python.

            buf_idx = 0;    // reset index for next time
            vTaskResume(start_task_handle);
        }
        // if(xQueueReceive(copy_queue, &q_receive, 1000/portTICK_PERIOD_MS) == pdTRUE) {
        //     // store q_receive into buffer
        //     if (buf_idx < PULSE_DATA_BUF_LEN) { /* if buffer has room */
        //         buffer[buf_idx++] = q_receive;  // increment index after statement executes!
        //
        //     } else {    // BUG: only runs if we sample long enough to fill a buffer!
        //         // write buffer to SD card and hope it is fast enough!
        //         // use sample_start_sector to write buffer to SD card via raw access
        //         size_t num_sectors = PULSE_DATA_BUF_LEN / 512; // sector size is 512 B. 
        //         sdmmc_write_sectors(handles->card, buffer, sample_sector, num_sectors);
        //         sample_sector += num_sectors; //update sector start for next buffer (global variable)
        //         // transfer is complete, store the data from this request
        //         buf_idx = 0;
        //         buffer[buf_idx++] = q_receive;  // post-increment the index
        //     }
        // } else { /* solution for not having a large enough sample buffer */
        //     // after one second, check if sampling has stopped to write rest of the buffer to SD card
        //     // needs to be coordinated with other tasks, use notification instead
        //     if (ulTaskGenericNotifyTake(1,pdTRUE, 0)){
        //         // export current buffer size to SD card
        //         size_t num_sectors = buf_idx*sizeof(pulse_data) / 512; // sector size is 512 B. 
        //         sdmmc_write_sectors(handles->card, buffer, sample_sector, num_sectors);
        //         sample_sector += num_sectors; //update sector start for next buffer (global variable)
        //
        //         xTaskNotifyGive(start_task_handle);
        //     } // else just continue back to queue
        // }
    }
    free(buffer);
}
/**
 * Task to transfer data from SD card to awaiting Python program. Performs:
 * - Gets sample information from files stored in flash
 * - Uses information to read sectors from SD card
 * - Prints to terminal for Python to read over UART serial bridge
 */
static void sample_transfer_task(void* args) {
    esp_err_t ret;
    // will need the SD card from args
    my_handlers_t* handles = (my_handlers_t*) args;
    sdmmc_card_t* card = handles->card;

    char data_buf[256] = {0};   // buffer to store the contents of the log file
    char log_file[128] = {0};    // buffer to hold log file
    uint32_t num_sectors = 0;  // number of sectors

    ESP_LOGI(TRANSFER_TAG, "Transfer task successfully initialized.");
    while (1) {
        ESP_LOGI(TRANSFER_TAG, "Transfer task waiting for notification...");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // wait until notification from start_task

        // start by getting and opening the file from flash.
        snprintf(log_file, 128, "%s/samp%03lu.txt", base_path, sample_num - 1); // sample_num was incremented before this loop, go back one
        ESP_LOGD(TRANSFER_TAG, "Openning log file '%s'", log_file);
        FILE* log_fp = fopen(log_file, "rb");    // open for reading (binary). File written with just write mode and binary.
        if (log_fp == NULL) {
            ESP_LOGE(TRANSFER_TAG, "Error openning log file (%s)", log_file);
            ESP_LOGE(TRANSFER_TAG, "NOTE: Sample results not exported!");
            // free(result_buf);   // don't forget to free it every time!
            vTaskResume(start_task_handle);   // don't forget to do this!
            continue;
        }   // else the file was openned successfully!
        // Data written: "sample_num: %lu\nstart_sector: %lu\nnum_sectors: %lu\nstop index: %u\nsample_freq: %lu\nsample_duration: %lu\n"
        ret = read_line_file(log_fp, data_buf, 256);
        // check ret val?
        ESP_LOGD(TRANSFER_TAG, "1st line read from log file: '%s'", data_buf);
        uint32_t d_num, sample_start, end_idx, s_freq, s_dur; // vars to read data into. 
        sscanf(data_buf, "%*s %lu", &d_num);  // %*s will read a string and discard. the ':' will be read and compared, if matches, continues.
        
        if (d_num != sample_num -1) /* Sanity check */ {
            ESP_LOGE(TRANSFER_TAG, "Housten, we have a problem! (expected sample number (global-1: %lu) != sample_number from file (%lu)",
                 sample_num -1, d_num);
            // free(result_buf);
            fclose(log_fp);
            vTaskResume(start_task_handle);
            continue;
        }
        ret = read_line_file(log_fp, data_buf, 256);
        sscanf(data_buf, "%*s %lu", &sample_start);    // gets starting sector address
        
        ret = read_line_file(log_fp, data_buf, 256);
        sscanf(data_buf, "%*s %lu", &num_sectors);  // gets number of sectors to read

        ret = read_line_file(log_fp, data_buf, 256);
        sscanf(data_buf, "%*s %lu", &end_idx);      // gets the ending index of the buffers

        ret = read_line_file(log_fp, data_buf, 256);
        sscanf(data_buf, "%*s %lu", &s_freq);       // sample frequency

        ret = read_line_file(log_fp, data_buf, 256);
        sscanf(data_buf, "%*s %lu", &s_dur);        //sample duration
        fclose(log_fp); // not super critical for reading only, but good practice.

        // log debug information
        ESP_LOGD(TRANSFER_TAG, "Metadata from file: Sample #%lu, start: %lu, size: %lu, end idx: %lu, freq: %lu, dur: %lu", 
                                                            d_num, sample_start, num_sectors, end_idx, s_freq, s_dur);
        if (end_idx < 0) {
            ESP_LOGW(TRANSFER_TAG, "Ending index less than zero, setting to zero");
            end_idx = 0;
        }
        // Now we can read over the sectors, printing each of them to the terminal, with some delay between them to help Python out.
        pulse_data * tmp_buf = (pulse_data*) calloc(PULSE_DATA_BUF_LEN, sizeof(pulse_data));
        if (tmp_buf == NULL) {
            ESP_LOGE(TRANSFER_TAG, "Error, not enough memory for buffer! Need %"PRIu16" bytes; free heap: %"PRIu32"", 
                PULSE_DATA_BUF_LEN*sizeof(pulse_data), esp_get_free_heap_size());
            free(tmp_buf);  // just in case
            continue;
        }
        
        // Send sample metadata to python
        printf("%s\nfreq: %"PRIu32"; duration: %"PRIu32"\n", PY_DATA, sample_frequency, sample_duration);
                
        uint32_t read_sector = sample_start;
        // prepare to loop over a set number of sectors
        // total sectors / number of sectors per buffer *full*
        // size_t step_sectors = num_sectors / (PULSE_DATA_BUF_LEN/512);   // ISSUE: last buffer might not be full! 
        // size_t step_sectors = PULSE_DATA_BUF_LEN / 512;
        // Logical Bug: step_sectors should be DATA_BUF_LEN / 512 (6), and total number of iterations = num_sectors/6
        // if (step_sectors == 0) {
        //     ESP_LOGE(TRANSFER_TAG, "step_sectors is zero! Breaking loop");
        //     free(tmp_buf);
        //     break;
        // }
        // size_t last_set = num_sectors % (PULSE_DATA_BUF_LEN/512);   // gives the number of sectors that were written at end of copy task

        size_t sectors_to_read = num_sectors;
        size_t buffer_sector_size = PULSE_DATA_BUF_LEN / 512;

        // determine the number of loop iterations (to get data in a single function within Python)
        size_t iterations = num_sectors / buffer_sector_size + 1;
        // send to Python
        printf("%s\nIterations: %u\n", PY_TAG, iterations);

        while (sectors_to_read >= buffer_sector_size) {
            // read 6 sectors (a full buffers worth)
            ret = sdmmc_read_sectors(card, tmp_buf, read_sector, buffer_sector_size);
            if (ret != ESP_OK) {
                ESP_LOGE(TRANSFER_TAG, "Error reading SD card sectors for addr: %lu, and size: %u", sample_start, buffer_sector_size);
                free(tmp_buf);
                break;  // breaks this loop only.
            }
            // Find number of lines to read (how many pulse_data structures are in one buffer)
            size_t numLines = PULSE_DATA_BUF_LEN / sizeof(pulse_data);  //6*512/24 = 128
            printf("Num Lines: %u\n", numLines);  // inform Python!
            // print the data out. 
            for (size_t i = 0; i < numLines; i++) {
                // format as timer count, then sensors 1-4
                printf("%6llu, %ld, %ld, %ld, %ld\n", tmp_buf[i].timer_count, tmp_buf[i].counts[0],tmp_buf[i].counts[1],
                            tmp_buf[i].counts[2], tmp_buf[i].counts[3]);
            }

            // update loop iterators
            read_sector += buffer_sector_size;
            sectors_to_read -= buffer_sector_size;
            vTaskDelay(10/portTICK_PERIOD_MS);  // give python 10ms to catchup
        }
        if (sectors_to_read > 0) {
            // read the remaining sectors
            ret = sdmmc_read_sectors(card, tmp_buf, read_sector, sectors_to_read);
            if (ret != ESP_OK) {
                ESP_LOGE(TRANSFER_TAG, "Error reading SD card sectors for addr: %lu, and size: %u", sample_start, buffer_sector_size);
                free(tmp_buf);
                break;  // breaks this loop only.
            }
            // Find number of lines to read (how many `pulse_data` structs have been read)
            // size_t numLines = sectors_to_read*512 / sizeof(pulse_data);
            size_t numLines = end_idx;
            printf("Num Lines: %u\n", numLines);  // inform Python!
            // print the data out. 
            for (size_t i = 0; i < numLines; i++) {
                // format as timer count, then sensors 1-4
                printf("%6llu, %ld, %ld, %ld, %ld\n", tmp_buf[i].timer_count, tmp_buf[i].counts[0],tmp_buf[i].counts[1],
                            tmp_buf[i].counts[2], tmp_buf[i].counts[3]);
            }

            // update loop iterators
            read_sector += sectors_to_read;
            // sectors_to_read -= sectors_to_read;  // unnecessary, not in loop
            vTaskDelay(10/portTICK_PERIOD_MS);  // give python 10ms to catchup
        }

        // tell python number of loops, and how much per loop? 
        // for (size_t transferred = 0; transferred < num_sectors; transferred += step_sectors) {
        //     // read from d_start into the tmp_buf  
        //     ret = sdmmc_read_sectors(card, tmp_buf, d_start, step_sectors);
        //     if (ret != ESP_OK) {
        //         ESP_LOGE(TRANSFER_TAG, "Error reading SD card sectors for addr: %lu, and size: %u", d_start, step_sectors);
        //         free(tmp_buf);
        //         break;  // breaks this for loop only.
        //     }
        //     printf("%s\nNum Lines: %u\n", PY_TAG, PULSE_DATA_BUF_LEN);
        //     // print data to terminal
        //     for (size_t i = 0; i < PULSE_DATA_BUF_LEN; i++) {
        //         // format as timer count, then sensors 1-4
        //         printf("%6llu, %ld, %ld, %ld, %ld\n", tmp_buf[i].timer_count, tmp_buf[i].counts[0],tmp_buf[i].counts[1],
        //                     tmp_buf[i].counts[2], tmp_buf[i].counts[3]);
        //     }
        //
        //     d_start += step_sectors;    // update address to read from next
        //     vTaskDelay(10/portTICK_PERIOD_MS);  // give python 10ms to catchup
        // }
        // // handle the leftovers, if any
        // if (last_set != 0) {
        //     ret = sdmmc_read_sectors(card, tmp_buf, d_start, last_set);
        //     if (ret != ESP_OK) {
        //         ESP_LOGE(TRANSFER_TAG, "Error reading SD card sectors for addr: %lu, and size: %u", d_start, step_sectors);
        //         free(tmp_buf);
        //         continue;
        //     }
        //     size_t lines = last_set*512/sizeof(pulse_data); // I believe that to be the correct # of lines
        //     ESP_LOGD(TRANSFER_TAG, "Last set of data (lines: %u)", lines);
        //     printf("%s\nNum Lines: %u\n", PY_TAG, lines);  
        //     // print data to terminal
        //     for (size_t i = 0; i < lines; i++) {
        //         // format as timer count, then sensors 1-4
        //         printf("%6llu, %ld, %ld, %ld, %ld\n", tmp_buf[i].timer_count, tmp_buf[i].counts[0],tmp_buf[i].counts[1],
        //                     tmp_buf[i].counts[2], tmp_buf[i].counts[3]);
        //     }
        //     d_start += last_set;    // update address for a sanity check
        // }
        
        // sanity check if data read matches expected amount.
        if (read_sector != sample_start+num_sectors) {
            ESP_LOGE(TRANSFER_TAG, "Sanity Check failed! Ending sector doesn't match expected");
            ESP_LOGE(TRANSFER_TAG, "Start Sector: %lu, Num Sectors: %lu, End Sector: %lu", read_sector, num_sectors, sample_start);
            // free(tmp_buf);
            // continue;
        }
        // done reading data
        free(tmp_buf);  // release buffer, not needed until next time.
        vTaskResume(start_task_handle);
    }
}
// Task to periodically display debug or info messages on variable states, etc. Also updates vars from PCNT events.
static void monitor_var_task(void* args) {
    ESP_LOGI(MONITOR_TASK_TAG, "Variable Monitoring Task started");
    my_pcnt_t *user_data = (my_pcnt_t*) args;       // get data from args
    // Counts for rotary encoder
    int pulse_count = 0;
    int event_count = 0;    //passed by reference to xQueueReceive
    while(1) {
        // 1000ms = 1s delay, queue waits for 1s. 
        // vTaskDelay(1000 / portTICK_PERIOD_MS);  // avoid watchdog and spamming of logs.

        // report PCNT count; problem: will only update every second because of delay to prevent log spamming. 
        if (xQueueReceive(*(user_data->event_queue), &event_count, 1000/portTICK_PERIOD_MS)) { // blocks for 1s waiting to see if update to counter
            ESP_LOGI(MONITOR_TASK_TAG, "Rotary PCNT Event Count: %d", event_count);
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
            // add some logic to prevent log spam without a delay that affects the other functionality?
            ESP_ERROR_CHECK(pcnt_unit_get_count(*(user_data->pcnt_handle), &pulse_count));
            ESP_LOGI(MONITOR_TASK_TAG, "Rotary PCNT Current Count: %d", pulse_count);
            
            // debug level logs
            // ESP_LOGD(MONITOR_TASK_TAG, "adc_state: %d", adc_state);  // visually shown with LED
            ESP_LOGD(MONITOR_TASK_TAG, "servo state: %d", pupil_path_state);    // not shown with LED
            // ESP_LOGD(MONITOR_TASK_TAG, "bytes_read: %"PRIu32"", bytes_read);
            ESP_LOGD(MONITOR_TASK_TAG, "Free heap memory: %"PRIu32"", esp_get_free_heap_size());
            // ESP_LOGD(MONITOR_TASK_TAG, "menuIndex: %d", menuIndex);     // visually shown on LCD, but good as debug. 
            // ESP_LOGD(MONITOR_TASK_TAG, "base_pos: %"PRIu16"", base_pos);    
        }
    }
}
// Task to service servo updates
static void servo_task(void* args) {
    // needs comparator to call mcpwm_comparator
    my_servos_t *servos = (my_servos_t*) args;

    ESP_LOGI(SERVO_TASK_TAG, "Servo task entering loop");
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // wait for notification from ISR
        if (pupil_path_state) {
            ESP_LOGI(SERVO_TASK_TAG, "Pupil Image ON, servos set to max");

            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*(servos->comp1), example_angle_to_compare(SERVO1_ON)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*(servos->comp2), example_angle_to_compare(SERVO2_ON)));
            vTaskDelay(pdMS_TO_TICKS(500));    // give servo time to get to position
        } else {
            ESP_LOGI(SERVO_TASK_TAG, "Pupil Image OFF, servos set to min");

            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*(servos->comp1), example_angle_to_compare(SERVO1_OFF)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*(servos->comp2), example_angle_to_compare(SERVO2_OFF)));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

// Task function to monitor UART events and handle received data
static void uart_event_task(void *pvParameters) {
    ESP_LOGI(UART_MON_TAG, "UART event task created successfully");
    uart_event_t event;                            // UART event structure to hold event data
    char* dtmp = (char*) malloc(UART_READ_BUF);   // Allocate buffer for incoming data
    char* tmpStr = (char*) calloc(UART_READ_BUF, sizeof(char)); // holds the string that will be built up
    char *pStr = tmpStr;

    while (1) {                                    // Loop to continuously listen for UART events
        // Wait for an event on the UART queue
        if (xQueueReceive(pc_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, UART_READ_BUF);                 // Clear the buffer

            switch (event.type) {                  // Check the type of UART event
                case UART_DATA:                    // If data is received
                    // Read the incoming data into the buffer
                    esp_log_level_set(MONITOR_TASK_TAG, ESP_LOG_NONE);  // reduce log statements
                    esp_log_level_set(SAMPLE_COPY_TAG, ESP_LOG_WARN);
                    uart_read_bytes(PC_UART_NUM, dtmp, event.size, portMAX_DELAY);  // only ever reads 1 byte at once (even CRLF is split!)
                    dtmp[event.size] = '\0';       // Null-terminate the string
                    // printf("Received %u bytes: %s (%X)\n", event.size, (char*) dtmp, dtmp[0]);  // Print received data
                    // build up a buffer until a newline character. (if event.size is 1)
                    // actually recieves CRLF (\r\n)
                    if (dtmp[0] == '\r') {
                        printf("\n");    // put a newline as printf below doesn't.
                        // esp_log_level_set(MONITOR_TASK_TAG, ESP_LOG_DEBUG);     // restore log level before handling command.
                        // process command
                        handle_command(tmpStr, pvParameters);   // handle the command
                        memset(tmpStr, 0, UART_READ_BUF);   // clear the tmpStr
                        pStr = tmpStr;
                        if (!logs_suppressed) {   // restore log level only when not suppressed
                            esp_log_level_set(MONITOR_TASK_TAG, ESP_LOG_DEBUG);     // restore log level (does so after suppress command! lol)
                            esp_log_level_set(SAMPLE_COPY_TAG, ESP_LOG_DEBUG);
                        }
                        break;
                    }
                    // else add dtmp to tmpStr
                    // strcat(tmpStr, (char*) dtmp);
                    *pStr = dtmp[0];
                    pStr += 1;  // move pointer address up by 1 (to next position in string)
                    printf("tmpStr: %s\n", tmpStr); // TODO: make more professional? (use \r to overwrite previous tmpStr?)
                    // handle_command((char*) dtmp, pvParameters);  // Pass received data to command handler
                    break;
                case UART_PATTERN_DET:
                    // Pattern is detected. use this to detect '\n' and then read line at once?
                    ESP_LOGI(UART_MON_TAG, "UART pattern detect event reached");
                    size_t len = 0;
                    uart_get_buffered_data_len(PC_UART_NUM, &len);
                    uart_read_bytes(PC_UART_NUM, dtmp, len, 0);
                    dtmp[len-1] = 0;  // give string its null-termination (removing the newline)
                    handle_command(dtmp, pvParameters);
                    break;
            #pragma region 
                case UART_FIFO_OVF:                // If UART FIFO buffer overflows
                    printf("UART FIFO Overflow\n");
                    uart_flush_input(PC_UART_NUM);    // Clear input buffer
                    xQueueReset(pc_uart_queue);       // Reset the UART queue
                    break;

                case UART_BUFFER_FULL:             // If UART buffer is full
                    printf("UART Buffer Full\n");
                    uart_flush_input(PC_UART_NUM);    // Clear input buffer
                    xQueueReset(pc_uart_queue);       // Reset the UART queue
                    break;

                case UART_BREAK:                   // If a UART break condition occurs
                    printf("UART Break detected\n");
                    break;

                case UART_PARITY_ERR:              // If a parity error occurs
                    printf("UART Parity Error\n");
                    break;

                case UART_FRAME_ERR:               // If a frame error occurs
                    printf("UART Frame Error\n");
                    break;

                default:                           // For any unknown UART event type
                    printf("Unknown UART event type: %d\n", event.type);
                    break;
            #pragma endregion
            }
        }
    }
    // free(tmpStr);
    free(dtmp);                                    // Free the allocated memory for the buffer
    vTaskDelete(NULL);                             // Delete this task when done
}

// Command is string from console, args are handles for starting different tasks
static void handle_command(const char* command, void* args) {
    my_handlers_t * handles = (my_handlers_t*) args;
    // adc_continuous_handle_t * adc_handle = handles->adc_handle;
    // my_servos_t * servos = handles->servos;

    if (strcmp(command, "start") == 0) {           // Check if the command is "start"
        printf("Received 'start' command. Beginning operation...\n");
        // --- Add code here for start operation (simulate button press?)
        // provide visual output of command input (flash LED)
        led_state = !led_state;
        gpio_set_level(GREEN_LED, led_state);
        vTaskDelay(500/portTICK_PERIOD_MS);
        led_state = !led_state;
        gpio_set_level(GREEN_LED, led_state);
        
        // first see if ADC is off, and check that handle is null (both should follow each other)
        if (!sample_state) {
            // notify the adc task to initialize adc module, and start sampling.
            xTaskNotifyGive(start_task_handle);
        }   //else it is already running, don't do anything.
    } else if (strcmp(command, "pupil") == 0) {
        pupil_path_state = !pupil_path_state;
        ESP_LOGI(UART_MON_TAG, "Recieved Pupil Command, servo state: %s", (pupil_path_state) ? "ON":"OFF" );
        // notify servo task
        xTaskNotifyGive(servo_task_handle);
    } else if (strcmp(command, "reset samples") == 0) {
        ESP_LOGI(UART_MON_TAG, "Resetting sample sector memory address (old sample data will be overwritten!)");
        nvs_set_u32(my_nvs_handle, SAMPLE_START_NVS, DEFAULT_STARTING_SECTOR);
        nvs_set_u32(my_nvs_handle, SAMPLE_NUM_NVS, 0);
        nvs_commit(my_nvs_handle);
        // also remove flash log files? technically those are still valid (until the data is overwritten)
        // TODO: unmount, reformat, and remount flash.
        // esp_vfs_fat_spiflash_unmount_rw_wl
        ESP_LOGI(UART_MON_TAG, "Reformatting SPI flash storage...");
        esp_vfs_fat_spiflash_format_rw_wl(base_path, "storage");
        ESP_LOGI(UART_MON_TAG, "... formatting finished. Flash mounted.");
        
    } else if(strcmp(command, "suppress") == 0) {
        ESP_LOGI(UART_MON_TAG, "Supressing all logs to WARN level, and some to INFO.");
        suppress_logs(SUPPRESS_LEVEL);

    } else if (strcmp(command, "unsuppress") == 0) {
        ESP_LOGI(UART_MON_TAG, "Unsuppressing all logs to DEBUG level");
        unsuppress_logs(ESP_LOG_DEBUG); // TODO: in final implementation, info level should do.

    } else {    // If the command is more advanced, or unknown
        // split string on space, see if left is "set", "setf", or "setd"
        char* token = strtok(command, " "); // tokenize based on space
        // set <freq [kHz]> <dur [ms]>
/*set*/ if (token != NULL && strcmp(token, "set") == 0) {
            token = strtok(NULL, " ");
            if (token == NULL) {
                ESP_LOGW(UART_MON_TAG, "Invalid argument for \"set\" command. Usage: \"set <freq [Hz]> <dur [ms]>\"");
                return;
            }   // else not NULL
            int ret = sscanf(token, "%"PRIu32"", &sample_frequency);
            if (ret < 1) {
                ESP_LOGE(UART_MON_TAG, "Invalid frequency for \"set\" command (%s).", token);
                sample_frequency = DEFAULT_SAMPLE_FREQ;
                return;
            }   // otherwise, integer read sucessfully
            // part 2, read the duration
            token = strtok(NULL, " ");
            if (token == NULL) {
                ESP_LOGW(UART_MON_TAG, "Invalid argument for \"set\" command. Usage: \"set <freq [kHz]> <dur [ms]>\"");
                sample_duration = DEFAULT_DURATION;
                return;
            }   // else not NULL
            ret = sscanf(token, "%"PRIu32"", &sample_duration);
            if (ret < 1) {
                ESP_LOGE(UART_MON_TAG, "Invalid duration for \"set\" command (%s).", token);
            }

            ESP_LOGI(UART_MON_TAG, "Command read correctly, sample_frequency = %"PRIu32"; sample_duration = %"PRIu32"",
                            sample_frequency, sample_duration);
/*setf*/} else if (token != NULL && strcmp(token, "setf") == 0) {
            token = strtok(NULL, " ");
            if (token == NULL) {
                ESP_LOGW(UART_MON_TAG, "Invalid argument for \"setf\" command. Usage: \"setf <freq [kHz]>\"");
                return;
            }   // else not NULL
            int ret = sscanf(token, "%"PRIu32"", &sample_frequency);
            if (ret < 1) {
                ESP_LOGE(UART_MON_TAG, "Invalid frequency for \"setf\" command (%s).", token);
                sample_frequency = DEFAULT_SAMPLE_FREQ;
                return;
            }   // otherwise, integer read sucessfully

            ESP_LOGI(UART_MON_TAG, "Command read correctly, sample_frequency = %"PRIu32"", sample_frequency);
/*setd*/} else if (token != NULL && strcmp(token, "setd") == 0) {
            token = strtok(NULL, " ");
            if (token == NULL) {
                ESP_LOGW(UART_MON_TAG, "Invalid argument for \"setd\" command. Usage: \"setd <dur [ms]>\"");
                return;
            }   // else not NULL
            int ret = sscanf(token, "%"PRIu32"", &sample_duration);
            if (ret < 1) {
                ESP_LOGE(UART_MON_TAG, "Invalid duration for \"setd\" command (%s).", token);
                sample_duration = DEFAULT_DURATION;
                return;
            }   // otherwise, integer read sucessfully

            ESP_LOGI(UART_MON_TAG, "Command read correctly, sample_duration = %"PRIu32"", sample_duration);
/*else*/} /* else if (token != NULL && strcmp(token, "get") == 0) {
            // set global sample number, and then 
            token = strtok(NULL, " ");
            if (token == NULL) {
                ESP_LOGW(UART_MON_TAG, "Invalid argument for \"get\" command. Usage: \"get <sample number>\"");
                return;
            }
            int desired_number = 0, max_num = 0, prev = sample_num;
            int ret = sscanf(token, "%d", &desired_number);
            ESP_ERROR_CHECK(get_nvs_uint(SAMPLE_NUM_NVS, &max_num));
            if (ret < 1 || desired_number < 0 || desired_number >= max_num) {
                ESP_LOGE(UART_MON_TAG, "Invalid sample number for \"get\" command (%s). Max is %lu.", token, max_num);
                return;
            }
            sample_num = desired_number +1;
            xTaskNotifyGive(sample_transfer_task);
            // how to wait for it to finish?
            sample_num = prev; 
        } */ else {
            ESP_LOGW(UART_MON_TAG, "Unknown command received: \"%s\"", command);
        }
    }
}

// 'inline' = hint to compiler to replace function call with the code inside the function (an optimization)
static inline uint32_t example_angle_to_compare(int angle)  {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

// Simple function to read a line from a file. 
static esp_err_t simple_file_read(char *path) {
    ESP_LOGI(FLASH_TAG, "Reading file (%s)", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(FLASH_TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EX_FILE_LEN];
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(FLASH_TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

/**
 * @brief Sets all log levels to WARNINGS or higher. 
 *      Sets logs specific to this file to general_level, with some exceptions.
 * @param general_level: specifies the highest log level of log TAGs in this file
 */
static void suppress_logs(esp_log_level_t general_level) {
    ESP_LOGI(MAIN_TAG, "Suppressing logs to %d (0=Error, 5=Verbose)", general_level);
    // suppresses log levels to Warning or lower/higher
    esp_log_level_set("*", ESP_LOG_WARN);   // suppresses all monitor logs, and logs from other files
    esp_log_level_set(MONITOR_TASK_TAG, ESP_LOG_WARN);  // explicitly suppress this one.
    // make it so that select tasks have higher log levels
    esp_log_level_set(SAMPLE_COPY_TAG, ESP_LOG_WARN);   // really just needs info level
    // esp_log_level_set(START_TASK_TAG, general_level);    // start task suppressed itself!
    // esp_log_level_set(TRANSFER_TAG, general_level);
    esp_log_level_set(MAIN_TAG, general_level);     // in loop, main has no logs!
    esp_log_level_set(UART_MON_TAG, ESP_LOG_INFO);  // should always be on info!
    esp_log_level_set(SERVO_TASK_TAG, ESP_LOG_INFO);

    logs_suppressed = true;
}
/**
 * @brief Sets all log levels to global_level. 
 *          Some logs are set to INFO, while one is set to DEBUG
 * @param global_level: specifies the highest log level of all TAGs
 */
static void unsuppress_logs(esp_log_level_t global_level) {
    esp_log_level_set("*", global_level);
    esp_log_level_set(MONITOR_TASK_TAG, ESP_LOG_DEBUG);

    // In case global is < Info (warn (2) or error(1)), these should always be on info
    if (global_level < ESP_LOG_INFO) {
        esp_log_level_set(UART_MON_TAG, ESP_LOG_INFO);
        esp_log_level_set(SERVO_TASK_TAG, ESP_LOG_INFO);
        esp_log_level_set(TRANSFER_TAG, ESP_LOG_DEBUG);
        esp_log_level_set(START_TASK_TAG, ESP_LOG_INFO);
        esp_log_level_set(SAMPLE_COPY_TAG, ESP_LOG_INFO);
    }
    logs_suppressed = false;
}

static TaskHandle_t init_1_handle = NULL;
/**
 * Function to allocate resources on core 1, so interrupts are attached there.
 * This is a workaround fix since all init. code is on core 0
 */
static void core_1_initializer(void* args) {
    // starts as a task, will return at the end.
    esp_err_t ret;
    my_handlers_t * handles = (my_handlers_t*) args; //malloc(sizeof(my_handlers_t));
    // memset(handles, 0, sizeof(my_handlers_t));  // set to zero
    ESP_LOGI(MAIN_TAG, "Core 1 Initializer. Free heap memory: %"PRIu32"", esp_get_free_heap_size());
    // heap_caps_check_integrity_all(true);
    // --- GPTimer / Debounce Timer ---
    #pragma region
    ESP_LOGI(MAIN_TAG, "Initializing GPTimer peripheral for debouncing");
    // gptimer_handle_t* timer_handle = malloc(sizeof(gptimer_handle_t));  // needed to be malloc'd!!
    // if (debounce_timer == NULL) {
    //     ESP_LOGE(MAIN_TAG, "No memory for debounce gptimer handle!");
    //     return; // will cause an abort because tasks aren't supposed to return!
    // }
    gptimer_config_t timer_conf = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .intr_priority = 0,
        .resolution_hz = 10*1000,   //10kHz = 0.1ms ticks
        // .flags.intr_shared
    };
    
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_conf, &debounce_timer));
    // setup the alarm for timer
    gptimer_alarm_config_t timer_alarm_conf = {
        .alarm_count = DEBOUNCE_TIME_MS*10,   // time in ms * 10ticks/ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true   // reloads alarm to reload count immediately after alarm. Will do this in ISR
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(debounce_timer, &timer_alarm_conf));
    // callback function
    gptimer_event_callbacks_t timer_cbs_conf =  {
        .on_alarm = on_debounce_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(debounce_timer, &timer_cbs_conf, (void*)&debounce_timer));
    ESP_ERROR_CHECK(gptimer_enable(debounce_timer));
    ESP_LOGI(MAIN_TAG, "Debounce timer initialized and enabled");
    #pragma endregion
    // --- GPTimers for pulse segments and duration/period
    #pragma region 
    ESP_LOGI(MAIN_TAG, "Creating Segment and Duration timers");
    // adjuct config/params for segment timer
    // gptimer_config_t timer_conf = {0};
    timer_conf.clk_src = GPTIMER_CLK_SRC_APB;   // 80 MHz
    timer_conf.direction = GPTIMER_COUNT_UP;
    timer_conf.intr_priority = 2;
    timer_conf.resolution_hz = 10000000;    // 10 MHz - fast enough for 1MHz sampling

    // gptimer_handle_t segment_timer = NULL;
    // gptimer_handle_t duration_timer = NULL;
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_conf, &segment_timer));
    // Adjust params for duration timer
    timer_conf.intr_priority = 1;   // change resolution? 
    timer_conf.resolution_hz = 10000;   // 10kHz    may want more resolution to know the time? 
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_conf, &duration_timer));
    // alarm config must wait until sample duration and frequency are known!
    // but we can do callbacks now!
    // gptimer_event_callbacks_t timer_cbs_conf = {0};
    timer_cbs_conf.on_alarm = on_segment_alarm;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(segment_timer, &timer_cbs_conf, (void*)NULL));
    timer_cbs_conf.on_alarm = on_duration_alarm;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(duration_timer, &timer_cbs_conf, (void*)NULL));
    #pragma endregion
    // --- SD Card / SDMMC ---
    #pragma region
    // esp_intr_dump(stdout);
    ESP_LOGI(MAIN_TAG, "Waiting to initialize SD card (2s). Ensure clock wire is secured.");
    vTaskDelay(2000/portTICK_PERIOD_MS);

    ESP_LOGI(MAIN_TAG, "Initializing SD card for raw access");
    init_sd_config(&host_g, &slot_config_g, SD_FREQUENCY);
    sdmmc_card_t *card;

    ESP_LOGI(MAIN_TAG, "Mounting SD card - raw access");
    ret = init_sd_card(&card);
    ESP_ERROR_CHECK(ret);
    #pragma endregion
    // before function terminates, update my_handles over to main.
    handles->gptimer = &debounce_timer;    // these are pointers, so the memory is allocated by the called functions.
    handles->card = card;

    // here to try and solve a problem, but it isn't the cause of the problem!
    ESP_LOGI(MAIN_TAG, "Installing GPIO ISRs, then creating tasks");
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));    //enable ISRs to be added.
    // Install GPIO ISRs
    gpio_isr_handler_add(START_BUT, start_isr_handler, (void*)&handles);    // handle's arg not really needed!
    gpio_isr_handler_add(AUX_BUT, aux_isr_handler, (void*)&handles);
    gpio_isr_handler_add(ROTARY_SWITCH, rot_switch_isr_handler, (void*)(handles->gptimer));

    vTaskResume(main_task_handle);
    vTaskDelete(NULL);  // remove this task to free resources
}

void app_main(void) {
    esp_err_t ret;  // var to hold return values
    esp_err_t err;  // other var for the same thing

    main_task_handle = xTaskGetCurrentTaskHandle();     // get task handle for main
    // Set log level to allow display of debug-level logging
    esp_log_level_set("*", ESP_LOG_DEBUG);  // enables debug logs globally (for debugging SD card)
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
    gpio_reset_pin(SD_DETECT);
    gpio_pullup_dis(SD_DETECT);     // disable the default pullup for SD_detect
    gpio_pulldown_en(SD_DETECT);    // active high, means it must be GND for off (pulldown). -- handled by breakout board?
    gpio_set_direction(SD_DETECT, GPIO_MODE_INPUT);
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
    gpio_set_level(SD_LED, 0);
    
    // ----------
    // --- GPTimer / Debounce Timer --- moved
    #pragma region
    /* ESP_LOGI(MAIN_TAG, "Initializing GPTimer peripheral for debouncing");
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
        .on_alarm = on_debounce_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &timer_cbs_conf, (void*)NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer_handle));
    ESP_LOGI(MAIN_TAG, "Debounce timer initialized and enabled"); */
    #pragma endregion
    /* // --- GPTimers for pulse segments and duration/period
    ESP_LOGI(MAIN_TAG, "Creating Segment and Duration timers");
    // adjuct config/params for segment timer
    gptimer_config_t timer_conf = {0};
    timer_conf.clk_src = GPTIMER_CLK_SRC_APB;   // 80 MHz
    timer_conf.direction = GPTIMER_COUNT_UP;
    timer_conf.intr_priority = 2;
    timer_conf.resolution_hz = 10000000;    // 10 MHz - fast enough for 1MHz sampling

    // gptimer_handle_t segment_timer = NULL;
    // gptimer_handle_t duration_timer = NULL;
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_conf, &segment_timer));
    // Adjust params for duration timer
    timer_conf.intr_priority = 1;   // change resolution? 
    timer_conf.resolution_hz = 10000;   // 10kHz
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_conf, &duration_timer));
    // alarm config must wait until sample duration and frequency are known!
    // but we can do callbacks now!
    gptimer_event_callbacks_t timer_cbs_conf = {0};
    timer_cbs_conf.on_alarm = on_segment_alarm;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(segment_timer, &timer_cbs_conf, (void*)NULL));
    timer_cbs_conf.on_alarm = on_duration_alarm;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(duration_timer, &timer_cbs_conf, (void*)NULL));
 */
    // --- PCNT for Sampling ---
    #pragma region 
    ESP_LOGI(MAIN_TAG, "Initializing pulse counters for data sampling");
    pcnt_unit_config_t unit_conf = {
        .high_limit = PULSE_HIGH_COUNT,         // counters are 16bit (64k, or rather +- 32k)
        .low_limit = -1,
        .intr_priority = 0,
        .flags.accum_count = true,   // uses accumulator if counter overflows. May be useful for a long sample duration. 
    };
    // moved to global!
    // pcnt_unit_handle_t counter_handle1 = NULL;      // 2 channels per handle, but interrupts are unit-based
    // pcnt_unit_handle_t counter_handle2 = NULL;
    // pcnt_unit_handle_t counter_handle3 = NULL;
    // pcnt_unit_handle_t counter_handle4 = NULL;
    // need separate unit for each counter handle
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_conf, &counter_handle1));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_conf, &counter_handle2));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_conf, &counter_handle3));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_conf, &counter_handle4));
    pcnt_glitch_filter_config_t counter_filter_conf = {
        .max_glitch_ns = 10,    // use a very short glitch/filter duration
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(counter_handle1, &counter_filter_conf));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(counter_handle2, &counter_filter_conf));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(counter_handle3, &counter_filter_conf));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(counter_handle4, &counter_filter_conf));

    pcnt_chan_config_t chan_conf = {
        .edge_gpio_num = PULSE_PIN_1,
        .level_gpio_num = -1,
        // .flags.virt_level_io_level = 1,
    };
    pcnt_channel_handle_t count_ch1 = NULL;
    pcnt_channel_handle_t count_ch2 = NULL;
    pcnt_channel_handle_t count_ch3 = NULL;
    pcnt_channel_handle_t count_ch4 = NULL;
    
    // make channels for each unit
    ESP_ERROR_CHECK(pcnt_new_channel(counter_handle1, &chan_conf, &count_ch1));
    chan_conf.edge_gpio_num = PULSE_PIN_2;
    ESP_ERROR_CHECK(pcnt_new_channel(counter_handle2, &chan_conf, &count_ch2));
    chan_conf.edge_gpio_num = PULSE_PIN_3;
    ESP_ERROR_CHECK(pcnt_new_channel(counter_handle3, &chan_conf, &count_ch3));
    chan_conf.edge_gpio_num = PULSE_PIN_4;
    ESP_ERROR_CHECK(pcnt_new_channel(counter_handle4, &chan_conf, &count_ch4));

    // set edge actions for the channels
    pcnt_channel_set_edge_action(count_ch1, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_edge_action(count_ch2, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_edge_action(count_ch3, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_edge_action(count_ch4, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);

    // Now set up watchpoints? Might be good for testing, but really we will just get the count directly after a timer expires
    int count_watchpoints[] = { -1, PULSE_HIGH_COUNT/2, PULSE_HIGH_COUNT};
    for (size_t i = 0; i < sizeof(count_watchpoints) / sizeof(count_watchpoints[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(counter_handle1, count_watchpoints[i]));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(counter_handle2, count_watchpoints[i]));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(counter_handle3, count_watchpoints[i]));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(counter_handle4, count_watchpoints[i]));
    }
    // set up callback, ie interrupt when watchpoint hit
    // pcnt_event_callbacks_t counter_cbs = {
    //     .on_reach = on_counter_watch,    // callback was causing error
    // };
    // QueueHandle_t count_queue = xQueueCreate(20, sizeof(count_event_data));
    // ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(counter_handle1, &counter_cbs, (void*)&count_queue));
    // ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(counter_handle2, &counter_cbs, (void*)&count_queue));
    // ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(counter_handle3, &counter_cbs, (void*)&count_queue));
    // ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(counter_handle4, &counter_cbs, (void*)&count_queue));

    ESP_LOGI(MAIN_TAG, "Enabling Counters");
    ESP_ERROR_CHECK(pcnt_unit_enable(counter_handle1));
    ESP_ERROR_CHECK(pcnt_unit_enable(counter_handle2));
    ESP_ERROR_CHECK(pcnt_unit_enable(counter_handle3));
    ESP_ERROR_CHECK(pcnt_unit_enable(counter_handle4));
    ESP_LOGI(MAIN_TAG, "Clearing Counters");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(counter_handle1));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(counter_handle2));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(counter_handle3));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(counter_handle4));
    ESP_LOGI(MAIN_TAG, "Counters configured! They start when a sample begins.");

    #pragma endregion
    // --- Waveplate UART ---
    #pragma region 
    // use functions defined in rotator_driver.h
    // set velocity ("sv") to 50-70% of max. so 0x32 to 0x48 would be sent as ASCII ('3' + '2', etc. )
    // set jog step size to 0 for continuous ("sj")
    init_waveplate_uart();  // initialize the UART for the waveplate

    uart_flush(WAVE_UART_NUM);
    send_waveplate_command( (const char*) "gs", NULL, 0);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    read_waveplate_response();// prints to screen

    send_waveplate_command("ho", 1, 1);
    vTaskDelay(200/portTICK_PERIOD_MS);
    read_waveplate_response();
    // if (!read_waveplate_response()) {
    //     ESP_LOGE(MAIN_TAG, "Error returning waveplate to home position");
    //     // return?
    // }
    send_waveplate_command("sv", 50, 2);    // set speed to 50%
    vTaskDelay(200/portTICK_PERIOD_MS);
    read_waveplate_response();
    // // read response, continue if OK
    // if (!read_waveplate_response()) {
    //     ESP_LOGE(MAIN_TAG, "Error setting velocity for waveplate");
    // }
    send_waveplate_command("sj", 0, 8);     // set jog size to 0 for continuous motion
    vTaskDelay(200/portTICK_PERIOD_MS);
    read_waveplate_response();
    // if (!read_waveplate_response()) {
    //     ESP_LOGE(MAIN_TAG, "Error setting jog step");
    // }

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
    // pcnt_unit_config_t unit_config = {
    //     .high_limit = ROTARY_HIGH,
    //     .low_limit = ROTARY_LOW,
    //     .intr_priority = 1,     // must be equal between all units!
    // };
    unit_conf.high_limit = ROTARY_HIGH;
    unit_conf.low_limit = ROTARY_LOW;

    pcnt_unit_handle_t rotary_counter = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_conf, &rotary_counter));

    ESP_LOGI(MAIN_TAG, "PCNT: setting glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(rotary_counter, &filter_config));

    ESP_LOGI(MAIN_TAG, "Installing pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ROTARY_A,      
        .level_gpio_num = ROTARY_B,   //use -1 if not used
    };  //Channel A: Watches edge of 'A', and level of 'B'.
    pcnt_channel_handle_t pcnt_chan_a = NULL;   //channel for 1 direction
    ESP_ERROR_CHECK(pcnt_new_channel(rotary_counter, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ROTARY_B,
        .level_gpio_num = ROTARY_A,
    };  //Channel B: watches edge of 'B', and level of 'A'. 
    pcnt_channel_handle_t pcnt_chan_b = NULL;   //channel for other direction
    ESP_ERROR_CHECK(pcnt_new_channel(rotary_counter, &chan_b_config, &pcnt_chan_b));
    
    ESP_LOGI(MAIN_TAG, "PCNT: setting edge actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(MAIN_TAG, "PCNT: add watch points and register callbacks");
    int watch_points[] = {ROTARY_LOW, ROTARY_HIGH}; //zero watchpoint adds extra event. 
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(rotary_counter, watch_points[i]));
    }
    pcnt_event_callbacks_t pcnt_cbs = {
        .on_reach = on_pcnt_watch,
    };
    QueueHandle_t pcnt_queue = xQueueCreate(20, sizeof(int));   // size limits how many events can occur in the time that it takes to process an event
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(rotary_counter, &pcnt_cbs, pcnt_queue));

    ESP_LOGI(MAIN_TAG, "Enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(rotary_counter));
    ESP_LOGI(MAIN_TAG, "Clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(rotary_counter));
    ESP_LOGI(MAIN_TAG, "Start pcnt unit");  // well that would do it!
    ESP_ERROR_CHECK(pcnt_unit_start(rotary_counter));
    ESP_LOGI(MAIN_TAG, "PCNT unit started.");
    // structure to send to monitor task
    my_pcnt_t monitor_data = {0};
    monitor_data.event_queue = &pcnt_queue;
    monitor_data.pcnt_handle = &rotary_counter;

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
    mcpwm_oper_handle_t oper2 = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper2));

    ESP_LOGI(MAIN_TAG, "MCPWM: Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer));

    ESP_LOGI(MAIN_TAG, "MCPWM: Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator1 = NULL;
    mcpwm_cmpr_handle_t comparator2 = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config, &comparator2));

    mcpwm_gen_handle_t generator1 = NULL;
    mcpwm_gen_handle_t generator2 = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_1,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator1));
    generator_config.gen_gpio_num = SERVO_2;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &generator_config, &generator2));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(0)));

    ESP_LOGI(MAIN_TAG, "MCPWM: Set generator action on timer and compare event");
    // go high on counter empty - servo 1
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator1,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold - servo 1
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator1,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator1, MCPWM_GEN_ACTION_LOW)));

    // go high on counter empty - servo 2
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold  - servo 2
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator1, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(MAIN_TAG, "MCPWM: Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    #pragma endregion
    // --- SD Card / SDMMC ---
    #pragma region
    /* esp_intr_dump(stdout);
    ESP_LOGI(MAIN_TAG, "Waiting to initialize SD card (2s). Ensure clock wire is secured.");
    vTaskDelay(2000/portTICK_PERIOD_MS);

    ESP_LOGI(MAIN_TAG, "Initializing SD card for raw access");
    init_sd_config(&host_g, &slot_config_g, SD_FREQUENCY);
    sdmmc_card_t *card;

    ESP_LOGI(MAIN_TAG, "Mounting SD card - raw access");
    ret = init_sd_card(&card);
    ESP_ERROR_CHECK(ret); */
    #pragma endregion
    // --- PC UART Monitor ---
    #pragma region
        /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue. - don't use TX buffer
    uart_driver_install(PC_UART_NUM, UART_READ_BUF * 2, 0, 20, &pc_uart_queue, 0);
    uart_param_config(PC_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(UART_MON_TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(PC_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(PC_UART_NUM, UART_PATTERN_CHAR, PATTERN_CHR_NUM, 9, 0, 0);  // use to set variables (by using synchronous prompts?)
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(PC_UART_NUM, 20);
    uart_flush_input(PC_UART_NUM);  // clear buffer to ensure no garbage in there
    #pragma endregion
    // Initialize NVS
    #pragma region 
    ESP_LOGI(MAIN_TAG, "Initializing non-volatile storage (NVS)...");
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    // printf("\n");
    ESP_LOGI(MAIN_TAG, "Opening Non-Volatile Storage (NVS) handle... ");
    // nvs_handle_t my_nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        printf("Done (openning)\n");

        // Read
        printf("Reading restart counter from NVS ... ");
        int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
        get_nvs_int("restart_counter", &restart_counter);
        // Write
        printf("Updating restart counter in NVS ...\n");
        restart_counter++;
        err = nvs_set_i32(my_nvs_handle, "restart_counter", restart_counter);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ...\n");
        err = nvs_commit(my_nvs_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        // nvs_close(my_nvs_handle);
    }
    #pragma endregion
    // Set up FATFS for SPI Flash
    #pragma region
    ESP_LOGI(MAIN_TAG, "Mounting FAT filesystem");
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formatted before
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
            // .use_one_fat = false,
    };
    err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to mount FATFS on flash (%s)", esp_err_to_name(err));
        return;
    }
    // write test file
    char hello_data[EX_FILE_LEN] = { 0};
    snprintf(hello_data, EX_FILE_LEN, "%s %s\n", "hello world, from ESP-IDF", esp_get_idf_version());
    ESP_LOGI(MAIN_TAG, "Writing hello_data to /spiflash/hello.txt; hello_data = %s", hello_data);
    err = write_log_file("/spiflash/hello.txt", hello_data);
    if (err != ESP_OK) {
        ESP_LOGW(MAIN_TAG, "Error writing test file. Reformatting");
        ESP_ERROR_CHECK(esp_vfs_fat_spiflash_format_rw_wl(base_path, "storage"));
        err = write_log_file("/spiflash/hello.txt", hello_data);
        if (err != ESP_OK) {
            ESP_LOGE(MAIN_TAG, "Failed to write second time.");
            return;
        }
    }
    // read file back
    // Open file for reading
    err = simple_file_read("/spiflash/hello.txt");
    if (err != ESP_OK) {
        return;
    }
#pragma endregion
    
    // adc_continuous_handle_t handle = NULL;  //needed in many places, hold it here for distribution.

    my_servos_t servo_handles = {
        .comp1 = &comparator1,
        .comp2 = &comparator2,
    };
    my_handlers_t handles = {
        // .adc_handle = &handle,
        // .gptimer = &timer_handle,
        .servos = &servo_handles,
        // .card = card,   //card is already a pointer
    };
    // run core1_initializer
    TaskHandle_t init_1_handle = NULL;
    xTaskCreatePinnedToCore(core_1_initializer, "CORE_1-INIT", 8*1024, (void*)&handles, 0, &init_1_handle, 1);
    vTaskSuspend(NULL); // core_1 will resume this task
    sdmmc_card_t* card = handles.card;   // get card after it is initialized by core1 initializer

    // ESP_LOGI(MAIN_TAG, "Installing GPIO ISRs, then creating tasks");
    // // Install GPIO ISRs
    // gpio_isr_handler_add(START_BUT, start_isr_handler, (void*)&handles);
    // gpio_isr_handler_add(AUX_BUT, aux_isr_handler, (void*)&handles);
    // gpio_isr_handler_add(ROTARY_SWITCH, rot_switch_isr_handler, (void*)(handles.gptimer));

    // check where ISR ended up
    ESP_LOGI(MAIN_TAG, "Dumping interrupts");
    esp_intr_dump(stdout);

    // forgot to initialize this queue
    copy_queue = xQueueCreate(20, sizeof(pulse_data));

    // register tasks: adc_task, adc_copy (to SD card), adc_transfer (from SD card to PC/python), monitor_vars (logging), servo (control)
    // xTaskCreatePinnedToCore(adc_task, "ADC-TASK", TASK_STACK_SIZE, (void*)&handles, ADC_TASK_PRIORITY, (void*)&adc_task_handle, 1);    
    // pin printing task to core 1, and limit number of tasks on that core (better to not be pinned. Pin other tasks instead). 
    // xTaskCreatePinnedToCore(adc_copy_task, "ADC-COPY", TASK_STACK_SIZE, (void*)&handles, COPY_TASK_PRIORITY, (void*) &adc_copy_handle, tskNO_AFFINITY);
    // xTaskCreatePinnedToCore(adc_transfer_task, "ADC-TRANSFER", TASK_STACK_SIZE*4, (void*)&handles, ADC_TRANSFER_PRIORITY, &adc_transfer_handle, 1);
    // vTaskSuspend(adc_copy_task);
    // Task that updates the lcd periodically. Cannot be preempted by other tasks when setting LCD display. Will scroll if interrupted.
    // xTaskCreate(lcd_task, "LCD-TASK", TASK_STACK_SIZE, lcd_ctx, 2, &lcd_task_handle);   // perhaps scrolling effect is caused by getting kicked off of CPU. increased priority
    // Sample Tasks. Copy on Core 1, transfer on Core 1, and let start be on either
    xTaskCreatePinnedToCore(sample_copy_task, "COPY-TASK" ,TASK_STACK_SIZE, (void*)&handles, COPY_TASK_PRIORITY, &copy_sample_handle, 1);
    xTaskCreatePinnedToCore(sample_start_task, "START-TASK", TASK_STACK_SIZE, (void*)NULL, START_TASK_PRIORITY, &start_task_handle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(sample_transfer_task, "TRANSFER-TASK", TASK_STACK_SIZE, (void*)&handles, SAMPLE_TRANSFER_PRIORITY, &sample_transfer_task_handle, 1);
    // Task for updating variables from PCNT events, and printing log messages.
    xTaskCreatePinnedToCore(monitor_var_task, "MONITOR-TASK", KB_TO_BYTES(2), (void*)&monitor_data, MONITOR_TASK_PRIORITY, &monitor_handle, 0);
    // servo task creation
    xTaskCreatePinnedToCore(servo_task, "SERVO-TASK", KB_TO_BYTES(2), (void*)&servo_handles, SERVO_TASK_PRIORITY, &servo_task_handle, tskNO_AFFINITY);
    // UART listening task needs access to the ADC handle, as well as the servo comparators to be passed to other functions.
    xTaskCreatePinnedToCore(uart_event_task, "UART-LISTEN", KB_TO_BYTES(2) + 512, (void*)&handles, UART_RX_TASK_PRIORITY, &uart_monitor_handle, 0);   // cannot be free to run on core 1.
    
    // set angle of servos to 'off' state
    // servo_angle = SERVO_MIN_DEGREE;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(SERVO1_OFF)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(SERVO2_OFF)));
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
        if (sample_state) { 
            // lcd1602_set_cursor(lcd_ctx, 1, 19);// lcd1602_string(lcd_ctx, BLANK_LINE);    // this causes the text to flash by updating...
            snprintf(strBuf, 22, " %-20s", "    Sampling...");
            lcd1602_set_cursor(lcd_ctx, 1, 19);
            lcd1602_string(lcd_ctx, strBuf);
        } else {
            snprintf(strBuf, 22, " %-20s", "    Standby");
            lcd1602_set_cursor(lcd_ctx, 1, 19);
            lcd1602_string(lcd_ctx, strBuf);
        }
        // check if SD card is inserted or not, if not, display a message, else ... temporarily display one? 
        if (gpio_get_level(SD_DETECT) == 0) {
            snprintf(strBuf, 22, " %-20s", "No SD Card Inserted!");
            gpio_set_level(SD_LED, 0);
            lcd1602_set_cursor(lcd_ctx, 0, 19);
            lcd1602_string(lcd_ctx, strBuf);
        }  else {   //else clear that line!
            gpio_set_level(SD_LED, 1);
            snprintf(strBuf, 22, " %s", BLANK_LINE);
            lcd1602_set_cursor(lcd_ctx, 0, 19);
            lcd1602_string(lcd_ctx, strBuf);
        } 
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
#if USE_RAW_SD
    deinit_sd_card(&card);
    ESP_LOGI(MAIN_TAG, "SD card unmounted - raw access");
    // Unmount FATFS
    ESP_LOGI(MAIN_TAG, "Unmounting FAT filesystem from flash");
    ESP_ERROR_CHECK( esp_vfs_fat_spiflash_unmount_rw_wl(base_path, s_wl_handle));
#else
    esp_vfs_fat_sdcard_unmount(mount_point, card_handle);
    ESP_LOGI(MAIN_TAG, "Card unmounted");
#endif
    free(handles.gptimer);
    lcd1602_deinit(lcd_ctx);
    ESP_LOGI(MAIN_TAG, "LCD deinitialized");
    nvs_close(my_nvs_handle);
}
