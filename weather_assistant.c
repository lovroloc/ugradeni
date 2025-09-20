/*
 * Smart Weather Assistant - ESP-IDF with FreeRTOS
 * TFT Display + SD Card version (BME280 removed)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>



#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_sleep.h"
#include "esp_vfs_fat.h"

#include "nvs_flash.h"
#include "cJSON.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

// Configuration
#define WIFI_SSID "Optima-756fe8"
#define WIFI_PASS "OPTIMA2901503445"
#define WEATHER_API_KEY "dddacd858742af40eae1b879b24adc39"
#define WEATHER_CITY "Osijek"
#define WEATHER_COUNTRY "HR"

// Pin definitions
#define TFT_CS_PIN      5
#define TFT_DC_PIN      16
#define TFT_RST_PIN     17
#define TFT_MOSI_PIN    23
#define TFT_CLK_PIN     18
#define TFT_MISO_PIN    19
#define SD_CS_PIN       4
#define BUZZER_PIN      25
#define SERVO_PIN       26

// Display dimensions
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  320

// SPI configuration
#define SPI_HOST_ID     VSPI_HOST
#define SPI_DMA_CHAN    2
#define SPI_MAX_TRANSFER_SIZE (DISPLAY_WIDTH*DISPLAY_HEIGHT*2+8)

// ILI9341 Commands
#define ILI9341_SWRESET     0x01
#define ILI9341_RDDID       0x04
#define ILI9341_RDDST       0x09
#define ILI9341_SLPIN       0x10
#define ILI9341_SLPOUT      0x11
#define ILI9341_PTLON       0x12
#define ILI9341_NORON       0x13
#define ILI9341_RDMODE      0x0A
#define ILI9341_RDMADCTL    0x0B
#define ILI9341_RDPIXFMT    0x0C
#define ILI9341_RDIMGFMT    0x0A
#define ILI9341_RDSELFDIAG  0x0F
#define ILI9341_INVOFF      0x20
#define ILI9341_INVON       0x21
#define ILI9341_GAMMASET    0x26
#define ILI9341_DISPOFF     0x28
#define ILI9341_DISPON      0x29
#define ILI9341_CASET       0x2A
#define ILI9341_PASET       0x2B
#define ILI9341_RAMWR       0x2C
#define ILI9341_RAMRD       0x2E
#define ILI9341_PTLAR       0x30
#define ILI9341_MADCTL      0x36
#define ILI9341_PIXFMT      0x3A

// Colors (RGB565)
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F
#define COLOR_ORANGE    0xFD20
#define COLOR_DARKGRAY  0x7BEF
#define COLOR_LIGHTGRAY 0xC618

// Servo configuration
#define SERVO_MIN_PULSEWIDTH_US 1000
#define SERVO_MAX_PULSEWIDTH_US 2000
#define SERVO_MAX_DEGREE        180
#define SERVO_PULSE_GPIO        13
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD        20000

// Task priorities
#define WEATHER_TASK_PRIORITY       3
#define DISPLAY_TASK_PRIORITY       2
#define AUDIO_TASK_PRIORITY         1

// WiFi event group
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// Logging tags
static const char *TAG = "WeatherAssistant";
static const char *WIFI_TAG = "WiFi";
static const char *WEATHER_TAG = "Weather";
static const char *DISPLAY_TAG = "Display";
static const char *SD_TAG = "SDCard";
static const char *SERVO_TAG = "Servo";
static const char *BUZZER_TAG = "Buzzer";

// Data structures
typedef struct {
    float temperature;
    float humidity;
    char description[64];
    char main[32];
    float wind_speed;
    bool valid;
} weather_data_t;

// Global data
static weather_data_t current_weather = {0};
static char http_response_buffer[2048];
static int http_response_len = 0;

// Hardware handles
static spi_device_handle_t tft_spi_device;
static bool tft_initialized = false;
static bool sd_initialized = false;

// MCPWM handles for servo
static mcpwm_timer_handle_t timer = NULL;
static mcpwm_oper_handle_t oper = NULL;
static mcpwm_cmpr_handle_t comparator = NULL;
static mcpwm_gen_handle_t generator = NULL;

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(WIFI_TAG, "Retry connecting to WiFi");
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// SPI communication functions
static void tft_send_cmd(uint8_t cmd) {
    gpio_set_level(TFT_DC_PIN, 0);  // Command mode
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_transmit(tft_spi_device, &t);
}

static void tft_send_data(uint8_t data) {
    gpio_set_level(TFT_DC_PIN, 1);  // Data mode
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    spi_device_transmit(tft_spi_device, &t);
}

static void tft_send_color(uint16_t color) {
    gpio_set_level(TFT_DC_PIN, 1);  // Data mode
    uint8_t data[2] = {color >> 8, color & 0xFF};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = data,
    };
    spi_device_transmit(tft_spi_device, &t);
}


// Set drawing window
static void tft_set_window(int x0, int y0, int x1, int y1) {
    tft_send_cmd(ILI9341_CASET);
    tft_send_data(x0 >> 8);
    tft_send_data(x0 & 0xFF);
    tft_send_data(x1 >> 8);
    tft_send_data(x1 & 0xFF);
    
    tft_send_cmd(ILI9341_PASET);
    tft_send_data(y0 >> 8);
    tft_send_data(y0 & 0xFF);
    tft_send_data(y1 >> 8);
    tft_send_data(y1 & 0xFF);
    
    tft_send_cmd(ILI9341_RAMWR);
}

static bool init_sd_card(void) {
    ESP_LOGI(SD_TAG, "SD card disabled - using external reader");
    return false; // Always return false to disable
}




// Simple 8x8 font (ASCII 32-127)
static const uint8_t font8x8[96][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
    {0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00}, // '!'
    {0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '"'
    {0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00}, // '#'
    {0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00}, // '$'
    {0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00}, // '%'
    {0x1C, 0x36, 0x1C, 0x6E, 0x3B, 0x33, 0x6E, 0x00}, // '&'
    {0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}, // '''
    {0x18, 0x0C, 0x06, 0x06, 0x06, 0x0C, 0x18, 0x00}, // '('
    {0x06, 0x0C, 0x18, 0x18, 0x18, 0x0C, 0x06, 0x00}, // ')'
    {0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00}, // '*'
    {0x00, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x00, 0x00}, // '+'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x06, 0x00}, // ','
    {0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00}, // '-'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00}, // '.'
    {0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00}, // '/'
    {0x3E, 0x63, 0x73, 0x7B, 0x6F, 0x67, 0x3E, 0x00}, // '0'
    {0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x00}, // '1'
    {0x1E, 0x33, 0x30, 0x1C, 0x06, 0x33, 0x3F, 0x00}, // '2'
    {0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, 0x00}, // '3'
    {0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x78, 0x00}, // '4'
    {0x3F, 0x03, 0x1F, 0x30, 0x30, 0x33, 0x1E, 0x00}, // '5'
    {0x1C, 0x06, 0x03, 0x1F, 0x33, 0x33, 0x1E, 0x00}, // '6'
    {0x3F, 0x33, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x00}, // '7'
    {0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, 0x00}, // '8'
    {0x1E, 0x33, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00}, // '9'
    // Add more characters as needed...
    // For brevity, showing just a few essential ones
};

// Draw character
static void tft_draw_char(int x, int y, char c, uint16_t color, uint16_t bg_color) {
    if (c < 32 || c > 127) c = 32; // Default to space
    
    const uint8_t *char_data = font8x8[c - 32];
    
    for (int row = 0; row < 8; row++) {
        uint8_t line = char_data[row];
        for (int col = 0; col < 8; col++) {
            uint16_t pixel_color = (line & (0x80 >> col)) ? color : bg_color;
            tft_set_window(x + col, y + row, x + col, y + row);
            tft_send_color(pixel_color);
        }
    }
}

// Draw string
static void tft_draw_string(int x, int y, const char* text, uint16_t color, uint16_t bg_color) {
    int start_x = x;
    int char_count = 0;
    
    while (*text) {
        if (*text == '\n') {
            x = start_x;
            y += 10;
        } else {
            tft_draw_char(x, y, *text, color, bg_color);
            x += 8;
        }
        text++;
        char_count++;
        
        // Yield every 5 characters
        if (char_count % 5 == 0) {
            taskYIELD();
        }
    }
}



// Load and display BMP from SD card
static bool display_bmp(const char* filename, int x, int y) {
    if (!sd_initialized) {
        ESP_LOGE(SD_TAG, "SD card not initialized");
        return false;
    }
    
    char path[64];
    snprintf(path, sizeof(path), "/sdcard/%s", filename);
    
    FILE* file = fopen(path, "rb");
    if (!file) {
        ESP_LOGE(SD_TAG, "Failed to open %s", path);
        return false;
    }
    
    // Read BMP header (simplified - assumes 24-bit BMP)
    uint8_t header[54];
    if (fread(header, 1, 54, file) != 54) {
        ESP_LOGE(SD_TAG, "Invalid BMP file");
        fclose(file);
        return false;
    }
    
    // Check BMP signature
    if (header[0] != 'B' || header[1] != 'M') {
        ESP_LOGE(SD_TAG, "Not a BMP file");
        fclose(file);
        return false;
    }
    
    int width = *(int*)&header[18];
    int height = *(int*)&header[22];
    int data_offset = *(int*)&header[10];
    
    ESP_LOGI(SD_TAG, "BMP: %dx%d pixels", width, height);
    
    // Seek to pixel data
    fseek(file, data_offset, SEEK_SET);
    
    // Read and display pixels (simplified for 24-bit BMP)
    uint8_t pixel[3];
    for (int row = height - 1; row >= 0; row--) { // BMP is stored bottom-up
        for (int col = 0; col < width; col++) {
            if (fread(pixel, 1, 3, file) != 3) break;
            
            // Convert BGR to RGB565
            uint16_t color = ((pixel[2] >> 3) << 11) | ((pixel[1] >> 2) << 5) | (pixel[0] >> 3);
            
            tft_set_window(x + col, y + row, x + col, y + row);
            tft_send_color(color);
        }
    }
    
    fclose(file);
    ESP_LOGI(SD_TAG, "Displayed %s successfully", filename);
    return true;
}

static void tft_fill_screen(uint16_t color) {
    tft_set_window(0, 0, DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1);
    
    // Send pixels in very small batches
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < DISPLAY_WIDTH; x++) {
            tft_send_color(color);
            
            // Yield every 100 pixels
            if ((y * DISPLAY_WIDTH + x) % 100 == 0) {
                taskYIELD();
            }
        }
        
        // Yield every 10 rows and add small delay
        if (y % 10 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

static void tft_fill_rect(int x, int y, int w, int h, uint16_t color) {
    if (x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT) return;
    if (x + w > DISPLAY_WIDTH) w = DISPLAY_WIDTH - x;
    if (y + h > DISPLAY_HEIGHT) h = DISPLAY_HEIGHT - y;
    
    tft_set_window(x, y, x + w - 1, y + h - 1);
    
    int total_pixels = w * h;
    int chunk_size = 50; // Even smaller for rectangles
    
    for (int i = 0; i < total_pixels; i += chunk_size) {
        int pixels_to_send = (i + chunk_size > total_pixels) ? (total_pixels - i) : chunk_size;
        
        for (int j = 0; j < pixels_to_send; j++) {
            tft_send_color(color);
        }
        
        // Yield after every chunk
        taskYIELD();
    }
}


// Initialize TFT display
static bool init_tft_display(void) {
    ESP_LOGI(DISPLAY_TAG, "Initializing ILI9341 TFT display...");
    
    // Configure GPIO pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TFT_DC_PIN) | (1ULL << TFT_RST_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(DISPLAY_TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = TFT_MISO_PIN,
        .mosi_io_num = TFT_MOSI_PIN,
        .sclk_io_num = TFT_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096, // Reduce from large value
    };
    
    ret = spi_bus_initialize(SPI_HOST_ID, &buscfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(DISPLAY_TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configure TFT device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10*1000*1000,    // Reduce to 10 MHz
        .mode = 0,
        .spics_io_num = TFT_CS_PIN,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    
    ret = spi_bus_add_device(SPI_HOST_ID, &devcfg, &tft_spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(DISPLAY_TAG, "Failed to add TFT device: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(DISPLAY_TAG, "SPI bus and device configured");
    
    // Hardware reset with longer delays
    gpio_set_level(TFT_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(TFT_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(DISPLAY_TAG, "Starting display initialization sequence...");
    
    //obrisi ovo
    gpio_set_level(TFT_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(TFT_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(DISPLAY_TAG, "Testing reset pin...");
    for(int i = 0; i < 5; i++) {
    gpio_set_level(TFT_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(TFT_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
}
    //
    

    // Initialize display with longer delays
    tft_send_cmd(ILI9341_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    tft_send_cmd(ILI9341_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    tft_send_cmd(ILI9341_PIXFMT);
    tft_send_data(0x55); // 16-bit color
    vTaskDelay(pdMS_TO_TICKS(50));
    
    tft_send_cmd(ILI9341_MADCTL);
    tft_send_data(0x48); // Rotation and mirroring
    vTaskDelay(pdMS_TO_TICKS(50));
    
    tft_send_cmd(ILI9341_DISPON);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(DISPLAY_TAG, "Display initialization complete");
    
    // Test with simple pattern
    ESP_LOGI(DISPLAY_TAG, "Testing display with simple pattern...");
    tft_fill_screen(COLOR_RED);
    vTaskDelay(pdMS_TO_TICKS(500));
    tft_fill_screen(COLOR_GREEN);
    vTaskDelay(pdMS_TO_TICKS(500));
    tft_fill_screen(COLOR_BLUE);
    vTaskDelay(pdMS_TO_TICKS(500));
    tft_fill_screen(COLOR_BLACK);
    
    ESP_LOGI(DISPLAY_TAG, "Display test complete - should see color flashes");
    
    return true;
}

// Initialize servo
static void init_servo(void) {
    ESP_LOGI(SERVO_TAG, "Initializing servo...");

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 1500)); // Center position

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(SERVO_TAG, "Servo initialized");
}

static void set_servo_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH_US + (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) / 180;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_width));
    
    ESP_LOGD(SERVO_TAG, "Servo set to %d degrees", angle);
}

static void update_servo_position(void) {
    if (!current_weather.valid) return;
    
    int angle = 90; // Default center
    
    if (strstr(current_weather.main, "Clear") != NULL) {
        angle = 30;  // Sun position
    } else if (strstr(current_weather.main, "Rain") != NULL) {
        angle = 150; // Rain position
    } else if (strstr(current_weather.main, "Cloud") != NULL) {
        angle = 90;  // Cloud position
    }
    
    set_servo_angle(angle);
}

// Initialize buzzer
static void init_buzzer(void) {
    ESP_LOGI(BUZZER_TAG, "Initializing buzzer...");
    
    ledc_timer_config_t timer_config = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 4000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_config);
    
    ledc_channel_config_t channel_config = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0,
    };
    ledc_channel_config(&channel_config);
    
    ESP_LOGI(BUZZER_TAG, "Buzzer initialized");
}

static void play_tone(int frequency, int duration_ms) {
    if (frequency > 0) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, frequency);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4095);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(50));
}
// Initialize WiFi
static void init_wifi(void) {
    wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "WiFi init finished");
}

// HTTP event handler
static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(WEATHER_TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(WEATHER_TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(WEATHER_TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(WEATHER_TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            if (http_response_len + evt->data_len < sizeof(http_response_buffer)) {
                memcpy(http_response_buffer + http_response_len, evt->data, evt->data_len);
                http_response_len += evt->data_len;
                http_response_buffer[http_response_len] = '\0';
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(WEATHER_TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(WEATHER_TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(WEATHER_TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}

// Parse weather JSON response
static void parse_weather_data(const char* json_string) {
    cJSON *json = cJSON_Parse(json_string);
    if (json == NULL) {
        ESP_LOGE(WEATHER_TAG, "Failed to parse JSON");
        return;
    }

    cJSON *main = cJSON_GetObjectItem(json, "main");
    if (main != NULL) {
        cJSON *temp = cJSON_GetObjectItem(main, "temp");
        cJSON *humidity = cJSON_GetObjectItem(main, "humidity");
        
        if (temp != NULL && cJSON_IsNumber(temp)) {
            current_weather.temperature = temp->valuedouble;
        }
        if (humidity != NULL && cJSON_IsNumber(humidity)) {
            current_weather.humidity = humidity->valuedouble;
        }
    }

    cJSON *weather_array = cJSON_GetObjectItem(json, "weather");
    if (weather_array != NULL && cJSON_IsArray(weather_array)) {
        cJSON *weather_item = cJSON_GetArrayItem(weather_array, 0);
        if (weather_item != NULL) {
            cJSON *description = cJSON_GetObjectItem(weather_item, "description");
            cJSON *main_weather = cJSON_GetObjectItem(weather_item, "main");
            
            if (description != NULL && cJSON_IsString(description)) {
                strncpy(current_weather.description, description->valuestring, sizeof(current_weather.description) - 1);
            }
            if (main_weather != NULL && cJSON_IsString(main_weather)) {
                strncpy(current_weather.main, main_weather->valuestring, sizeof(current_weather.main) - 1);
            }
        }
    }

    cJSON *wind = cJSON_GetObjectItem(json, "wind");
    if (wind != NULL) {
        cJSON *speed = cJSON_GetObjectItem(wind, "speed");
        if (speed != NULL && cJSON_IsNumber(speed)) {
            current_weather.wind_speed = speed->valuedouble;
        }
    }

    current_weather.valid = true;
    
    ESP_LOGI(WEATHER_TAG, "Weather updated: %.1f°C, %s, Wind: %.1f m/s", 
             current_weather.temperature, current_weather.description, current_weather.wind_speed);

    cJSON_Delete(json);
}

// Play weather melody
static void play_weather_melody(void) {
    if (!current_weather.valid) return;
    
    ESP_LOGI(BUZZER_TAG, "Playing weather melody...");
    
    if (strstr(current_weather.main, "Clear") != NULL || current_weather.temperature > 20) {
        // Happy melody for sunny weather
        int happy_melody[] = {262, 294, 330, 349, 392, 440, 494, 523}; // C major scale
        for (int i = 0; i < 8; i++) {
            play_tone(happy_melody[i], 200);
        }
    } else if (strstr(current_weather.main, "Rain") != NULL) {
        // Calm melody for rainy weather
        int rain_melody[] = {523, 494, 440, 392, 349, 330, 294, 262}; // Descending scale
        for (int i = 0; i < 8; i++) {
            play_tone(rain_melody[i], 300);
        }
    } else {
        // Neutral melody for other weather
        int neutral_melody[] = {330, 392, 330, 392, 440, 392, 330};
        for (int i = 0; i < 7; i++) {
            play_tone(neutral_melody[i], 250);
        }
    }
}

// Get weather icon filename based on weather condition
static const char* get_weather_icon_filename(void) {
    if (!current_weather.valid) return "unknown.bmp";
    
    if (strstr(current_weather.main, "Clear") != NULL) {
        return "sun.bmp";
    } else if (strstr(current_weather.main, "Rain") != NULL || 
               strstr(current_weather.main, "Drizzle") != NULL) {
        return "rain.bmp";
    } else if (strstr(current_weather.main, "Cloud") != NULL) {
        return "cloud.bmp";
    } else if (strstr(current_weather.main, "Snow") != NULL) {
        return "snow.bmp";
    } else if (strstr(current_weather.main, "Thunderstorm") != NULL) {
        return "storm.bmp";
    } else {
        return "default.bmp";
    }
}

// Enhanced display function with BMP sprites
static void display_weather_info(void) {
    if (!tft_initialized) return;
    
    ESP_LOGI(DISPLAY_TAG, "Updating display with weather info and sprites...");
    
    // Clear screen with blue background
    tft_fill_screen(COLOR_BLUE);
    
    // Draw title
    tft_fill_rect(0, 0, DISPLAY_WIDTH, 30, COLOR_DARKGRAY);
    tft_draw_string(10, 8, "WEATHER ASSISTANT", COLOR_WHITE, COLOR_DARKGRAY);
    
    // Display weather icon sprite
    if (sd_initialized && current_weather.valid) {
        const char* icon_file = get_weather_icon_filename();
        ESP_LOGI(DISPLAY_TAG, "Loading weather icon: %s", icon_file);
        
        // Display weather icon in upper right
        if (!display_bmp(icon_file, DISPLAY_WIDTH - 80, 40)) {
            // Fallback - draw colored rectangle if BMP fails
            uint16_t icon_color = COLOR_YELLOW;
            if (strstr(current_weather.main, "Rain") != NULL) icon_color = COLOR_CYAN;
            else if (strstr(current_weather.main, "Cloud") != NULL) icon_color = COLOR_LIGHTGRAY;
            
            tft_fill_rect(DISPLAY_WIDTH - 70, 40, 60, 60, icon_color);
        }
    }
    
    // Weather information section
     if (current_weather.valid) {
        char temp_str[32];
        snprintf(temp_str, sizeof(temp_str), "Temp: %.1f C", current_weather.temperature);
        tft_draw_string(10, 50, temp_str, COLOR_WHITE, COLOR_BLUE);
        
        char desc_str[80];  // Increased size to accommodate "Weather: " + description
        snprintf(desc_str, sizeof(desc_str), "Weather: %.50s", current_weather.description);  // Limit description to 50 chars
        tft_draw_string(10, 70, desc_str, COLOR_WHITE, COLOR_BLUE);
        
        char humid_str[32];
        snprintf(humid_str, sizeof(humid_str), "Humidity: %.0f%%", current_weather.humidity);
        tft_draw_string(10, 90, humid_str, COLOR_WHITE, COLOR_BLUE);
        
        char wind_str[32];
        snprintf(wind_str, sizeof(wind_str), "Wind: %.1f m/s", current_weather.wind_speed);
        tft_draw_string(10, 110, wind_str, COLOR_WHITE, COLOR_BLUE);
    } else {
        tft_draw_string(10, 50, "Weather data", COLOR_RED, COLOR_BLUE);
        tft_draw_string(10, 70, "not available", COLOR_RED, COLOR_BLUE);
    }
    
    
    // Draw separator
    tft_fill_rect(0, 140, DISPLAY_WIDTH, 2, COLOR_WHITE);
    
    // Recommendations section
    tft_draw_string(10, 155, "CLOTHING ADVICE:", COLOR_YELLOW, COLOR_BLUE);
    
    if (current_weather.valid) {
        const char* advice = "";
        uint16_t advice_color = COLOR_WHITE;
        
        if (current_weather.temperature > 25) {
            advice = "Light clothes, t-shirt";
            advice_color = COLOR_ORANGE;
        } else if (current_weather.temperature > 15) {
            advice = "Light jacket/sweater";
            advice_color = COLOR_GREEN;
        } else if (current_weather.temperature > 5) {
            advice = "Warm jacket needed";
            advice_color = COLOR_YELLOW;
        } else {
            advice = "Heavy coat, gloves!";
            advice_color = COLOR_RED;
        }
        
        tft_draw_string(10, 175, advice, advice_color, COLOR_BLUE);
        
        if (strstr(current_weather.main, "Rain") != NULL) {
            tft_draw_string(10, 195, "Don't forget umbrella!", COLOR_CYAN, COLOR_BLUE);
        }
        
        if (current_weather.wind_speed > 5.0) {
            tft_draw_string(10, 215, "Windy - secure hat!", COLOR_MAGENTA, COLOR_BLUE);
        }
    }
    
    // Display additional decorative sprites if available
    if (sd_initialized) {
        // Try to load and display decorative elements
        display_bmp("logo.bmp", 10, DISPLAY_HEIGHT - 60);  // Bottom left logo
        display_bmp("border.bmp", 0, DISPLAY_HEIGHT - 20); // Bottom border
    }
    
    // Status bar at bottom
    char status_str[64];
    snprintf(status_str, sizeof(status_str), "SD:%s WiFi:%s", 
             sd_initialized ? "OK" : "NO", 
             (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) ? "OK" : "NO");
    
    tft_fill_rect(0, DISPLAY_HEIGHT - 15, DISPLAY_WIDTH, 15, COLOR_BLACK);
    tft_draw_string(5, DISPLAY_HEIGHT - 12, status_str, COLOR_GREEN, COLOR_BLACK);
}

// Save weather data to SD card log
static void log_weather_to_sd(void) {
    if (!sd_initialized || !current_weather.valid) return;
    
    FILE* log_file = fopen("/sdcard/weather_log.txt", "a");
    if (log_file) {
        time_t now;
        time(&now);
        
        fprintf(log_file, "%lld,%.1f,%.0f,%s,%.1f\n", 
                (long long)now, current_weather.temperature, current_weather.humidity,
                current_weather.main, current_weather.wind_speed);
        fclose(log_file);
        
        ESP_LOGI(SD_TAG, "Weather data logged to SD card");
    } else {
        ESP_LOGE(SD_TAG, "Failed to open log file");
    }
}

// Weather task
static void weather_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    int fetch_counter = 0;
    
    while (1) {
        fetch_counter++;
        ESP_LOGI(WEATHER_TAG, "Weather fetch attempt #%d", fetch_counter);
        
        // Wait for WiFi connection
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                                               pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));
        
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(WEATHER_TAG, "WiFi connected, fetching weather data...");
            
            // Reset response buffer
            http_response_len = 0;
            memset(http_response_buffer, 0, sizeof(http_response_buffer));
            
            // Create HTTP client
            char url[256];
            snprintf(url, sizeof(url), 
                    "http://api.openweathermap.org/data/2.5/weather?q=%s,%s&appid=%s&units=metric",
                    WEATHER_CITY, WEATHER_COUNTRY, WEATHER_API_KEY);
            
            ESP_LOGI(WEATHER_TAG, "Requesting: %s", url);
            
            esp_http_client_config_t config = {
                .url = url,
                .event_handler = http_event_handler,
                .timeout_ms = 15000, // Increase timeout
            };
            
            esp_http_client_handle_t client = esp_http_client_init(&config);
            esp_err_t err = esp_http_client_perform(client);
            
            if (err == ESP_OK) {
                int status_code = esp_http_client_get_status_code(client);
                ESP_LOGI(WEATHER_TAG, "HTTP Status: %d, Content length: %d", 
                        status_code, esp_http_client_get_content_length(client));
                
                if (status_code == 200 && http_response_len > 0) {
                    ESP_LOGI(WEATHER_TAG, "Parsing weather data...");
                    parse_weather_data(http_response_buffer);
                    update_servo_position();
                    ESP_LOGI(WEATHER_TAG, "Weather data updated successfully");
                } else {
                    ESP_LOGE(WEATHER_TAG, "HTTP request failed with status: %d", status_code);
                }
            } else {
                ESP_LOGE(WEATHER_TAG, "HTTP perform failed: %s", esp_err_to_name(err));
            }
            
            esp_http_client_cleanup(client);
        } else {
            ESP_LOGW(WEATHER_TAG, "WiFi not connected, skipping weather update");
        }
        
        // Wait 5 minutes for testing (normally 10 minutes)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(300000));
    }
}


// Display task
static void display_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    int update_counter = 0;
    
    while (1) {
        update_counter++;
        ESP_LOGI(DISPLAY_TAG, "Display update #%d", update_counter);
        
        if (tft_initialized) {
            // Simple text display without screen clearing
            // Draw weather info at fixed positions
            
            // Draw title
            char title[32];
            snprintf(title, sizeof(title), "Weather Update %d", update_counter);
            tft_draw_string(10, 10, title, COLOR_GREEN, COLOR_BLACK);
            
            // Draw weather data if available
            if (current_weather.valid) {
                char temp_str[32];
                snprintf(temp_str, sizeof(temp_str), "Temp: %.1f C", current_weather.temperature);
                tft_draw_string(10, 30, temp_str, COLOR_YELLOW, COLOR_BLACK);
                
                char desc_str[64];
                snprintf(desc_str, sizeof(desc_str), "%.50s", current_weather.description);
                tft_draw_string(10, 50, desc_str, COLOR_WHITE, COLOR_BLACK);
                
                char humid_str[32];
                snprintf(humid_str, sizeof(humid_str), "Humidity: %.0f%%", current_weather.humidity);
                tft_draw_string(10, 70, humid_str, COLOR_CYAN, COLOR_BLACK);
                
                char wind_str[32];
                snprintf(wind_str, sizeof(wind_str), "Wind: %.1f m/s", current_weather.wind_speed);
                tft_draw_string(10, 90, wind_str, COLOR_MAGENTA, COLOR_BLACK);
            } else {
                tft_draw_string(10, 30, "No weather data", COLOR_RED, COLOR_BLACK);
                tft_draw_string(10, 50, "Check WiFi/API", COLOR_RED, COLOR_BLACK);
            }
            
            // Draw status
            char wifi_status[32];
            bool wifi_connected = (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
            snprintf(wifi_status, sizeof(wifi_status), "WiFi: %s", wifi_connected ? "OK" : "NO");
            tft_draw_string(10, 280, wifi_status, wifi_connected ? COLOR_GREEN : COLOR_RED, COLOR_BLACK);
            
            ESP_LOGI(DISPLAY_TAG, "Display updated successfully");
        } else {
            ESP_LOGW(DISPLAY_TAG, "Display not initialized");
        }
        
        // Update every 15 seconds for testing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(15000));
    }
}
//test za servo
static void test_servo_motor(void) {
    ESP_LOGI(SERVO_TAG, "Testing servo motor...");
    
    // Test servo sweep
    for (int angle = 0; angle <= 180; angle += 30) {
        ESP_LOGI(SERVO_TAG, "Setting servo to %d degrees", angle);
        set_servo_angle(angle);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second at each position
    }
    
    // Return to center
    set_servo_angle(90);
    ESP_LOGI(SERVO_TAG, "Servo test complete");
}

// Morning alarm task
static void morning_alarm_task(void *pvParameters) {
    while (1) {
        // Check every hour if it's time for morning alarm
        // For demo purposes, play alarm every 30 minutes
        ESP_LOGI(BUZZER_TAG, "Morning alarm check...");
        
        if (current_weather.valid) {
            ESP_LOGI(BUZZER_TAG, "Playing morning weather alert!");
            play_weather_melody();
        }
        
        // Wait 30 minutes for demo (in real use, check time and play at specific hour)
        vTaskDelay(pdMS_TO_TICKS(1800000)); // 30 minutes
    }
}

// Main application
void app_main(void) {
    ESP_LOGI(TAG, "Starting Smart Weather Assistant...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize hardware
    ESP_LOGI(TAG, "Initializing hardware...");
    
    tft_initialized = init_tft_display();
    if (tft_initialized) {
        ESP_LOGI(TAG, "✓ TFT display ready");
    } else {
        ESP_LOGE(TAG, "✗ TFT display failed");
    }
    
    // Disable SD card completely
    sd_initialized = false;
    ESP_LOGI(TAG, "SD card disabled - using external reader");
    
    // Initialize servo
    init_servo();
    ESP_LOGI(TAG, "✓ Servo initialized");
    
    // Test servo immediately
    test_servo_motor();
    
    // Initialize buzzer
    init_buzzer();
    ESP_LOGI(TAG, "✓ Buzzer ready");
    
    // Initialize WiFi
    init_wifi();
    ESP_LOGI(TAG, "✓ WiFi starting...");

    // Create tasks
    ESP_LOGI(TAG, "Creating tasks...");
    
    xTaskCreate(weather_task, "weather_task", 4096, NULL, WEATHER_TASK_PRIORITY, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, DISPLAY_TASK_PRIORITY, NULL);
    
    ESP_LOGI(TAG, "Weather Assistant running!");
}