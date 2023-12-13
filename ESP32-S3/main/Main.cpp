#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb_hid_host.h"

#include "driver/i2s_std.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

// I2S AUDIO OUT PINMAPPING
#define I2S_WS_PIN GPIO_NUM_39
#define I2S_BCLK_PIN GPIO_NUM_38
#define I2S_DATA_OUT_PIN GPIO_NUM_45

#define WAV_FILE "/sdcard/test.wav" // wav file to play
#define AUDIO_BUFFER 2048           // buffer size for reading the wav file and sending to i2s

// SDCARD 1-BIT SDIO PINMAPPING
#define SDMMC_CLK_PIN GPIO_NUM_40
#define SDMMC_CMD_PIN GPIO_NUM_42
#define SDMMC_D0_PIN GPIO_NUM_41
#define SDMMC_D1_PIN GPIO_NUM_NC
#define SDMMC_D2_PIN GPIO_NUM_NC
#define SDMMC_D3_PIN GPIO_NUM_NC

// RGB565 LCD PINMAPPING
#define LCD_PIXEL_CLOCK_HZ (25 * 1000 * 1000)
#define PIN_NUM_HSYNC 1   // New Line
#define PIN_NUM_VSYNC 2   // New Frame
#define PIN_NUM_DE 48     // Data Enable On/Off
#define PIN_NUM_PCLK 47   // Pixel clock
#define PIN_NUM_DATA0 15  // B0
#define PIN_NUM_DATA1 16  // B1
#define PIN_NUM_DATA2 17  // B2
#define PIN_NUM_DATA3 18  // B3
#define PIN_NUM_DATA4 21  // B4
#define PIN_NUM_DATA5 9   // G0
#define PIN_NUM_DATA6 10  // G1
#define PIN_NUM_DATA7 11  // G2
#define PIN_NUM_DATA8 12  // G3
#define PIN_NUM_DATA9 13  // G4
#define PIN_NUM_DATA10 14 // G5
#define PIN_NUM_DATA11 4  // R0
#define PIN_NUM_DATA12 5  // R1
#define PIN_NUM_DATA13 6  // R2
#define PIN_NUM_DATA14 7  // R3
#define PIN_NUM_DATA15 8  // R4

// The pixel number in horizontal and vertical
#define LCD_H_RES 640
#define LCD_V_RES 480

// FrameBuffer
#define LCD_NUM_FB 2 // allocate double frame buffer, Maximum number of buffers are 3

static const char *LCD_TAG = "RGB_LCD";
static const char *USB_TAG = "USB_HID";
static const char *I2S_TAG = "I2S";
static const char *SDIO_TAG = "SDCARD";

static esp_lcd_panel_handle_t panel_handle;
static uint8_t s_led_state = 0;

// USB Host stuff
QueueHandle_t app_event_queue;
app_event_queue_t evt_queue;
extern "C" QueueHandle_t create_queue(int queue_length, unsigned int item_size);
extern "C" void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                         const hid_host_driver_event_t event,
                                         void *arg);
extern "C" void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                      const hid_host_driver_event_t event,
                                      void *arg);

// static lv_disp_t *disp;
// static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
// static lv_disp_drv_t disp_drv;      // contains callback functions

// I2S stuff
i2s_chan_handle_t tx_handle;

// SDCard stuff, SDMMC_HOST_FLAG_SPI
sdmmc_card_t *card;

void print_sdcard_info(void)
{
    ESP_LOGI(SDIO_TAG, "SD card info:");
    ESP_LOGI(SDIO_TAG, "Name: %s", card->cid.name);
    ESP_LOGI(SDIO_TAG, "Speed: %s", (card->csd.tr_speed < 25000000) ? "Default Speed" : "High Speed");
    ESP_LOGI(SDIO_TAG, "Frequency: %ukHz", card->max_freq_khz);
    ESP_LOGI(SDIO_TAG, "Log Bus Width: %u", card->log_bus_width);
    ESP_LOGI(SDIO_TAG, "Read Block Length: %u", card->csd.read_block_len);
}

/*
static void configure_led(void)
{
    gpio_reset_pin(GPIO_NUM_38);
    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT);
}
*/

static void init_rgb_lcd(void)
{
    ESP_LOGI(LCD_TAG, "Install RGB LCD panel driver");
    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {// The timing parameters should refer to your LCD spec
                    .pclk_hz = LCD_PIXEL_CLOCK_HZ,
                    .h_res = LCD_H_RES,
                    .v_res = LCD_V_RES,
                    .hsync_pulse_width = 96,
                    .hsync_back_porch = 48,
                    .hsync_front_porch = 16,
                    .vsync_pulse_width = 2,
                    .vsync_back_porch = 33,
                    .vsync_front_porch = 10},
        .data_width = 16, // RGB565 in parallel mode, thus 16-bit in width
        .num_fbs = LCD_NUM_FB,
        .psram_trans_align = 64,
        .hsync_gpio_num = PIN_NUM_HSYNC,
        .vsync_gpio_num = PIN_NUM_VSYNC,
        .de_gpio_num = PIN_NUM_DE,
        .pclk_gpio_num = PIN_NUM_PCLK,
        .disp_gpio_num = -1, // display control signal not used
        .data_gpio_nums = {PIN_NUM_DATA0, PIN_NUM_DATA1, PIN_NUM_DATA2, PIN_NUM_DATA3, PIN_NUM_DATA4, PIN_NUM_DATA5, PIN_NUM_DATA6, PIN_NUM_DATA7, PIN_NUM_DATA8, PIN_NUM_DATA9, PIN_NUM_DATA10, PIN_NUM_DATA11, PIN_NUM_DATA12, PIN_NUM_DATA13, PIN_NUM_DATA14, PIN_NUM_DATA15},
        .flags = {.fb_in_psram = true}};
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    /*
        ESP_LOGI(LCD_TAG, "Register event callbacks");
        esp_lcd_rgb_panel_event_callbacks_t cbs = {
            .on_vsync = on_vsync_event};
        ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));
    */
    ESP_LOGI(LCD_TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
}

static void usb_lib_task(void *pvParameter)
{
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskNotifyGive((TaskHandle_t)pvParameter);

    while (1)
    {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        // usb_host_lib_handle_events(100, &event_flags);
        //  In this example, there is only one client registered
        //  So, once we deregister the client, this call must succeed with ESP_OK
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            ESP_LOGI(USB_TAG, "NO CLIENTS");
            ESP_ERROR_CHECK(usb_host_device_free_all());
            break;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            ESP_LOGI(USB_TAG, "ALL FREE");
        }
    }

    ESP_LOGI(USB_TAG, "USB shutdown");
    // Clean up USB Host
    vTaskDelay(10); // Short delay to allow clients clean-up
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

static void init_usb_hid(void *pvParameter)
{
    ESP_LOGI(USB_TAG, "Configure USB HID");
    /*
     * HID host driver configuration
     * - create background task for handling low level event inside the HID driver
     * - provide the device callback to get new HID Device connection event
     */

    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = tskNO_AFFINITY,
        .callback = hid_host_device_callback,
        .callback_arg = NULL};

    ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

    // Create queue to receive USB HID Events
    app_event_queue = create_queue(10, sizeof(app_event_queue_t));
    ESP_LOGI(USB_TAG, "Waiting for HID Device to be connected");
    xTaskNotifyGive((TaskHandle_t)pvParameter);

    while (1)
    {
        if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY))
        {
            if (APP_EVENT_HID_HOST == evt_queue.event_group)
            {
                ESP_LOGI(USB_TAG, "APP_EVENT_HID_HOST");
                hid_host_device_event(evt_queue.hid_host_device.handle,
                                      evt_queue.hid_host_device.event,
                                      evt_queue.hid_host_device.arg);
            }
        }
    }
    ESP_LOGI(USB_TAG, "HID Driver uninstall");
    ESP_ERROR_CHECK(hid_host_uninstall());
    xQueueReset(app_event_queue);
    vQueueDelete(app_event_queue);
}

esp_err_t i2s_setup(void)
{
    ESP_LOGI(I2S_TAG, "Setup I2S");
    // setup a standard config and the channel
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    // setup the i2s config
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),                                                  // the wav file sample rate
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO), // the wav faile bit and channel config
        .gpio_cfg = {
            // refer to configuration.h for pin setup
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws = I2S_WS_PIN,
            .dout = I2S_DATA_OUT_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    return i2s_channel_init_std_mode(tx_handle, &std_cfg);
}

esp_err_t init_sdcard(void)
{
    ESP_LOGI(SDIO_TAG, "Initializing SD card");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SDMMC_CLK_PIN;
    slot_config.cmd = SDMMC_CMD_PIN;
    slot_config.d0 = SDMMC_D0_PIN;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(SDIO_TAG, "Failed to mount filesystem. "
                               "If you want the card to be formatted, set format_if_mount_failed = true.");
        }
        else
        {
            ESP_LOGE(SDIO_TAG, "Failed to initialize the card (%s). "
                               "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t play_wav(char *fp)
{
    FILE *fh = fopen(fp, "rb");
    if (fh == NULL)
    {
        ESP_LOGE(SDIO_TAG, "Failed to open file");
        return ESP_ERR_INVALID_ARG;
    }

    // skip the header...
    fseek(fh, 44, SEEK_SET);

    // create a writer buffer
    void *buf = calloc(AUDIO_BUFFER, sizeof(int16_t));
    size_t bytes_read = 0;
    size_t bytes_written = 0;

    bytes_read = fread(buf, sizeof(int16_t), AUDIO_BUFFER, fh);

    i2s_channel_enable(tx_handle);

    while (bytes_read > 0)
    {
        // write the buffer to the i2s
        i2s_channel_write(tx_handle, buf, bytes_read * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        bytes_read = fread(buf, sizeof(int16_t), AUDIO_BUFFER, fh);
        ESP_LOGV(SDIO_TAG, "Bytes read: %d", bytes_read);
    }

    i2s_channel_disable(tx_handle);
    free(buf);

    return ESP_OK;
}

extern "C" void app_main()
{
    ESP_LOGI(USB_TAG, "Create usb_lib_task to initialize USB Host library");
    BaseType_t usb_lib_task_created = xTaskCreate(usb_lib_task, "usb_events", 4096, xTaskGetCurrentTaskHandle(), 2, NULL);
    assert(usb_lib_task_created == pdTRUE);
    ulTaskNotifyTake(false, 1000); // Wait for notification from usb_lib_task to proceed

    BaseType_t usb_hid_task_created = xTaskCreate(init_usb_hid, "usb_hid_task", 4096, xTaskGetCurrentTaskHandle(), 2, NULL);
    assert(usb_hid_task_created == pdTRUE);
    ulTaskNotifyTake(false, 1000); // Wait for notification from init_usb_hid to proceed

    //configure_led();

    ESP_ERROR_CHECK(init_sdcard());
    print_sdcard_info();
    i2s_setup();

    // play the wav file
    ESP_ERROR_CHECK(play_wav(WAV_FILE));
    init_rgb_lcd();
    i2s_del_channel(tx_handle); // delete the channel

    while (1)
    {
        /*
        gpio_set_level(GPIO_NUM_38, s_led_state);
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        */

        /* Gameloop
            processInput(); //handles any user input that has happened since the last call.
            update(); // advances the game simulation one step. Run AI and physics.
            render(); // draws the game so the player can see what happened.
        */
    }
}