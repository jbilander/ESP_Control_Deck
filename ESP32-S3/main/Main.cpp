
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <esp_timer.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <usb/usb_host.h>
#include <usb/hid_host.h>
#include <driver/i2s_std.h>
#include <driver/sdmmc_host.h>
#include <sdmmc_cmd.h>
#include <esp_vfs_fat.h>
#include <lvgl.h>

#include "usb_hid_host.h"
#include "Main.h"
#include "GameScreen.h"

static const char *LCD_TAG = "RGB_LCD";
static const char *USB_TAG = "USB_HID";
static const char *I2S_TAG = "I2S";
static const char *SDIO_TAG = "SDCARD";

// RGB Display stuff, we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
static esp_lcd_panel_handle_t panel_handle;
static lv_disp_t *disp;             // pointer to a display structure, lv_disp_drv_t should be the first member of the structure
static lv_disp_draw_buf_t disp_buf; // holds display buffer information, draw buffers
static lv_disp_drv_t disp_drv;      // display driver structure, contains callback function
static SemaphoreHandle_t lvgl_mux;
static SemaphoreHandle_t sem_gui_ready;
static SemaphoreHandle_t sem_vsync_end;
static SemaphoreHandle_t sem_gameloop_sync;

GameScreen gamescreen;

// USB Host stuff
app_event_queue_t evt_queue;
extern "C"
{
    QueueHandle_t create_queue(int queue_length, unsigned int item_size);
    void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg);
    void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                               const hid_host_driver_event_t event,
                               void *arg);
}

// I2S stuff
i2s_chan_handle_t tx_handle;

// SDCard stuff
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

static bool on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE)
    {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
    xSemaphoreGive(sem_gameloop_sync);

    return high_task_awoken == pdTRUE;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);

    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static bool lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(LCD_TAG, "Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (lvgl_lock(-1))
        {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

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
        .data_width = LCD_DATA_WIDTH, // RGB565 in parallel mode, thus 16-bit in width
        .num_fbs = LCD_NUM_FB,
        .psram_trans_align = 64,
        .hsync_gpio_num = PIN_NUM_HSYNC,
        .vsync_gpio_num = PIN_NUM_VSYNC,
        .de_gpio_num = PIN_NUM_DE,
        .pclk_gpio_num = PIN_NUM_PCLK,
        .disp_gpio_num = -1, // display control signal not used
        .data_gpio_nums = {PIN_NUM_DATA0, PIN_NUM_DATA1, PIN_NUM_DATA2, PIN_NUM_DATA3, PIN_NUM_DATA4, PIN_NUM_DATA5, PIN_NUM_DATA6, PIN_NUM_DATA7, PIN_NUM_DATA8, PIN_NUM_DATA9, PIN_NUM_DATA10, PIN_NUM_DATA11, PIN_NUM_DATA12, PIN_NUM_DATA13, PIN_NUM_DATA14, PIN_NUM_DATA15},
        .flags = {.fb_in_psram = true, .double_fb = true}};
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(LCD_TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = on_vsync_event};
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

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
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),                                                    // the wav file sample rate
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

static void init_lvgl_lib()
{
    ESP_LOGI(LCD_TAG, "Initialize LVGL library");
    lv_init();

    void *buf1, *buf2;
    ESP_LOGI(LCD_TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    //buf1 = heap_caps_malloc(LCD_H_RES * LCD_V_RES * (LCD_DATA_WIDTH / 8), MALLOC_CAP_SPIRAM);
    //buf2 = heap_caps_malloc(LCD_H_RES * LCD_V_RES * (LCD_DATA_WIDTH / 8), MALLOC_CAP_SPIRAM);

    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LCD_V_RES);

    ESP_LOGI(LCD_TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers

    disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(LCD_TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);

    ESP_LOGI(LCD_TAG, "Create LVGL task");
    xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
}

static void periodic_timer_callback(void *arg)
{
    //gamescreen.updateTextAreaFPS();
    //gamescreen.resetFpsCounter();
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

    ESP_ERROR_CHECK(init_sdcard());
    print_sdcard_info();
    i2s_setup();

    // play the wav file
    // ESP_ERROR_CHECK(play_wav(WAV_FILE));

    ESP_LOGI(LCD_TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
    sem_gameloop_sync = xSemaphoreCreateBinary();
    assert(sem_gameloop_sync);

    init_rgb_lcd();
    init_lvgl_lib();
    gamescreen.init();

    // a periodic timer to calculate fps.
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "periodic"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000));

    i2s_del_channel(tx_handle); // delete the channel
    int speed_cap = 0;

    while (1)
    {
        gamescreen.update();
        gamescreen.incrementFpsCounter();
        if (speed_cap == 4)
        {
            xSemaphoreTake(sem_gameloop_sync, portMAX_DELAY);
            speed_cap = 0;
        }
        speed_cap++;
    }
}