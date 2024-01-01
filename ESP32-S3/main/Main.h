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
#define LCD_DATA_WIDTH 16 // RGB565

// FrameBuffer
#define LCD_NUM_FB 2 // allocate double frame buffer, Maximum number of buffers are 3

// LVGL stuff
#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE (4 * 1024)
#define LVGL_TASK_PRIORITY 2