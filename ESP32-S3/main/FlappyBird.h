#ifndef FLAPPYBIRD_H_FILE
#define FLAPPYBIRD_H_FILE

#include <esp_log.h>
#include <esp_timer.h>
#include <ctime>
#include <vector>
#include <lvgl.h>
#include <usb/hid_usage_mouse.h>
#include "Main.h"

extern "C" hid_mouse_input_report_boot_t get_mouse_report();

// Background components
LV_IMG_DECLARE(background_houses);
static lv_img_dsc_t background_img = background_houses;
static lv_color_t mountain_range_blue = {.ch = {.blue = 26, .green = 48, .red = 9}};
static lv_color_t lightish_green = {.ch = {.blue = 14, .green = 57, .red = 11}};
static lv_style_t bg_style;
static lv_obj_t *bg_img;
static lv_obj_t *bg_bottom_rect;
static lv_style_t style_rect;

// Pipe
LV_IMG_DECLARE(pipe_top)
LV_IMG_DECLARE(pipe_bottom)

// Birdie
LV_IMG_DECLARE(flappy1)
LV_IMG_DECLARE(flappy2)
LV_IMG_DECLARE(flappy3)
LV_IMG_DECLARE(flappy1_rotated)
LV_IMG_DECLARE(flappy2_rotated)
LV_IMG_DECLARE(flappy3_rotated)

static const void *bird_anim_flappy123[3] = {
    &flappy1,
    &flappy2,
    &flappy3,
};
static const void *bird_anim_flappy23_rotated[2] = {
    &flappy1_rotated,
    &flappy2_rotated,
};
static const void *bird_anim_flappy2[1] = {
    &flappy2};

class FlappyBird
{
private:
    lv_obj_t *bird;
    std::vector<lv_obj_t *> pipes;
    bool left_mouse_btn_released;
    float gravity;
    bool has_collided;
    bool reset_game;
    bool run_game;

protected:
    void moveBird();
    void movePipes();

public:
    FlappyBird();
    ~FlappyBird();
    void onUpdate();
};

#endif /* FLAPPYBIRD_H_FILE */