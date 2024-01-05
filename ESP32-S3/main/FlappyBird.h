#ifndef FLAPPYBIRD_H_FILE
#define FLAPPYBIRD_H_FILE

#include <esp_log.h>
#include <esp_timer.h>
#include <list>
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

// Birdie
LV_IMG_DECLARE(flappy1)
LV_IMG_DECLARE(flappy2)
LV_IMG_DECLARE(flappy3)

static const void *bird_anim_imgs[3] = {
    &flappy1,
    &flappy2,
    &flappy3,
};

class FlappyBird
{
private:
    lv_obj_t *bird;
    lv_obj_t *img_flappy1;
    lv_obj_t *img_flappy2;
    lv_obj_t *img_flappy3;

    float section_width;
    std::list<int> section_list;

    bool has_collided = false;
    bool reset_game = false;

protected:
public:
    FlappyBird();
    ~FlappyBird();
    void onUpdate();
};

#endif /* FLAPPYBIRD_H_FILE */