#include "FlappyBird.h"

FlappyBird::FlappyBird()
{
    // Set background stuff
    bg_img = lv_img_create(lv_scr_act());
    lv_img_set_src(bg_img, &background_houses);
    lv_style_set_bg_color(&bg_style, mountain_range_blue);
    lv_obj_add_style(lv_scr_act(), &bg_style, LV_OPA_TRANSP);
    bg_bottom_rect = lv_obj_create(lv_scr_act());
    lv_obj_set_size(bg_bottom_rect, LCD_H_RES, 100);
    lv_style_init(&style_rect);
    lv_style_set_bg_color(&style_rect, lightish_green);
    lv_style_set_radius(&style_rect, 0);
    lv_style_set_border_width(&style_rect, 0);
    lv_obj_add_style(bg_bottom_rect, &style_rect, LV_OPA_TRANSP);
    lv_obj_align(bg_bottom_rect, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_align_to(bg_img, bg_bottom_rect, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

    // Bird animation
    bird = lv_animimg_create(lv_scr_act());
    lv_obj_align(bird, LV_ALIGN_CENTER, -150, 0);
    lv_animimg_set_src(bird, (const void **)bird_anim_imgs, 3);
    lv_animimg_set_duration(bird, 300);
    lv_animimg_set_repeat_count(bird, LV_ANIM_REPEAT_INFINITE);
    lv_animimg_start(bird);

    section_list = {0, 0, 0, 0};
    reset_game = true;
    section_width = (float)LCD_V_RES / (float)(section_list.size() - 1);
}

FlappyBird::~FlappyBird()
{
}

void FlappyBird::onUpdate()
{
}