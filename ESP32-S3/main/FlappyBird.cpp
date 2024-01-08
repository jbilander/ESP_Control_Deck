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

    // Add birdie
    bird = lv_animimg_create(lv_scr_act());
    reset_game = true;
}

FlappyBird::~FlappyBird()
{
}

void FlappyBird::onUpdate()
{
    if (reset_game)
    {
        has_collided = false;
        reset_game = false;
        run_game = false;
        gravity = {0.f};
        left_mouse_btn_released = true;
        lv_obj_set_pos(bird, LCD_H_RES / 3, LCD_V_RES / 2 - flappy1.header.h / 2);
        lv_animimg_set_src(bird, (const void **)bird_anim_flappy2, 1);
        lv_animimg_set_duration(bird, 300);
        lv_animimg_set_repeat_count(bird, LV_ANIM_REPEAT_INFINITE);
        lv_animimg_start(bird);
    }

    // Game
    if (has_collided)
    {
        // Game is over, wait for right mouse button click to restart
        if (get_mouse_report().buttons.button2)
            reset_game = true;
    }
    else
    {
        if (run_game)
        {
            moveBird();
            // movePipes();
        }
        else
        {
            // Wait for left mouse button click to start game
            if (get_mouse_report().buttons.button1)
            {
                run_game = true;
            }
        }
    }
}

void FlappyBird::moveBird()
{
    if (get_mouse_report().buttons.button1 && left_mouse_btn_released)
    {
        gravity = -8.f;
        left_mouse_btn_released = false;
        lv_animimg_set_src(bird, (const void **)bird_anim_flappy123_rotated, 3);
        lv_animimg_start(bird);
    }
    else
    {
        left_mouse_btn_released = !get_mouse_report().buttons.button1;

        if (lv_obj_get_y(bird) + gravity <= LCD_V_RES - flappy1.header.h)
        {
            lv_obj_set_y(bird, lv_obj_get_y(bird) + gravity);
            if (gravity == 0)
            {
                lv_animimg_set_src(bird, (const void **)bird_anim_flappy123, 3);
            }
        }
        else
        {
            lv_animimg_set_src(bird, (const void **)bird_anim_flappy2, 1);
            lv_obj_set_y(bird, LCD_V_RES - flappy1.header.h / 2);
            has_collided = true;
        }
        gravity += 0.5f;
    }
}