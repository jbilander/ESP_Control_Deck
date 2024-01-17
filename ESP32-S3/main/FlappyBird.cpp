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

    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_opa(&style, LV_OPA_TRANSP);

    // Fill pipe storage
    for (int i = 0; i < 5; i++)
    {
        lv_obj_t *section = lv_canvas_create(lv_scr_act());
        lv_obj_set_scrollbar_mode(section, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(section, pipe_top.header.w, LCD_V_RES);
        lv_obj_add_style(section, &style, 0);
        move_status.push_back(true);

        lv_obj_t *top_pipe = lv_img_create(section);
        lv_obj_t *bottom_pipe = lv_img_create(section);

        lv_img_set_src(lv_img_create(top_pipe), &pipe_top);
        lv_img_set_src(lv_img_create(bottom_pipe), &pipe_bottom);
        lv_obj_set_y(top_pipe, -175);
        lv_obj_set_y(bottom_pipe, 275);

        pipe_storage.push_back(section);
    }

    // Add birdie
    bird = lv_animimg_create(lv_scr_act());

    lv_obj_set_scrollbar_mode(lv_scr_act(), LV_SCROLLBAR_MODE_OFF);
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
        pipe_velocity = 8;
        left_mouse_btn_released = true;
        lv_obj_set_pos(bird, LCD_H_RES / 3, LCD_V_RES / 2 - flappy1.header.h / 2);
        lv_animimg_set_src(bird, (const void **)bird_anim_flappy2, 1);
        lv_animimg_set_duration(bird, 300);
        lv_animimg_set_repeat_count(bird, LV_ANIM_REPEAT_INFINITE);
        lv_animimg_start(bird);

        for (int i = 0; i < pipe_storage.size(); i++)
        {
            lv_obj_set_x(pipe_storage[i], LCD_H_RES);
            lv_obj_add_flag(pipe_storage[i], LV_OBJ_FLAG_HIDDEN);
        }

        active_pipes.clear();

        // Start with three pipes
        active_pipes.push_back(pipe_storage[0]);
        active_pipes.push_back(pipe_storage[1]);
        active_pipes.push_back(pipe_storage[2]);

        lv_obj_set_x(active_pipes[0], 100);
        lv_obj_set_x(active_pipes[1], 300);
        lv_obj_set_x(active_pipes[2], 500);
        lv_obj_set_y(lv_obj_get_child(active_pipes[0], 0), -180);
        lv_obj_set_y(lv_obj_get_child(active_pipes[0], 1), 340);
        lv_obj_set_y(lv_obj_get_child(active_pipes[1], 0), -175);
        lv_obj_set_y(lv_obj_get_child(active_pipes[1], 1), 275);
        lv_obj_set_y(lv_obj_get_child(active_pipes[2], 0), -250);
        lv_obj_set_y(lv_obj_get_child(active_pipes[2], 1), 320);

        /* Start with first section hidden as it's pos. to the left of birdie already at start. */
        // lv_obj_clear_flag(active_pipes[0], LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(active_pipes[1], LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(active_pipes[2], LV_OBJ_FLAG_HIDDEN);
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
            // addSection();
            moveBird();
            movePipes();
        }
        else
        {
            // Wait for left mouse button click to start game
            if (get_mouse_report().buttons.button1)
            {
                lv_animimg_set_src(bird, (const void **)bird_anim_flappy23_rotated, 2);
                lv_animimg_start(bird);
                run_game = true;
            }
        }
    }
}

void FlappyBird::moveBird()
{
    if (get_mouse_report().buttons.button1 && left_mouse_btn_released)
    {
        gravity = -12.f;
        left_mouse_btn_released = false;
        lv_animimg_set_src(bird, (const void **)bird_anim_flappy23_rotated, 2);
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
            lv_obj_invalidate(bird);
            lv_obj_set_y(bird, LCD_V_RES - flappy1.header.h / 2);
            has_collided = true;
        }
        gravity += 0.5f;
    }
}

void FlappyBird::movePipes()
{

    // used to randomize length of top pipe and the gap to bottom pipe.
    uint32_t top_pipe_y;
    uint32_t section_gap;

    for (int i = 0; i < active_pipes.size(); i++)
    {
        if (lv_obj_get_x(active_pipes[i]) > -60)
        {
            lv_obj_set_x(active_pipes[i], lv_obj_get_x(active_pipes[i]) - pipe_velocity);
            move_status[i] = true;
        }
        else
        {
            if (move_status[i])
            {
                top_pipe_y = lv_rand(100, 290);
                section_gap = lv_rand(80, 130);
                lv_obj_set_y(lv_obj_get_child(active_pipes[i], 0), -top_pipe_y);
                lv_obj_set_y(lv_obj_get_child(active_pipes[i], 1), pipe_top.header.h - top_pipe_y + section_gap);
                lv_obj_set_x(active_pipes[i], LCD_H_RES);
                lv_obj_clear_flag(active_pipes[i], LV_OBJ_FLAG_HIDDEN);
                move_status[i] = false;
            }
        }
    }
}
