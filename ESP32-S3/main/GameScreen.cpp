#include "GameScreen.h"

GameScreen::GameScreen()
{
}

GameScreen::~GameScreen()
{
}

void GameScreen::init()
{
  LV_IMG_DECLARE(img_rgb565_palette_argb);
  lv_obj_t *img = lv_img_create(lv_scr_act());
  textarea_fps = lv_textarea_create(lv_scr_act());

  lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
  lv_img_set_src(img, &img_rgb565_palette_argb);

  lv_style_init(&bg_style);
  lv_style_set_bg_color(&bg_style, bg_color);
  lv_obj_add_style(lv_scr_act(), &bg_style, LV_OPA_TRANSP);

  lv_obj_set_size(textarea_fps, 50, 40);
  lv_obj_align(textarea_fps, LV_ALIGN_TOP_LEFT, 0, 0);
}

int GameScreen::getFpsCounter()
{
  return fps_count;
}

void GameScreen::resetFpsCounter()
{
  fps_count = 0;
}

void GameScreen::incrementFpsCounter()
{
  fps_count++;
}

void GameScreen::updateTextAreaFPS()
{
  snprintf(buffer, sizeof(buffer), "%d", fps_count);
  lv_textarea_set_text(textarea_fps, buffer);
}

void GameScreen::setBackgroundColor(uint16_t R, uint16_t G, uint16_t B)
{
  bg_color.ch = {.blue = B, .green = G, .red = R};
  lv_style_set_bg_color(&bg_style, bg_color);
  lv_obj_invalidate(lv_scr_act());
}