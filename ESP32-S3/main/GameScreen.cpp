#include "GameScreen.h"

GameScreen::GameScreen()
{
}

GameScreen::~GameScreen()
{
}

void GameScreen::init()
{
  flappy_bird = new FlappyBird();

  //textarea_fps = lv_textarea_create(lv_scr_act());
  //lv_obj_set_size(textarea_fps, 50, 40);
  //lv_obj_align(textarea_fps, LV_ALIGN_TOP_LEFT, 0, 0);
}

void GameScreen::update()
{
  flappy_bird->onUpdate();
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