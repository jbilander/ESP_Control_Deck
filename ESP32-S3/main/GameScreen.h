#ifndef GAMESCREEN_H_FILE
#define GAMESCREEN_H_FILE

#include <stdio.h>
#include <lvgl.h>
#include "Main.h"
#include "FlappyBird.h"

class GameScreen
{
private:
  lv_obj_t *textarea_fps; // textarea to display fps
  int fps_count = 0;
  char buffer[8]; // buffer to contain fps text.
  FlappyBird *flappy_bird;

protected:
public:
  GameScreen();
  ~GameScreen();
  void init();
  void update();
  int getFpsCounter();
  void resetFpsCounter();
  void incrementFpsCounter();
  void updateTextAreaFPS();
};

#endif /* GAMESCREEN_H_FILE */