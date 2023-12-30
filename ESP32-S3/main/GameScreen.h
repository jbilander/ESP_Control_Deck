#ifndef GAMESCREEN_H_FILE
#define GAMESCREEN_H_FILE

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <lvgl.h>

class GameScreen
{
private:
  lv_obj_t *textarea_fps; // textarea to display fps
  lv_style_t bg_style;
  lv_color_t bg_color = {.ch = {.blue = 0, .green = 0, .red = 0}};
  volatile int fps_count = 0;
  char buffer[8]; // buffer to contain fps text.

protected:
public:
  GameScreen();
  ~GameScreen();
  void init();
  int getFpsCounter();
  void resetFpsCounter();
  void incrementFpsCounter();
  void updateTextAreaFPS();
  void setBackgroundColor(uint16_t Red, uint16_t Green, uint16_t Blue);
};

#endif /* GAMESCREEN_H_FILE */