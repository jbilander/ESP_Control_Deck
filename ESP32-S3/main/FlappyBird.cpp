#include "FlappyBird.h"

FlappyBird::FlappyBird()
{
}

FlappyBird::~FlappyBird()
{
}

void FlappyBird::init()
{
    section_list = {0, 0, 0, 0};
    reset_game = true;
    section_width = (float)LCD_V_RES / (float)(section_list.size() - 1);
}

void FlappyBird::onUpdate()
{
    ESP_LOGI("FLAP:", "FLAPPING");
}