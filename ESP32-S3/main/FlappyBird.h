#ifndef FLAPPYBIRD_H_FILE
#define FLAPPYBIRD_H_FILE

#include <esp_log.h>
#include <list>
#include "Main.h"

class FlappyBird
{
private:
    // float fBirdPosition = 0.0f;
    // float fBirdVelocity = 0.0f;
    // float fBirdAcceleration = 0.0f;
    // float fGravity = 100.0f;
    // float fLevelPosition = 0.0f;

    float section_width;
    std::list<int> section_list;

    // bool bHasCollided = false;
    bool reset_game = false;

    // int nAttemptCount = 0;
    // int nFlapCount = 0;
    // int nMaxFlapCount = 0;

protected:
public:
    FlappyBird();
    ~FlappyBird();
    void init();
    void onUpdate();
};

#endif /* FLAPPYBIRD_H_FILE */