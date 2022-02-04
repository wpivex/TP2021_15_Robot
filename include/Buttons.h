#pragma once
#include "vex.h"

const static int NUM_BUTTONS = 12;

class Buttons {

  public:
    enum Button {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3, X = 4, Y = 5, A = 6, B = 7, L1 = 8, L2 = 9, R1 = 10, R2 = 11, NONE = 12};


    Buttons(vex::controller* c);
    void updateButtonState();

    bool pressing(Button b);
    bool pressing(int index);
    bool pressed(Button b);
    bool released(Button b);
    Button get();

  private:
    vex::controller* rController;
    vex::controller::button* getObject(Button b);
    bool prevButtonState[NUM_BUTTONS] = {false}; // initalize all values to false

};