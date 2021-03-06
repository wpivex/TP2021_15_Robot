#pragma once
#include "vex.h"

const static int NUM_BUTTONS = 12;

class Buttons {

  public:
    enum Button {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3, X = 4, Y = 5, A = 6, B = 7, L1 = 8, L2 = 9, R1 = 10, R2 = 11, NONE = 12, INVALID = 13};
    enum Axis {RIGHT_HORIZONTAL = 0 , RIGHT_VERTICAL = 1, LEFT_VERTICAL = 2, LEFT_HORIZONTAL = 3};


    Buttons(vex::controller* c);
    void updateButtonState();

    bool pressing(Button b);
    bool pressing(int index);
    bool pressed(Button b);
    bool released(Button b);
    Button get();
    float axis(Axis a);

  private:
    vex::controller* rController;
    const vex::controller::button* getObject(Button b);
    const vex::controller::axis *AXES[5];
    bool prevButtonState[NUM_BUTTONS] = {false}; // initalize all values to false

};