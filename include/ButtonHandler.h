#pragma once
#include "vex.h"

const int NUM_BUTTONS = 12;

class ButtonHandler {

public:

enum Button {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3, X = 4, Y = 5, A = 6, B = 7, L1 = 8, L2 = 9, R1 = 10, R2 = 11, NONE = 12};


ButtonHandler(vex::controller* c);
void updateButtonState();

bool currentlyPressing(Button b);
bool currentlyPressing(int index);
bool startPressed(Button b);
bool startReleased(Button b);
Button getButtonPressed();

private:
vex::controller* rController;
vex::controller::button* get(Button b);
bool prevButtonState[NUM_BUTTONS] = {false}; // initalize all values to false


};