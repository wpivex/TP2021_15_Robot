#include "AutonSelector.h"

// Add auton to AutonSelector, which will automatically configure buttons and running the auton
void AutonSelector::addAuton(int (*func)(), const char* name, BTN::Button key) {
  autons.push_back(Auton(func, name, key));
  if (autons.size() == 1) logController(autons[0].name.c_str());
}

// Actually run the auton that was selected by buttons
int AutonSelector::runSelectedAuton() {
  return autons[selectedAuton].function();
}

// Internal method to draw auton buttons
void AutonSelector::drawButtons() {

  Brain.Screen.clearScreen();

  const int NUM_COLS = 3;
  const int NUM_ROWS = 2;
  const float X_MARGIN = 10;
  const float Y_MARGIN = 10;

  float width = (SCREEN_WIDTH - X_MARGIN) / NUM_COLS - X_MARGIN;
  float height = (SCREEN_HEIGHT - Y_MARGIN) / NUM_ROWS - Y_MARGIN;

  int i = -1;
  for (int r = 0; r < NUM_ROWS; r++) {
    for (int c = 0; c < NUM_COLS; c++) {
      if (++i == autons.size()) {r = NUM_ROWS; break;} // no more buttons to draw

      int x = X_MARGIN + ((float) c) / NUM_COLS * (SCREEN_WIDTH - X_MARGIN);
      int y = Y_MARGIN + ((float) r) / NUM_ROWS * (SCREEN_HEIGHT - Y_MARGIN);
      
      // Draw button
      Brain.Screen.setFillColor(i == selectedAuton ? green : red);
      Brain.Screen.drawRectangle(x, y, width, height);

      // Draw button hold
      if (i == heldAuton) {
        int w = width / BUTTON_FRAMES_NEEDED * lengthHeld;
        Brain.Screen.setFillColor(orange);
        Brain.Screen.drawRectangle(x, y, w, height);
      }

      // Draw centered text
      const char* text = autons[i].name.c_str();
      float sw = Brain.Screen.getStringWidth(text);
      float sh = Brain.Screen.getStringHeight(text);
      int tx = x + width/2 - sw/2;
      int ty = y + height/2 - sh/2;
      Brain.Screen.setPenColor(white);
      Brain.Screen.printAt(tx, ty, text);

    }
  }
  Brain.Screen.setPenColor(blue);
  Brain.Screen.printAt(200, 50, "%d", lengthHeld);
  Brain.Screen.render();
}

// To be run every tick; keep track of held buttons and which auto is selected
void AutonSelector::tick() {

  // Keep track of which button is being held
  bool anyPressed = false;
  for (int i = 0; i < autons.size(); i++) {
    if (i == selectedAuton) continue;
    if (buttons.pressing(autons[i].key)) {
      anyPressed = true;
      if (i == heldAuton) {
        lengthHeld++; // continue to hold the button
        if (lengthHeld >= BUTTON_FRAMES_NEEDED) { // pressed for sufficient time, so set as selected auton
          logController(autons[i].name.c_str()); // log to controller as well
          selectedAuton = i;
          heldAuton = -1;
          lengthHeld = 0;
        }
      } else {
        selectedAuton = i;
        lengthHeld = 1;
      }
    }
  }
  if (!anyPressed) {
    heldAuton = -1;
    lengthHeld = 0;
  }

  drawButtons();
  buttons.updateButtonState();

}