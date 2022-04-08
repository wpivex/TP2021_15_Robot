#include "VisualGraph.h"

VisualGraph::VisualGraph(float minY, float maxY, int numMarkersY, int numX): data(numX) {
  this->minY = minY;
  this->maxY = maxY;
  this->numMarkersY = numMarkersY;
  this->numX = numX;
}

void VisualGraph::push(float dataPoint) {
  data.push(dataPoint);
}

static int Y_AXIS_XPOS = 50;
static int RIGHT_X = 460;
static int TOP_Y = 20;
static int BOTTOM_Y = 200;
static int X_AXIS_TEXT_OFFSET = 20;
static color grey = color(50,50,50);

int VisualGraph::valueToY(float value) {
  float scalar = (value - minY) / (maxY - minY);
  return BOTTOM_Y - (BOTTOM_Y - TOP_Y) * scalar;
}

void VisualGraph::display() {

  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(transparent);
  


  // Draw y axis labels
  for (int i = 0; i <= numMarkersY + 1; i++) {

    float label = maxY - (maxY - minY) * ((float) i) / (numMarkersY + 1);
    int ypos = TOP_Y + (BOTTOM_Y - TOP_Y) * ((float) i) / (numMarkersY + 1);

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(0, ypos, "%.1f", label);
    Brain.Screen.setPenColor(grey);
    Brain.Screen.drawLine(Y_AXIS_XPOS, ypos, RIGHT_X, ypos);
  }

  // Draw x axis labels
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(Y_AXIS_XPOS, BOTTOM_Y + X_AXIS_TEXT_OFFSET, "0");
  Brain.Screen.printAt(RIGHT_X - 10, BOTTOM_Y + X_AXIS_TEXT_OFFSET, "%d", numX);

  Brain.Screen.setPenColor(white);
  Brain.Screen.drawLine(Y_AXIS_XPOS, BOTTOM_Y, Y_AXIS_XPOS, TOP_Y); // Draw y axis
  Brain.Screen.drawLine(Y_AXIS_XPOS, BOTTOM_Y, RIGHT_X, BOTTOM_Y); // Draw x axis

  if (data.getSize() < 2) return;

  // Draw f(x)
  Brain.Screen.setPenColor(green);
  int y1 = valueToY(data.get(0));
  for (int i = 1; i < data.getSize(); i++) {
    int x1 = Y_AXIS_XPOS + (RIGHT_X - Y_AXIS_XPOS) * (i-1) / (numX - 1);
    int x2 = Y_AXIS_XPOS + (RIGHT_X - Y_AXIS_XPOS) * (i) / (numX - 1);
    int y2 = valueToY(data.get(i));

    Brain.Screen.drawLine(x1, y1, x2, y2);
    y1 = y2;
  }

}