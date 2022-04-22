#ifndef VISUAL_MATCH_HPP
#define VISUAL_MATCH_HPP

#include "main.hpp"

#define COLORWIDTH 30


/* Functions for Users */
void webcamInit(vehicleControl_t *robot);

float visualMatch(vehicleControl_t *robot, const string comparePath);

int midPointCapture(vehicleControl_t *robot);

void resizeCamera(int width, int height);

#endif
