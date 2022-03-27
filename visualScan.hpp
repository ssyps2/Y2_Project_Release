#ifndef VISUAL_MATCH_HPP
#define VISUAL_MATCH_HPP

#include "main.hpp"


/* Functions for Users */
void webcamInit(vehicleControl_t *robot);

float visualMatch(vehicleControl_t *robot);

int midPointCapture(vehicleControl_t *robot);

void readWebcam();

#endif
