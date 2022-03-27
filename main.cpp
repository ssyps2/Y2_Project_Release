#include "main.hpp"
#include "visualScan.hpp"
#include "lineTrack.hpp"
#include "devices.hpp"
#include <thread>

vehicleControl_t robot;

//std::thread webcamReadThread;

/* Start of Main */
int main() {
    webcamInit(&robot);
    chassisInit(&robot);

    //webcamReadThread(readWebcam);

    while(true){
        midPointCapture(&robot);
        trackLine(&robot);
        //visualMatch(&robot);
    }
}
