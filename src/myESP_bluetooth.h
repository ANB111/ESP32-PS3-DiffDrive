#include <map>

#include <Ps3Controller.h>
#include "freertos/task.h"

typedef struct ControllerData {
    int8_t lx;
    int8_t ly;
    int8_t rx;
    int8_t ry;
} ControllerData;

ControllerData controllerData;

void getControlData() {

    if(Ps3.event.analog_changed.stick.lx) {
        controllerData.lx = Ps3.data.analog.stick.lx;
    }
    if(Ps3.event.analog_changed.stick.ly) {
        controllerData.ly = Ps3.data.analog.stick.ly;
    }
    if(Ps3.event.analog_changed.stick.rx) {
        controllerData.rx = Ps3.data.analog.stick.rx;
    }
    if(Ps3.event.analog_changed.stick.ry) {
        controllerData.ry = Ps3.data.analog.stick.ry;
    }
}
void controllerConfig(const char* controllerMAC) {
    Ps3.attach(getControlData);
    Serial.println("The device started, now you can pair it with bluetooth!");
    Ps3.begin(controllerMAC);
    while(!Ps3.isConnected()) {
        delay(1000);
        if (Ps3.isConnected()) {
            Serial.println("Connected");
        } else {
            Serial.println("Not connected");
        }
    }
}
ControllerData getControllerData() {
    return controllerData;
}
