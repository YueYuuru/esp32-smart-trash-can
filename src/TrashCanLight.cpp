#include <Arduino.h>


#include "TrashCanLight.h"


// 引腳定義
const int ledPin = 4;


TrashCanLight::TrashCanLight() {
    enable = false;
}

TrashCanLight& TrashCanLight::getInstance() {
    static TrashCanLight instance;
    return instance;
}

TrashCanLight& trashCanLight = TrashCanLight::getInstance();    // 垃圾桶燈

void TrashCanLight::setup() {
    pinMode(ledPin, OUTPUT);
}


void TrashCanLight::loop() {
    
    if (enable) {
        digitalWrite(ledPin, HIGH);
    } else {
        digitalWrite(ledPin, LOW);
    }
}

void TrashCanLight::setEnable(bool enable_) {
    enable =  enable_;
}
