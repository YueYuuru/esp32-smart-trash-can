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
	Serial.println("初始化 - 垃圾桶燈");

    pinMode(ledPin, OUTPUT);

	available = true;
}


void TrashCanLight::loop() {
    
    if (enable) {
        digitalWrite(ledPin, HIGH);
    } else {
        digitalWrite(ledPin, LOW);
    }
}

bool TrashCanLight::isAvailable() {
	return available;
}

void TrashCanLight::setEnable(bool enable_) {
    enable =  enable_;
}
