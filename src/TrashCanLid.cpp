#include <Arduino.h>
#include <ESP32Servo.h>


#include "TrashCanLid.h"


// Debug
const int debug = false;


// 引腳定義
const int trigPin = 11;    // 超音波Trig引腳
const int echoPin = 12;    // 超音波Echo引腳
const int servoPin = 7;    // 伺服馬達控制引腳

// 參數設定
const int openAngle = 150;			                // 開蓋角度
const int closeAngle = 0;			                // 關蓋角度
const int detectionDist = 15;		                // 檢測距離threshold(cm)
const int maxDetectionDist = detectionDist + 10;    // 檢測距離上限(cm)
const unsigned long delayTime = 3000;               // 開蓋保持時間(ms)

Servo lidServo;					     // 伺服馬達對象
bool isOpen = false;			     // 蓋子狀態
unsigned long lastDetectTime = 0;    // 最後檢測時間
bool objectCleared = false;          // 追踪物體是否離開超音波檢測範圍
bool lock = false;                   // 提供給手動開關蓋使用

TrashCanLid::TrashCanLid() {

}

TrashCanLid& TrashCanLid::getInstance() {
    static TrashCanLid instance;
    return instance;
}

TrashCanLid& trashCanLid = TrashCanLid::getInstance();    // 垃圾桶蓋

// 測量距離函數
float measureDistance() {
	if (debug) {
		return 0;
	}

	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);

	return (pulseIn(echoPin, HIGH) * 0.034 / 2); // 聲音速度換算(cm/μs)
}


// 開蓋函數
void openLid() {
	lidServo.attach(servoPin);
	lidServo.write(openAngle);
	vTaskDelay(pdMS_TO_TICKS(500)); // 等待轉好到位
	lidServo.detach();
	isOpen = true;
	Serial.println("垃圾桶蓋打開");
}

// 關蓋函數
void closeLid() {
	lidServo.attach(servoPin);
	lidServo.write(closeAngle);
	vTaskDelay(pdMS_TO_TICKS(500)); // 等待轉好到位
	lidServo.detach();
	isOpen = false;
	Serial.println("垃圾桶蓋關閉");
}


void TrashCanLid::setup() {
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);

	lidServo.attach(servoPin);
	lidServo.write(closeAngle); // 初始關閉蓋子
	vTaskDelay(pdMS_TO_TICKS(500));
	lidServo.detach(); // 避免持續供電導致過熱
	enable = false;
	objectCleared = false;
	lock = false;
}

void TrashCanLid::loop() {
	float distance = measureDistance();

	/*
	String serialMessage = "";
	while (Serial.available() > 0) {
		serialMessage = serialMessage + Serial.read();
	}
	if (serialMessage != "") {
		Serial.println(serialMessage);

		if (serialMessage == "test.trashCanLid.distance.30") {
			distance = 30;
		}

		if (serialMessage == "test.trashCanLid.distance.10") {
			distance = 10;
		}
	}
	*/

	if (!lock) {

		// 偵測到物體靠近且蓋子關閉
		if (distance < detectionDist && !isOpen && objectCleared) {
			openLid();
			objectCleared = false;
			lastDetectTime = millis(); // 紀錄檢測時間
		}

		// 蓋子已開啟且超過保持時間
		if (isOpen && (millis() - lastDetectTime > delayTime)) {
			closeLid();
		}

		// 如果物體遠離允許下次開蓋
		if (distance > detectionDist + 5) {
			objectCleared = true;
		}
	}

	vTaskDelay(pdMS_TO_TICKS(100)); // 迴圈延遲
}

void TrashCanLid::setOpen(bool enable_) {

	if (enable == enable_) {
		return;
	}

	enable = enable_;

	if (enable) {
		Serial.println("手動開蓋");
		lock = true;
		openLid();
	} else {
		Serial.println("手動關蓋");
		closeLid();
		lock = false;
	}
}





