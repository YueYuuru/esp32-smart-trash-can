#include <Arduino.h>


#include "TrashCanCapacity.h"


// 目前是放在上面 由上往下發射

// 超聲波垃圾桶高度監測系統
// 腳位定義
const int trigPin = 13;    // 觸發腳位
const int echoPin = 14;    // 回波腳位

// 參數設定
const int maxHeight = 50;    // 垃圾桶最大高度(cm)


TrashCanCapacity::TrashCanCapacity() {
    
}

TrashCanCapacity& TrashCanCapacity::getInstance() {
    static TrashCanCapacity instance;
    return instance;
}

TrashCanCapacity& trashCanCapacity = TrashCanCapacity::getInstance();    // 垃圾桶容量

void TrashCanCapacity::setup() {
	pinMode(trigPin, OUTPUT);      // 設定觸發腳為輸出
	pinMode(echoPin, INPUT);       // 設定回波腳為輸入
}


void TrashCanCapacity::loop() {
	// 發送超聲波脈衝
	digitalWrite(trigPin, LOW);    // 確保觸發腳低電位
	delayMicroseconds(2);          
	digitalWrite(trigPin, HIGH);   // 發送10微秒高電位脈衝
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);

	// 測量回波時間
	long duration = pulseIn(echoPin, HIGH);  // 讀取高電位持續時間(微秒)

	// 計算距離 (聲速340m/s = 0.034公分/微秒)
	float distance = duration * 0.034 / 2;   // 除以2 (來回距離)

	// 計算垃圾高度 = 總高度 - 感測器到垃圾表面的距離
	float trashHeight = maxHeight - distance;

	// 顯示結果
	Serial.printf("感測器距離: %.2f cm | 垃圾高度: %.2f cm\n", distance, trashHeight);

	// 垃圾桶狀態判斷
	if (trashHeight >= maxHeight * 0.8) {  // 超過 80% 容量
		Serial.println("警告：垃圾桶即將滿載！");

		// TODO: 寫入到database 由App監聽狀態&推播

	} else if (distance < 5) {             // 距離過近可能感測異常
		Serial.println("注意：感測器可能被異物遮擋");
	}

	vTaskDelay(pdMS_TO_TICKS(1000));    // 每秒測量一次
}
