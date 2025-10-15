#include <Arduino.h>

#include <vector>
#include <WebSocketsServer.h>


#define ENABLE_DATABASE


#include <FirebaseClient.h>


// 連接板子的接口 插在OTG 不是TTL



#include "WifiConnect.h"              // Wifi
#include "CustomWebSocketServer.h"    // 自訂 WebSocketServer
#include "DatabaseClient.h"           // 資料庫
#include "TrashCanLight.h"            // 垃圾桶燈
#include "TrashCanLid.h"              // 垃圾桶蓋
#include "TrashCanCapacity.h"         // 垃圾桶容量
#include "EnvironmentalSensor.h"      // 環境感應器


// 任務
TaskHandle_t taskHandle_databaseClient;
TaskHandle_t taskHandle_trashCanLight;
TaskHandle_t taskHandle_trashCanLid;
TaskHandle_t taskHandle_trashCanCapacity;
TaskHandle_t taskHandle_environmentalSensor;


// WifiConnect& wifiConnect = WifiConnect::getInstance();            // Wifi
// DatabaseClient& databaseClient = DatabaseClient::getInstance();   // 資料庫
// TrashCanLight& trashCanLight = TrashCanLight::getInstance();      // 垃圾桶燈
// TrashCanLid& trashCanLid = TrashCanLid::getInstance();            // 垃圾桶蓋

RealtimeDatabase database;


void task_databaseClient(void * parameter);
void task_trashCanLight(void * parameter);
void task_trashCanLid(void * parameter);
void task_trashCanCapacity(void * parameter);
void task_environmentalSensor(void * parameter);


void setup() {
	Serial.begin(115200);

	// 連接 Wifi
	wifiConnect.setup();

	// 資料庫
	databaseClient.setup();
	database = databaseClient.getRealtimeDatabase();

	// 自訂 WebSocketServer
	customWebSocketServer.setup();

	// 垃圾桶燈
	trashCanLight.setup();

	// 垃圾桶蓋
	trashCanLid.setup();

	// 垃圾桶容量
	trashCanCapacity.setup();

	// 環境感應器
	environmentalSensor.setup();


	xTaskCreate(
		// 任務函式
		// 任務名稱
		// 堆疊大小(words)
		// 傳入參數
		// 優先權(1~5)
		// 任務句柄
		task_databaseClient, "databaseClient", 16384, NULL, 1, &taskHandle_databaseClient
	);

	xTaskCreate(task_trashCanLight, "trashCanLight", 2048, NULL, 1, &taskHandle_trashCanLight);
	xTaskCreate(task_trashCanLid, "trashCanLid", 4096, NULL, 1, &taskHandle_trashCanLid);
	xTaskCreate(task_trashCanCapacity, "trashCanCapacity", 4096, NULL, 1, &taskHandle_trashCanCapacity);
	xTaskCreate(task_environmentalSensor, "environmentalSensor", 16384, NULL, 1, &taskHandle_environmentalSensor);
}


void loop() {
	wifiConnect.loop();
	customWebSocketServer.loop();


	/*
	String serialMessage = "";
	while (Serial.available() > 0) {
		serialMessage = serialMessage + Serial.readString();
	}
	if (serialMessage != "") {
		Serial.println(serialMessage);

		if (serialMessage == "test.websocket.sendMsg") {
			for (uint8_t id : clients) {
				webSocket.sendTXT(id, "Hello from ESP32 Websocket Server! # 這是一條測試訊息 來自ESP32 !!!!");
			}
		}
	}
	*/

}


void task_databaseClient(void * parameter) {
	// 資料庫

	while(true) {
		databaseClient.loop();
	}
}

void task_trashCanLight(void * parameter) {
	// 垃圾桶燈

	while (true) {
		trashCanLight.loop();
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void task_trashCanLid(void * parameter) {
	// 垃圾桶蓋

	while (true) {
		trashCanLid.loop();
		// 不用延遲(已經寫在loop裡)
	}
}

void task_trashCanCapacity(void * parameter) {
	// 垃圾桶容量

	while (true) {
		trashCanCapacity.loop();
		// 不用延遲(已經寫在loop裡)
	}
}

void task_environmentalSensor(void * parameter) {
	// 環境感應器

	while (true) {
		environmentalSensor.loop();
		// 不用延遲(已經寫在loop裡)
	}
}



