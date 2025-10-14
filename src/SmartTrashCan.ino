#include <Arduino.h>

#include <vector>
#include <WebSocketsServer.h>


#define ENABLE_DATABASE


#include <FirebaseClient.h>


// 連接板子的接口 插在OTG 不是TTL



#include "WifiConnect.h"            // Wifi
#include "DatabaseClient.h"         // 資料庫
#include "TrashCanLight.h"          // 垃圾桶燈
#include "TrashCanLid.h"            // 垃圾桶蓋
#include "TrashCanCapacity.h"       // 垃圾桶容量
#include "EnvironmentalSensor.h"    // 環境感應器


// 任務
TaskHandle_t taskHandle_databaseClient;
TaskHandle_t taskHandle_trashCanLight;
TaskHandle_t taskHandle_trashCanLid;
TaskHandle_t taskHandle_trashCanCapacity;
TaskHandle_t taskHandle_environmentalSensor;


// 儲存 client_id 的容器
std::vector<uint8_t> clients;

// 搞伺服器在ESP32上
WebSocketsServer webSocket = WebSocketsServer(81);  // Port 81



// WifiConnect& wifiConnect = WifiConnect::getInstance();            // Wifi
// DatabaseClient& databaseClient = DatabaseClient::getInstance();   // 資料庫
// TrashCanLight& trashCanLight = TrashCanLight::getInstance();      // 垃圾桶燈
// TrashCanLid& trashCanLid = TrashCanLid::getInstance();            // 垃圾桶蓋

RealtimeDatabase database;

void onReceivedMessage(String message);
void webSocketEvent(uint8_t client_id, WStype_t event_type, uint8_t *payload, size_t length);
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

	// 垃圾桶燈
	trashCanLight.setup();

	// 垃圾桶蓋
	trashCanLid.setup();

	// 垃圾桶容量
	trashCanCapacity.setup();

	// 環境感應器
	environmentalSensor.setup();


	// 啟動 WebSocket 伺服器
	webSocket.begin();

	// 伺服器事件
	webSocket.onEvent(webSocketEvent);

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
	// 每個循環要處理 WebSocket 的事件
	webSocket.loop();

	wifiConnect.loop();

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


void onReceivedMessage(String message) {
	
	/*
	if (message == "App.TrashCanActivity.switch_led.on") {
		trashCanLight.setEnable(true);
	}

	if (message == "App.TrashCanActivity.switch_led.off") {
		trashCanLight.setEnable(false);
	}

	if (message == "App.TrashCanActivity.switch_cover.on") {
		trashCanLid.setOpen(true);
	}

	if (message == "App.TrashCanActivity.switch_cover.off") {
		trashCanLid.setOpen(false);
	}
	*/
	
}



// 當有 WebSocket 資料進來時呼叫
void webSocketEvent(uint8_t client_id, WStype_t event_type, uint8_t *payload, size_t length) {
	switch (event_type) {
		case WStype_CONNECTED: {
			clients.push_back(client_id);
			Serial.printf("Client [%u] connected", client_id);
			Serial.println();
			IPAddress ip = webSocket.remoteIP(client_id);
			Serial.print("Client IP: ");
			Serial.println(ip);

			webSocket.sendTXT(client_id, "Hello from ESP32 Websocket Server! # 你已經連線 這是一條測試訊息 來自ESP32");

			break;
		}
		case WStype_TEXT: {
			String message = String((char*)payload);
			Serial.printf("Received from client [%u]: %s", client_id, message.c_str());
			Serial.println();

			// Echo message back
			// webSocket.sendTXT(num, "ESP received: " + message);

			onReceivedMessage(message);

			break;
		}
		case WStype_DISCONNECTED: {
			clients.erase(std::remove(clients.begin(), clients.end(), client_id), clients.end());
			Serial.printf("Client [%u] disconnected", client_id);
			Serial.println();
			break;
		}
		default: {
			break;
		}
	}
}
