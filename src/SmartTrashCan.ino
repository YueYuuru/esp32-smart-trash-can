#include <Arduino.h>
#include <string>
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


void taskInit();

void task_databaseClient(void * parameter);
void task_trashCanLight(void * parameter);
void task_trashCanLid(void * parameter);
void task_trashCanCapacity(void * parameter);
void task_environmentalSensor(void * parameter);

const std::vector<std::string> split(const std::string& str, const std::string& pattern);


void setup() {
	delay(5000);  // 等個5秒 等你開好序列監視器 :D
	Serial.begin(115200);
	Serial.println("初始化");
	Serial.println();

	// 連接 Wifi
	wifiConnect.setup();

	// 自訂 WebSocketServer
	customWebSocketServer.setup();

	delay(1000);

	// 建立任務
	// xTaskCreate(<任務函式>, <任務名稱>, <堆疊大小(words)>, <傳入參數>, <優先權(1~5)>, <任務句柄>);
	xTaskCreate(task_databaseClient, "databaseClient", 16384, NULL, 1, &taskHandle_databaseClient);
	xTaskCreate(task_trashCanLight, "trashCanLight", 2048, NULL, 1, &taskHandle_trashCanLight);
	xTaskCreate(task_trashCanLid, "trashCanLid", 4096, NULL, 1, &taskHandle_trashCanLid);
	xTaskCreate(task_trashCanCapacity, "trashCanCapacity", 4096, NULL, 1, &taskHandle_trashCanCapacity);
	xTaskCreate(task_environmentalSensor, "environmentalSensor", 16384, NULL, 1, &taskHandle_environmentalSensor);

	// 任務初始化
	taskInit();

	// 開始任務
	vTaskResume(taskHandle_databaseClient);
	vTaskResume(taskHandle_trashCanLight);
	vTaskResume(taskHandle_trashCanLid);
	vTaskResume(taskHandle_trashCanCapacity);
	vTaskResume(taskHandle_environmentalSensor);

	Serial.println();
	Serial.println("初始化完成");
	Serial.println();
}


void loop() {
	wifiConnect.loop();
	customWebSocketServer.loop();


	if (Serial.available()) {
		String inputString = Serial.readStringUntil('\n');
		inputString.trim();

		Serial.printf("輸入: %s\n", inputString);
		std::vector<std::string> command = split(inputString.c_str(), ".");

		if (inputString == "help") {
			Serial.printf("%s\n%s\n%s\n%s\n",
				"可用指令:",
				"help - 顯示此說明",
				"database.resetListener - 重新設置資料庫更新監聽",
				"envSensor.help - 顯示說明"
			);
		}

		if (command[0] == "database") {
			databaseClient.handleSerialCommands(command);
		}

		if (command[0] == "envSensor") {
			environmentalSensor.handleSerialCommands(command);
		}
	}
}


const std::vector<std::string> split(const std::string& str, const std::string& pattern) {
    std::vector<std::string> result;
    std::string::size_type end = str.find(pattern);
    std::string::size_type begin = 0;

    while (end != std::string::npos) {
        if (end - begin != 0) {
            result.push_back(str.substr(begin, end-begin)); 
        }    
        begin = end + pattern.size();
        end = str.find(pattern, begin);
    }

    if (begin != str.length()) {
        result.push_back(str.substr(begin));
    }
    return result;        
}


void taskInit() {
	vTaskResume(taskHandle_databaseClient);
	delay(100);
	vTaskResume(taskHandle_trashCanLight);
	vTaskResume(taskHandle_trashCanLid);
	vTaskResume(taskHandle_trashCanCapacity);
	delay(100);
	vTaskResume(taskHandle_environmentalSensor);

	// 等待環境感測器初始化完成(不管成功或失敗)
	while (environmentalSensor.isAvailable() == -1) {
		delay(10);
	}
}


void task_databaseClient(void * parameter) {
	vTaskSuspend(taskHandle_databaseClient);

	// 資料庫
	databaseClient.setup();
	database = databaseClient.getRealtimeDatabase();

	vTaskSuspend(taskHandle_databaseClient);

	while(databaseClient.isAvailable()) {
		databaseClient.loop();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void task_trashCanLight(void * parameter) {
	vTaskSuspend(taskHandle_trashCanLight);
	
	// 垃圾桶燈
	trashCanLight.setup();

	vTaskSuspend(taskHandle_trashCanLight);

	while (trashCanLight.isAvailable()) {
		trashCanLight.loop();
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void task_trashCanLid(void * parameter) {
	vTaskSuspend(taskHandle_trashCanLid);
	
	// 垃圾桶蓋
	trashCanLid.setup();

	vTaskSuspend(taskHandle_trashCanLid);

	while (trashCanLid.isAvailable()) {
		trashCanLid.loop();
		// 不用延遲(已經寫在loop裡)
	}
}

void task_trashCanCapacity(void * parameter) {
	vTaskSuspend(taskHandle_trashCanCapacity);

	// 垃圾桶容量
	trashCanCapacity.setup();

	vTaskSuspend(taskHandle_trashCanCapacity);

	while (trashCanCapacity.isAvailable()) {
		trashCanCapacity.loop();
		// 不用延遲(已經寫在loop裡)
	}
}

void task_environmentalSensor(void * parameter) {
	vTaskSuspend(taskHandle_environmentalSensor);
	
	// 環境感應器
	environmentalSensor.setup();

	vTaskSuspend(taskHandle_environmentalSensor);

	while (environmentalSensor.isAvailable() == 0) {
		environmentalSensor.loop();
		// 不用延遲(已經寫在loop裡)
	}
	
	vTaskDelete(NULL);
}



