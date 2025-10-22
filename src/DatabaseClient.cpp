#include <Arduino.h>
#include <string>
#include <vector>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>


#define ENABLE_USER_AUTH
#define ENABLE_DATABASE


#include <FirebaseClient.h>


#include "DatabaseClient.h"
#include "TrashCanLight.h"
#include "TrashCanLid.h"


// 專案設定
#include "ProjectConfig.h"


// 資料庫 callback 設置
bool databaseSetListener = false;


void processData(AsyncResult &aResult);
void processData_initializeApp(AsyncResult &aResult);


UserAuth user_auth(ProjectConfig::Web_API_KEY, ProjectConfig::USER_EMAIL, ProjectConfig::USER_PASSWORD);

FirebaseApp app;
WiFiClientSecure ssl_client;

using AsyncClient = AsyncClientClass;
AsyncClient async_client(ssl_client);

RealtimeDatabase Database;
AsyncResult dbResult;


// TrashCanLight& trashCanLight = TrashCanLight::getInstance();
// TrashCanLid& trashCanLid = TrashCanLid::getInstance();


DatabaseClient::DatabaseClient() {

}

DatabaseClient& DatabaseClient::getInstance() {
    static DatabaseClient instance;
    return instance;
}

DatabaseClient& databaseClient = DatabaseClient::getInstance();    // 資料庫

void DatabaseClient::setup() {
	Serial.println("初始化 - 資料庫");

	ssl_client.setInsecure();

	ssl_client.setConnectionTimeout(1000);
	ssl_client.setHandshakeTimeout(5);


	initializeApp(async_client, app, getAuth(user_auth), processData_initializeApp, "授權任務(authTask)");

	app.getApp<RealtimeDatabase>(Database);

	Database.url(ProjectConfig::DATABASE_URL);
	
	available = true;
}

RealtimeDatabase& DatabaseClient::getRealtimeDatabase() {
	return Database;
}

AsyncClient& DatabaseClient::getAsyncClient() {
	return async_client;
}

void DatabaseClient::loop() {

	app.loop();

	if (app.ready() && !databaseSetListener) {
		databaseSetListener = true;

		// Database.set<boolean_t>(async_client, "data/state/light", boolean_t(false)); // 客戶端 路徑 值
		
		Database.get(async_client, "data/state", processData, true); // 客戶端 路徑 呼叫處理資料的函數或給予數值給該變數 使用stream
	}
}

bool DatabaseClient::isAvailable() {
	return available;
}

void DatabaseClient::handleSerialCommands(std::vector<std::string> commands) {
	String command = String((commands[1]).c_str());

	if (command == "resetListener") {
		databaseSetListener = false;
	}
}

void processData(AsyncResult &asyncResult) {
	if (asyncResult.isResult() && asyncResult.available()) {
		RealtimeDatabaseResult& resultStream = asyncResult.to<RealtimeDatabaseResult>();
		if (resultStream.isStream()) {
			/*
			Serial.println("------------start------------");
			Firebase.printf("task: %s\n", asyncResult.uid().c_str());
			Firebase.printf("event: %s\n", resultStream.event().c_str());
			Firebase.printf("path: %s\n", resultStream.dataPath().c_str());
			Firebase.printf("data: %s\n", resultStream.to<const char *>());
			Firebase.printf("type: %d\n", resultStream.type());
			Serial.println("-------------end-------------");
			*/
			
			if (resultStream.event().equals("put") && resultStream.type() == realtime_database_data_type_json) {
				Serial.println("------------start json parse------------");
				// const char* json = ("%s", resultStream.to<const char *>());
				JsonDocument jsonDocument;
				// Serial.printf("json: %s\n", json);
				DeserializationError deserializationError = deserializeJson(jsonDocument, resultStream.to<const char *>());

				if (deserializationError) {
					Serial.print("deserializeJson() failed: ");
					Serial.println(deserializationError.f_str());
					return;
				}

				bool light = jsonDocument["light"];
				bool cover = jsonDocument["cover"];

				trashCanLight.setEnable(light);
				trashCanLid.setOpen(cover);

				Serial.println();
				Serial.printf("light = %s\n", light ? "true" : "false");
				Serial.printf("cover = %s\n", cover ? "true" : "false");
				Serial.println();
				Serial.println("-------------end json parse-------------");
			}
		}
	}
}


void processData_initializeApp(AsyncResult &asyncResult) {
	
	if (asyncResult.isEvent()) {
		Firebase.printf("Event task: %s, code: %d, msg: %s", asyncResult.uid().c_str(), asyncResult.eventLog().code(), asyncResult.eventLog().message().c_str());
		Firebase.printf("\n");
	}

	if (asyncResult.isDebug()) {
		Firebase.printf("Debug task: %s, code: %d, msg: %s", asyncResult.uid().c_str(), asyncResult.debugLog().code(), asyncResult.debugLog().message().c_str());
		Firebase.printf("\n");
	}

	if (asyncResult.isError()) {
		Firebase.printf("Error task: %s, code: %d, msg: %s", asyncResult.uid().c_str(), asyncResult.error().code(), asyncResult.error().message().c_str());
		Firebase.printf("\n");
	}
}
