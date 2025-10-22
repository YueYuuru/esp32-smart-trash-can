#include <Arduino.h>
#include <WiFi.h>


#include "WifiConnect.h"
#include "DatabaseClient.h"


// 專案設定
#include "ProjectConfig.h"


bool writeIpAddress = false;


WifiConnect::WifiConnect() {
    
}

WifiConnect& WifiConnect::getInstance() {
    static WifiConnect instance;
    return instance;
}

WifiConnect& wifiConnect = WifiConnect::getInstance();    // Wifi

void WifiConnect::setup() {
	Serial.println("初始化 - Wifi");

	// 連接WiFi
	WiFi.begin(ProjectConfig::WIFI_SSID, ProjectConfig::WIFI_PASSWORD);

	Serial.println();
	Serial.print("Connecting to WiFi...");

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println();
	Serial.println("WiFi connected");
	Serial.print("EPS32 IP address: ");
	Serial.println(WiFi.localIP());

	writeIpAddress = false;

	available = true;
}

void WifiConnect::loop() {
	
	if (!writeIpAddress) {
		databaseClient.getInstance().getRealtimeDatabase().set<string_t>(databaseClient.getInstance().getAsyncClient(), "data/esp32/ip", string_t(WiFi.localIP()));
		writeIpAddress = true;
	}
    
}

bool WifiConnect::isAvailable() {
	return available;
}

