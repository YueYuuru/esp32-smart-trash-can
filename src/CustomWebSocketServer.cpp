#include <Arduino.h>
#include <vector>
#include <WebSocketsServer.h>


#include "CustomWebSocketServer.h"


// 儲存 client_id 的容器
std::vector<uint8_t> clients;

// WebSocket 伺服器實例
WebSocketsServer webSocket = WebSocketsServer(81);    // Port 81


void webSocketEvent(uint8_t client_id, WStype_t event_type, uint8_t *payload, size_t length);
void onReceivedMessage(String message);


CustomWebSocketServer::CustomWebSocketServer() {
	
}

CustomWebSocketServer& CustomWebSocketServer::getInstance() {
    static CustomWebSocketServer instance;
    return instance;
}

CustomWebSocketServer& customWebSocketServer = CustomWebSocketServer::getInstance();    // WebSocket 伺服器

void CustomWebSocketServer::setup() {
	Serial.println("初始化 - WebSocketClient");

	// 啟動 WebSocket 伺服器
	webSocket.begin();

	// 伺服器事件
	webSocket.onEvent(webSocketEvent);
	
	available = true;
}


void CustomWebSocketServer::loop() {
    // 每個循環要處理 WebSocket 的事件
	webSocket.loop();
}

bool CustomWebSocketServer::isAvailable() {
	return available;
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

