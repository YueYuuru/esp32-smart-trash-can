#include <Arduino.h>


#include "CustomWebSocketServer.h"


CustomWebSocketServer::CustomWebSocketServer() {

}

CustomWebSocketServer& CustomWebSocketServer::getInstance() {
    static CustomWebSocketServer instance;
    return instance;
}

CustomWebSocketServer& customWebSocketServer = CustomWebSocketServer::getInstance();    // WebSocket 伺服器

void CustomWebSocketServer::setup() {

}


void CustomWebSocketServer::loop() {
    
}
