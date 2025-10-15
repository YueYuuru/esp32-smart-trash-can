

#ifndef SMARTTRASHCAN_CUSTOMWEBSOCKETSERVER_H
#define SMARTTRASHCAN_CUSTOMWEBSOCKETSERVER_H


class CustomWebSocketServer {

    public:
		static CustomWebSocketServer& getInstance();

        void setup();
        void loop();

    private:
		CustomWebSocketServer();
		CustomWebSocketServer(const CustomWebSocketServer&) = delete;
		CustomWebSocketServer& operator=(const CustomWebSocketServer&) = delete;
};

extern CustomWebSocketServer& customWebSocketServer;

#endif
