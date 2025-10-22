

#ifndef SMARTTRASHCAN_CUSTOMWEBSOCKETSERVER_H
#define SMARTTRASHCAN_CUSTOMWEBSOCKETSERVER_H


class CustomWebSocketServer {

    public:
		static CustomWebSocketServer& getInstance();

        void setup();
        void loop();

		bool isAvailable();

    private:
		CustomWebSocketServer();
		CustomWebSocketServer(const CustomWebSocketServer&) = delete;
		CustomWebSocketServer& operator=(const CustomWebSocketServer&) = delete;

		bool available = false;
};

extern CustomWebSocketServer& customWebSocketServer;

#endif
