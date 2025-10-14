

#ifndef SMARTTRASHCAN_WIFICONNECT_H
#define SMARTTRASHCAN_WIFICONNECT_H


class WifiConnect {

    public:
        static WifiConnect& getInstance();

        void setup();
        void loop();

    private:
        WifiConnect();
		WifiConnect(const WifiConnect&) = delete;
		WifiConnect& operator=(const WifiConnect&) = delete;
};

extern WifiConnect& wifiConnect;

#endif
