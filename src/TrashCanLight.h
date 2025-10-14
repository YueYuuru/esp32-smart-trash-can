

#ifndef SMARTTRASHCAN_TRASHCANLIGHT_H
#define SMARTTRASHCAN_TRASHCANLIGHT_H


class TrashCanLight {

    public:
		static TrashCanLight& getInstance();

        void setup();
        void loop();
        void setEnable(bool enable);

    private:
		TrashCanLight();
		TrashCanLight(const TrashCanLight&) = delete;
		TrashCanLight& operator=(const TrashCanLight&) = delete;

        bool enable;
};

extern TrashCanLight& trashCanLight;

#endif
