

#ifndef SMARTTRASHCAN_TRASHCANLID_H
#define SMARTTRASHCAN_TRASHCANLID_H


class TrashCanLid {

    public:
		static TrashCanLid& getInstance();

        void setup();
        void loop();

		bool isAvailable();
		
        void setOpen(bool enable);

    private:
		TrashCanLid();
		TrashCanLid(const TrashCanLid&) = delete;
		TrashCanLid& operator=(const TrashCanLid&) = delete;

		bool available = false;

        bool enable;
};

extern TrashCanLid& trashCanLid;

#endif
