

#ifndef SMARTTRASHCAN_TRASHCANCAPACITY_H
#define SMARTTRASHCAN_TRASHCANCAPACITY_H


class TrashCanCapacity {

    public:
		static TrashCanCapacity& getInstance();

        void setup();
        void loop();

    private:
		TrashCanCapacity();
		TrashCanCapacity(const TrashCanCapacity&) = delete;
		TrashCanCapacity& operator=(const TrashCanCapacity&) = delete;
};

extern TrashCanCapacity& trashCanCapacity;

#endif
