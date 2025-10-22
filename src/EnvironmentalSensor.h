#include <string>
#include <vector>


#ifndef SMARTTRASHCAN_ENVIRONMENTALSENSOR_H
#define SMARTTRASHCAN_ENVIRONMENTALSENSOR_H


class EnvironmentalSensor {

    public:
		static EnvironmentalSensor& getInstance();

        void setup();
        void loop();

		int isAvailable();

		void handleSerialCommands(std::vector<std::string> commands);

    private:
		EnvironmentalSensor();
		EnvironmentalSensor(const EnvironmentalSensor&) = delete;
		EnvironmentalSensor& operator=(const EnvironmentalSensor&) = delete;

		int available = -1;
};

extern EnvironmentalSensor& environmentalSensor;

#endif
