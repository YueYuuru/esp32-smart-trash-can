#include <string>
#include <vector>


#ifndef SMARTTRASHCAN_ENVIRONMENTALSENSOR_H
#define SMARTTRASHCAN_ENVIRONMENTALSENSOR_H


class EnvironmentalSensor {

    public:
		static EnvironmentalSensor& getInstance();

        bool setup();
        void loop();

		void handleSerialCommands(std::vector<std::string> commands);

    private:
		EnvironmentalSensor();
		EnvironmentalSensor(const EnvironmentalSensor&) = delete;
		EnvironmentalSensor& operator=(const EnvironmentalSensor&) = delete;
};

extern EnvironmentalSensor& environmentalSensor;

#endif
