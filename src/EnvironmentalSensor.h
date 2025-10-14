

#ifndef SMARTTRASHCAN_ENVIRONMENTALSENSOR_H
#define SMARTTRASHCAN_ENVIRONMENTALSENSOR_H


class EnvironmentalSensor {

    public:
		static EnvironmentalSensor& getInstance();

        void setup();
        void loop();

    private:
		EnvironmentalSensor();
		EnvironmentalSensor(const EnvironmentalSensor&) = delete;
		EnvironmentalSensor& operator=(const EnvironmentalSensor&) = delete;
};

extern EnvironmentalSensor& environmentalSensor;

#endif
