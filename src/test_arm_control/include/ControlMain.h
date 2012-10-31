#ifndef CONTROLLERMAIN_H
#define CONTROLLERMAIN_H

/** @file
  * @brief The main class for this test controller.
  * This should be called periodically with new sensor data and to compute new outputs.
  */

#include <ControllerStructs.h>

namespace osurc {

namespace controller {

class ControllerMain {
	public:
		/** @brief Feed the controller with new sensor data.
		  * @param new_sensor_data Information on the latest sensor readings, including a timestamp.
		  * This should be called whenever new sensor readings are available. Preferably called before,
		  * not after, new outputs are requested, unless the sensor reading and output setting are
		  * done independently.
		  */
		void newSensorData(SensorData new_sensor_data);

		/** @brief Calculate new controller outputs.
		  * @param timestamp The time, in nanoseconds, at which the outputs are expected to be set.
		  * @return The new commanded outputs.
		  */
		ControllerOutput calcOutput(int64_t timestamp);
};

}

}

#endif // CONTROLLERMAIN_H
