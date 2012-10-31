#ifndef CONTROLLERSTRUCTS_H
#define CONTROLLERSTRUCTS_H

/** @file
  * @brief Defines several structs to be used by controllers.
  */

namespace osurc {

namespace controller {

/** @brief Contains information about new sensor data.
  * @TODO: Figure out if all the data's coming in at the same time, or if it will be split up more.
  */
struct SensorData {
	int64_t timestamp
};

}

}

#endif // CONTROLLERSTRUCTS_H
