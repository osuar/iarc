#ifndef CONTROLLERSTRUCTS_H
#define CONTROLLERSTRUCTS_H

/** @file
  * @brief Defines several structs to be used by controllers.
  */

namespace osurc {

namespace controller {

/** @brief Contains information about new gyro data.
  */
struct GyroData {
	/** @brief The approximate time at which this data was taken.
	  * In ns.
	  */
	int64_t timestamp;

	/** @brief The rotational velocity about the X axes (robot-local).
	  * In rad/s
	  */
	double  xAngRate;
	double  yAngRate;
	double  zAngRate;
};

}

}

#endif // CONTROLLERSTRUCTS_H
